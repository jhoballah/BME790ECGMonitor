
// define all constants 
const float brachyLim = 40.0;
const float tachyLim = 180.0;
const int MAXVOLT = 5;
const long MILLIS_TO_SEC = 1000;
const int SEC_TO_MIN = 60;
const long MIN_IN_MILLIS = 60000;

// set input signal pins
const int diagnosticLED = 12;
const int ecgAnalogSignalPin = A5;
const int ecgDigitalSignalPin = 10;

// initialize counters for analog and digital HR monitor functions
int analog_beat_counter = 0;
int digital_beat_counter = 0;

// initialize all times for calculating beat intervals
unsigned long currentAnalogMillis = 0;
unsigned long currentDigitalMillis = 0;
unsigned long previousTimeAnalog = 0;
unsigned long previousTimeDigital = 0;
unsigned long startTimeDigital = 0;
unsigned long startTimeAnalog = 0;

// set intervals for the LED blinking
unsigned long tachyInterval = 50;
unsigned long brachyInterval = 250;
unsigned long previousLEDMillis = 0;

// initialize analog signal reading and voltage values
float ecgAnalogSignal = 0;
float voltage = 0;
float prev_voltage = 0;

// set threshold for analog signal based upon analog output, can be modified as needed
float analog_threshold = 5*750/1023;

// set a minimum time threshold for beat interval times to exclude false beats
int timeThreshold = 200;

// initialize digital signal values
int ecgDigitalSignal = HIGH;
int prev_ecgDigitalSignal = HIGH;

// initialize pause state pin and state
const byte pausePin = 2;
volatile byte pauseState = LOW;


// initialize all hr variables
float analog_inst_hr = 0;
float digital_inst_hr = 0;
float analog_avg_hr_total = 0;
float digital_avg_hr_total = 0;
float hrVal = 0;

void pause();
int ledState = LOW;

void setup() {
  // put your setup code here, to run once:

  ledSetup();
  signalReadSetup();
  
  // add pause interrupt
  attachInterrupt(digitalPinToInterrupt(pausePin), pause, CHANGE);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  if (pauseState != HIGH){
    
    //readECGDigital();
    //digitalHRMonitor(currentDigitalMillis);  
    
    readECGAnalog();
    analogHRMonitor(currentAnalogMillis);
    
    // performs the diagnosis LED blinking once an analog beat is detected
    if (analog_beat_counter > 0) {
      diagnosis(analog_inst_hr);  
    }
    
  }
  else {
    delay(10000);
  }
}

void ledSetup() {
  // initialize LED for output
  pinMode(diagnosticLED, OUTPUT); 
}

void signalReadSetup() {
  // Prepare pins as inputs for analog and digital signals
  pinMode(ecgAnalogSignalPin, INPUT);
  pinMode(ecgDigitalSignalPin, INPUT);
}



void readECGAnalog() {
  // read analog signal from analog signal pin
  ecgAnalogSignal = analogRead(ecgAnalogSignalPin);

  voltage = MAXVOLT*ecgAnalogSignal/1023;
  
  //begin timer
  currentAnalogMillis = millis();
  //Serial.println(voltage);
  
}

void readECGDigital() {

  // read digital signal from digital signal pin
  ecgDigitalSignal = digitalRead(ecgDigitalSignalPin);
  currentDigitalMillis = millis();
  //Serial.println(digitalSignalState);
}


void analogHRMonitor(unsigned long currentAnalogMillis) {
  // check if voltage is above a specific threshold, can be modified from analog_threshold var at top of code
  if (voltage < analog_threshold && prev_voltage > analog_threshold ) {
    if (currentAnalogMillis - previousTimeAnalog >= timeThreshold) {
      // update counter when monitor finds that a beat has been measured
      analog_beat_counter++;
      float analog_inst_hr = {1./ ((currentAnalogMillis - previousTimeAnalog) / MILLIS_TO_SEC)*SEC_TO_MIN};  
      hrVal = analog_inst_hr;
      analog_avg_hr_total = analog_avg_hr_total + analog_inst_hr;
      
    }
      previousTimeAnalog = currentAnalogMillis;
  }
  
  // if the current analog millis time is greater than one minute, begin calculating the rolling average
  // currently updates at each minute. 
  if (currentAnalogMillis - startTimeAnalog >= (MIN_IN_MILLIS)) {
    float analog_avg_hr_val = (analog_avg_hr_total / analog_beat_counter);
    
    //Serial.println(analog_avg_hr_val);
    startTimeAnalog = currentAnalogMillis;
    analog_beat_counter = 0;
    analog_avg_hr_total = 0;
    
  }
  prev_voltage = voltage;
}

void digitalHRMonitor(unsigned long currentDigitalMillis) {
  // check if digital signal has reached the peak
  if (ecgDigitalSignal  == LOW && prev_ecgDigitalSignal == HIGH) {
    if (currentDigitalMillis - previousTimeDigital >= timeThreshold) {
      // update counter when monitor finds that a beat has been measured
      digital_beat_counter++;
      float digital_inst_hr = {1./ ((currentDigitalMillis - previousTimeDigital) / MILLIS_TO_SEC)*SEC_TO_MIN}; 
      digital_avg_hr_total = digital_avg_hr_total + digital_inst_hr;
    }
    previousTimeDigital = currentDigitalMillis;  
  }

  // if the current digital millis time is greater than one minute, begin calculating the rolling average
  // currently updates at each minute. 
  if (currentDigitalMillis - startTimeDigital >= (MIN_IN_MILLIS)) {
    float digital_avg_hr_val = (digital_beat_counter / 60);
    //Serial.println(digital_avg_hr_val);
    startTimeDigital = currentDigitalMillis;
    digital_beat_counter = 0;
    digital_avg_hr_total = 0;
  
  }
  prev_ecgDigitalSignal = ecgDigitalSignal;
}


// The diagnosis section needs to be verified once the avgHR values are working as intended
void diagnosis(float hrVal) {

  if (hrVal < brachyLim) {
    blinkLED(brachyInterval);
  }
  else if (hrVal > tachyLim) {
    blinkLED(tachyInterval);
  }
  else 
    digitalWrite(diagnosticLED, LOW);
  
  
}

// Need to verify if the LED blink without delay works as expected
void blinkLED(unsigned long interval) {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousLEDMillis >= interval) {
    // save the last time you blinked the LED
    previousLEDMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } 
    else {
      ledState = LOW;
    }
    // set the LED with the ledState of the variable:
    digitalWrite(diagnosticLED, ledState);
    }
}

void pause() {
pauseState = ! pauseState;
  
}

