#include <Average.h>
int brachyLim = 40;
int tachyLim = 180;
int rolling_average_size = 30;
int MAXVOLT = 5;
int MILLIS_TO_SEC = 1000;
int SEC_TO_MIN = 60;
int MIN_IN_MILLIS = 60000;

int diagnosticLED = 12;
int ecgAnalogSignalPin = A5;
int ecgDigitalSignalPin = 10;

int ledState = LOW;

int analog_beat_counter = 0;
int analog_inst_hr_count = 0;

unsigned long currentAnalogMillis = 0;
unsigned long previousTime = 0;
unsigned long startTime = 0;
unsigned long tachyInterval = 50;
unsigned long brachyInterval = 250;
unsigned long previousTachyMillis = 0;
unsigned long previousBrachyMillis = 0;

float ecgAnalogSignal = 0;
float voltage = 0;
float analog_threshold = .75;

const byte pausePin = 2;
volatile byte pauseState = LOW;

// holds the analog_avg_hr using the Average library
Average<float> analog_avg_hr(rolling_average_size);


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
    readECGAnalog();
    analogHRMonitor(currentAnalogMillis);
  }
  else {
    
  }
}

void ledSetup() {
  // initialize LEDs for output
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
  Serial.println(voltage);
  
}


void analogHRMonitor(unsigned long currentAnalogMillis) {
  // check if voltage is above a specific threshold, can be modified from analog_threshold var at top of code
  if (voltage > analog_threshold) {
    delay(250);
    // update counter when monitor finds that a beat has been measured
    analog_beat_counter++;
    float analog_interval_hr[analog_beat_counter] = {1./ ((currentAnalogMillis - previousTime) / MILLIS_TO_SEC)*SEC_TO_MIN};

    // update instantaneous heart rate count when there are at least two beats identified
    if (analog_beat_counter >= 2) {
      analog_inst_hr_count++;
      //calculate instantaneous heart rate
      float analog_inst_hr[analog_inst_hr_count] = {(analog_interval_hr[1] + analog_interval_hr[2])/2};

      //send the analog_inst_hr value to the analog_avg_hr array
      analog_avg_hr.push(analog_inst_hr[analog_inst_hr_count]);
      Serial.println(analog_inst_hr[analog_inst_hr_count]);
    }
  
    previousTime = currentAnalogMillis;
    
  }

  // if the current analog millis time is greater than one minute, begin calculating the rolling average
  // currently updates at each minute. Can be updated to calculate it at each new beat
  if (currentAnalogMillis - startTime >= (MILLIS_TO_SEC * SEC_TO_MIN)) {
    float analog_avg_hr_val = analog_avg_hr.mean();
    startTime = currentAnalogMillis;

  }
}

void readECGdigital() {
  

  
}


// Still need to fix diagnosis to blink LEDs appropriately
void diagnosis(float hrVal) {

  if (hrVal < brachyLim) {
    //blinkLED();
  }
  else if (hrVal > tachyLim) {
    //blinkLED();
  }
  else 
    digitalWrite(diagnosticLED, LOW);
  
  
}

// Need to verify if the LED blink without delay works as expected
void blinkLED(unsigned long previousMillis, unsigned long interval) {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval ) {
    previousMillis = currentMillis;

    // if LED was off, turn it on. If LED was on, turn it off.
    if (ledState == LOW)
      ledState == HIGH;
    else
      ledState == LOW;

    // set LED with current LED state
    digitalWrite(diagnosticLED, ledState);
    
  }
}

void pause() {
pauseState = ! pauseState;
  
}

