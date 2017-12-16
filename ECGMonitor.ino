#include "RunningAverage.h"

// define all constants 
const float brachyLim = 40.0;
const float tachyLim = 180.0;
const int rolling_average_size = 30;
const int MAXVOLT = 5;
const long MILLIS_TO_SEC = 1000;
const int SEC_TO_MIN = 60;
const long MIN_IN_MILLIS = 60000;
const int TIME_THRESHOLD = 200;

// set input signal pins
const int diagnosticLED = 12;
const int ecgAnalogSignalPin = A5;
const int ecgDigitalSignalPin = 10;

int ledState = LOW;

// initialize counters for analog and digital HR monitor functions
int analog_beat_counter = 0;
int analog_inst_hr_count = 0;
int digital_beat_counter = 0;
int digital_inst_hr_count = 0;

// initialize all times for calculating beat intervals
unsigned long currentAnalogMillis = 0;
unsigned long currentDigitalMillis = 0;
unsigned long previousTimeAnalog = 0;
unsigned long previousTimeDigital = 0;
unsigned long startTimeDigital = 0;
unsigned long startTimeAnalog = 0;
unsigned long tachyInterval = 50;
unsigned long brachyInterval = 250;
unsigned long previousLEDMillis = 0;

// initialize analog signal reading and voltage values
float ecgAnalogSignal = 0;
float voltage = 0;
float prev_voltage = 0;

// set threshold for analog signal based upon analog output, can be modified as needed
float analog_threshold = 5*860/1023;

// initialize digital signal values
int ecgDigitalSignal = HIGH;
int prev_ecgDigitalSignal = HIGH;

// initialize pause state pin and state
const byte pausePin = 2;
volatile byte pauseState = LOW;

// heart rate value used for diagnosis of tachycardia and brachycardia
float hrVal = 0;

// holds the analog_avg_hr using the RunningAverage library
RunningAverage analog_avg_hr(60);
RunningAverage digital_avg_hr(60);

void pause();

void setup() {
  // put your setup code here, to run once:

  ledSetup();
  signalReadSetup();
  avgHRArraySetup();
  
  // add pause interrupt
  attachInterrupt(digitalPinToInterrupt(pausePin), pause, HIGH);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (pauseState != HIGH){
    
    readECGAnalog();
    analogHRMonitor(currentAnalogMillis);

    readECGDigital();
    digitalHRMonitor(currentDigitalMillis);  
    if (hrVal != 0) {
      diagnosis(hrVal);  
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

void avgHRArraySetup() {

  // clear arrays to make sure that they start without any values inside
  analog_avg_hr.clear();
  digital_avg_hr.clear();
  
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
    // update counter when monitor finds that a beat has been measured
    if ((currentAnalogMillis - previousTimeAnalog) >= TIME_THRESHOLD) {
      analog_beat_counter++;
      float analog_interval_hr[analog_beat_counter] = {1* MIN_IN_MILLIS / (currentAnalogMillis - previousTimeAnalog)};
  
      // update instantaneous heart rate count when there are at least two beats identified
      if (analog_beat_counter >= 2) {
        analog_inst_hr_count++;
        //calculate instantaneous heart rate based upon two beats
        float analog_inst_hr[analog_inst_hr_count] = {(analog_interval_hr[1] + analog_interval_hr[2])/2};
        
        //check to ensure that the instHR value is not infinity to avoid an infinite average value
        if (analog_inst_hr[analog_inst_hr_count] != INFINITY) {
          
          //send the analog_inst_hr value to the analog_avg_hr array
          analog_avg_hr.addValue(analog_inst_hr[analog_inst_hr_count]);
          Serial.print("Analog Inst HR: ");
          Serial.println(analog_inst_hr[analog_inst_hr_count]);
        }
        
      }
    
      previousTimeAnalog = currentAnalogMillis;
    }
    
  }

  // if the current analog millis time is greater than one minute, begin calculating the rolling average
  // currently updates at each minute. Can be updated to calculate it at each new beat
  if (currentAnalogMillis - startTimeAnalog >= (MIN_IN_MILLIS)) {
    float analog_avg_hr_val = analog_avg_hr.getAverage();
    hrVal = analog_avg_hr_val;
    Serial.print("Analog AVGHR: ");
    Serial.println(analog_avg_hr_val);
    startTimeAnalog = currentAnalogMillis;
    //clear array after each minute to ensure it doesn't overflow over time
    analog_avg_hr.clear();
    
    
  }
  prev_voltage = voltage;
}

void digitalHRMonitor(unsigned long currentDigitalMillis) {
  if (ecgDigitalSignal  == LOW && prev_ecgDigitalSignal == HIGH) {
    if ((currentDigitalMillis - previousTimeDigital) >= TIME_THRESHOLD){
      // update counter when monitor finds that a beat has been measured
      digital_beat_counter++;
      float digital_interval_hr[digital_beat_counter] = {1* MIN_IN_MILLIS / (currentDigitalMillis - previousTimeDigital)};
  
      // update instantaneous heart rate count when there are at least two beats identified
      if (digital_beat_counter >= 2) {
        digital_inst_hr_count++;
        //calculate instantaneous heart rate based upon two beats
        float digital_inst_hr[digital_inst_hr_count] = {(digital_interval_hr[1] + digital_interval_hr[2])/2};

        //check to ensure that the instHR value is not infinity to avoid an infinite average value
        if (digital_inst_hr[digital_inst_hr_count] != INFINITY) {
          //send the digital_inst_hr value to the digital_avg_hr array
          digital_avg_hr.addValue(digital_inst_hr[digital_inst_hr_count]);
          Serial.print("Digital InstHR: ");
          Serial.println(digital_inst_hr[digital_inst_hr_count]);
          
        }
        
      }
  
    previousTimeDigital = currentDigitalMillis;
    }  
  }

  // if the current digital millis time is greater than one minute, begin calculating the rolling average
  // currently updates at each minute. 
  if (currentDigitalMillis - startTimeDigital >= (MIN_IN_MILLIS)) {
    float digital_avg_hr_val = digital_avg_hr.getAverage();
    Serial.print("Digital AVGHR: ");
    Serial.println(digital_avg_hr_val);
    startTimeDigital = currentDigitalMillis;
    //clear array after each minute to ensure it doesn't overflow over time
    digital_avg_hr.clear();
    

  
  }
  prev_ecgDigitalSignal = ecgDigitalSignal;
}


// Needs to be tested
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

