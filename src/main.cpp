#include <Arduino.h>
#include <math.h>

// ADC pins and configuration
const int voltagePin         = 35;
const int currentPin         = 34;
const float voltageReference = 3.3;
const int ADCResolution      = 4095;

// Relay pins for capacitor and inductor banks
enum RelayPins { CAP_RELAY = 25, IND_RELAY = 26 };

// Global calibration variables
float zeroOffsetVoltage = 0.0;
float zeroOffsetCurrent = 0.0;

// Scaling and sensitivity factors
const float voltageScalingFactor = 0.723;
const float currentSensitivity   = 0.185;

// AC parameters
const float acFrequency  = 50.0;
const unsigned long periodMicros = (unsigned long)(1000000 / acFrequency);

// Measurement and filtering settings
// Increased NUM_CYCLES to average over more zero-crossings to reduce jitter
typedef struct { int cycles; float alpha; } MeasurementSettings;
MeasurementSettings meas = { 250, 0.1 };
float filteredVoltage = 0;
float filteredCurrent = 0;

// Median filter settings for phase angle
const int ANGLE_MEDIAN_SIZE = 5;
float angleBuffer[ANGLE_MEDIAN_SIZE] = {0};
int   angleIndex = 0;

// Power factor thresholds for hysteresis
typedef struct { float engageAngle; float disengageAngle; } AngleThresholds;
// Capacitor: engage at +36.87°, disengage at +18.19°
// Inductor: engage at -40°, disengage at +5°
AngleThresholds capThresh = { acos(0.80)*180.0/PI, acos(0.95)*180.0/PI };
AngleThresholds indThresh = { -40.0, 5.0 };

// State machine for power factor correction
enum PFCState { MONITORING, CAP_CORRECTION, IND_CORRECTION, CALIBRATING };
PFCState currentState = MONITORING;

//--------------------- Helpers ---------------------
float medianAngle(float *buf, int size) {
  float temp[ANGLE_MEDIAN_SIZE];
  for (int i = 0; i < size; i++) temp[i] = buf[i];
  // Simple insertion sort for small array
  for (int i = 1; i < size; i++) {
    float key = temp[i];
    int j = i - 1;
    while (j >= 0 && temp[j] > key) {
      temp[j+1] = temp[j];
      j--;
    }
    temp[j+1] = key;
  }
  return temp[size/2];
}

void updatePFCState(float phaseAngle) {
  switch (currentState) {
    case MONITORING:
      if (phaseAngle > capThresh.engageAngle) {
        digitalWrite(CAP_RELAY, HIGH);
        digitalWrite(IND_RELAY, LOW);
        currentState = CAP_CORRECTION;
        Serial.println("Capacitor Bank ENGAGED");
      } else if (phaseAngle < indThresh.engageAngle) {
        digitalWrite(IND_RELAY, HIGH);
        digitalWrite(CAP_RELAY, LOW);
        currentState = IND_CORRECTION;
        Serial.println("Inductor Bank ENGAGED");
      }
      break;
    case CAP_CORRECTION:
      if (phaseAngle < capThresh.disengageAngle) {
        digitalWrite(CAP_RELAY, LOW);
        currentState = MONITORING;
        Serial.println("Capacitor Bank DISENGAGED");
      }
      break;
    case IND_CORRECTION:
      if (phaseAngle > indThresh.disengageAngle) {
        digitalWrite(IND_RELAY, LOW);
        currentState = MONITORING;
        Serial.println("Inductor Bank DISENGAGED");
      }
      break;
    case CALIBRATING:
      break;
  }
}

void calibrateSensors() {
  Serial.println("Starting calibration. Ensure NO LOAD is connected...");
  long vSum = 0, iSum = 0;
  const int samples = 1000;
  for (int i = 0; i < samples; i++) {
    vSum += analogRead(voltagePin);
    iSum += analogRead(currentPin);
    delay(1);
  }
  zeroOffsetVoltage = (float)vSum / samples;
  zeroOffsetCurrent = (float)iSum / samples;
  Serial.printf("Calibration complete. Voltage offset: %.2f, Current offset: %.2f\n", zeroOffsetVoltage, zeroOffsetCurrent);
}

void updateEMA(float &filt, int raw, float a) {
  filt = a * raw + (1 - a) * filt;
}

float measureRMS(int pin, float offset, int count, bool isVoltage) {
  double sumSq = 0;
  for (int i = 0; i < count; i++) {
    int r = analogRead(pin);
    float v = (r - offset)*(voltageReference/ADCResolution);
    sumSq += v*v;
    delayMicroseconds(100);
  }
  float rms = sqrt(sumSq/count);
  return isVoltage ? rms*voltageScalingFactor : rms/currentSensitivity;
}

//--------------------- Setup & Loop ---------------------
void setup() {
  Serial.begin(115200);
  analogSetAttenuation(ADC_11db);
  pinMode(CAP_RELAY, OUTPUT);
  pinMode(IND_RELAY, OUTPUT);
  digitalWrite(CAP_RELAY, LOW);
  digitalWrite(IND_RELAY, LOW);
  calibrateSensors();
  filteredVoltage = analogRead(voltagePin);
  filteredCurrent = analogRead(currentPin);
  Serial.println("Enhanced Synchronous PF Correction Starting (Median Filtering)");
}

void loop() {
  long sumDt = 0;
  for (int c = 0; c < meas.cycles; c++) {
    unsigned long tV = 0, tI = 0;
    bool vCross = false, iCross = false;
    int pV = analogRead(voltagePin);
    updateEMA(filteredVoltage, pV, meas.alpha);
    while (!vCross) {
      int rV = analogRead(voltagePin);
      updateEMA(filteredVoltage, rV, meas.alpha);
      if (pV < zeroOffsetVoltage && filteredVoltage >= zeroOffsetVoltage) {
        tV = micros(); vCross = true;
      }
      pV = filteredVoltage;
    }
    int pI = analogRead(currentPin);
    updateEMA(filteredCurrent, pI, meas.alpha);
    while (!iCross) {
      int rI = analogRead(currentPin);
      updateEMA(filteredCurrent, rI, meas.alpha);
      if (pI < zeroOffsetCurrent && filteredCurrent >= zeroOffsetCurrent) {
        tI = micros(); iCross = true;
      }
      pI = filteredCurrent;
    }
    sumDt += (long)(tI - tV);
    delay(2);
  }
  float avgDt = (float)sumDt / meas.cycles;
  float angle = (avgDt/periodMicros)*360.0;
  while (angle>180) angle-=360;
  while (angle<-180) angle+=360;
  // median filter
  angleBuffer[angleIndex] = angle;
  angleIndex = (angleIndex+1)%ANGLE_MEDIAN_SIZE;
  float medAngle = medianAngle(angleBuffer, ANGLE_MEDIAN_SIZE);

  float Vrms = measureRMS(voltagePin, zeroOffsetVoltage, 1000, true);
  float Irms = measureRMS(currentPin, zeroOffsetCurrent, 1000, false);
  float pf = cos(medAngle*PI/180.0);
  float P  = Vrms*Irms*pf;
  float S  = Vrms*Irms;
  float Q  = sqrt(S*S - P*P);

  Serial.printf("Raw Angle: %.1f°, Median Angle: %.1f°, PF: %.3f\n", angle, medAngle, pf);
  Serial.printf("Vrms: %.2f V, Irms: %.3f A | P: %.2f W, S: %.2f VA, Q: %.2f VAR\n", Vrms, Irms, P, S, Q);

  if (Vrms < 0.1) {
    digitalWrite(CAP_RELAY, LOW);
    digitalWrite(IND_RELAY, LOW);
    currentState = MONITORING;
    Serial.println("AC OFF detected: Resetting PF correction state");
  } else {
    updatePFCState(medAngle);
  }
  delay(2000);
}
