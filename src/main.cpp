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
AngleThresholds capThresh = { acos(0.80)*180.0/PI, acos(0.95)*180.0/PI };
// Inductor: engage at -40°, disengage at +10° (wider gap)
AngleThresholds indThresh = { -40.0, 10.0 };

// State machine for power factor correction
enum PFCState { MONITORING, CAP_CORRECTION, IND_CORRECTION, CALIBRATING };
PFCState currentState = MONITORING;

// —— NO-LOAD & DEBOUNCE CONFIGURATION ——
const float noLoadCurrent       = 0.13;       // A threshold to call “no load”
const float noLoadVoltage = 0.15;       // V threshold to call “no load”
const unsigned long noLoadDelay = 3000;      // ms before we reset on no-load
unsigned long noLoadSince       = 0;         // timer start for no-load
const int REQUIRED_STABLE_READINGS = 3;
int stableReadingCount = 0;

const unsigned long minSwitchInterval = 5000; // ms minimum between ANY relay action
unsigned long lastSwitchTime         = 0;    // last time we flipped a relay

//--------------------- Helpers ---------------------
float medianAngle(float *buf, int size) {
  float temp[ANGLE_MEDIAN_SIZE];
  memcpy(temp, buf, sizeof(temp));
  // Insertion sort
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

bool isAngleStable(float rawAngle, float medianAngle) {
  float difference = abs(rawAngle - medianAngle);
  
  // More strict stability check for angles near the thresholds
  if ((rawAngle > -50 && rawAngle < -35) || // Near inductor engage threshold
      (rawAngle > 5 && rawAngle < 15)) {    // Near inductor disengage threshold
    return difference < 20.0; // More strict near thresholds
  }
  
  return difference < 45.0; // Standard threshold elsewhere
}

void engageCap() {
  digitalWrite(CAP_RELAY, HIGH);
  digitalWrite(IND_RELAY, LOW);
  lastSwitchTime = millis();
  currentState = CAP_CORRECTION;
  Serial.println("Capacitor Bank ENGAGED");
}

void disengageCap() {
  digitalWrite(CAP_RELAY, LOW);
  lastSwitchTime = millis();
  currentState = MONITORING;
  Serial.println("Capacitor Bank DISENGAGED");
}

void engageInd() {
  digitalWrite(IND_RELAY, HIGH);
  digitalWrite(CAP_RELAY, LOW);
  lastSwitchTime = millis();
  currentState = IND_CORRECTION;
  Serial.println("Inductor Bank ENGAGED");
}

void disengageInd() {
  digitalWrite(IND_RELAY, LOW);
  lastSwitchTime = millis();
  currentState = MONITORING;
  Serial.println("Inductor Bank DISENGAGED");
}

void updatePFCState(float phaseAngle) {
  // Enforce minimum interval
  if (millis() - lastSwitchTime < minSwitchInterval) return;

  switch (currentState) {
    case MONITORING:
      if (phaseAngle > capThresh.engageAngle) {
        engageCap();
      } else if (phaseAngle < indThresh.engageAngle) {
        engageInd();
      }
      break;

    case CAP_CORRECTION:
      if (phaseAngle < capThresh.disengageAngle) {
        disengageCap();
      }
      break;

    case IND_CORRECTION:
      if (phaseAngle > indThresh.disengageAngle) {
        disengageInd();
      }
      break;

    case CALIBRATING:
      // no-op
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
  Serial.printf("Calibration complete. Voltage offset: %.2f, Current offset: %.2f\n",
                zeroOffsetVoltage, zeroOffsetCurrent);
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
  Serial.println("Enhanced Synchronous PF Correction Starting");
}

void loop() {
  // --- Measure phase angle via zero-cross timing ---
  long sumDt = 0;
  for (int c = 0; c < meas.cycles; c++) {
    unsigned long tV=0, tI=0;
    bool vCross=false, iCross=false;
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
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  angleBuffer[angleIndex] = angle;
  angleIndex = (angleIndex+1) % ANGLE_MEDIAN_SIZE;
  float medAngle = medianAngle(angleBuffer, ANGLE_MEDIAN_SIZE);

  // --- Calculate RMS & power stats ---
  float Vrms = measureRMS(voltagePin, zeroOffsetVoltage, 1000, true);
  float Irms = measureRMS(currentPin,  zeroOffsetCurrent, 1000, false);
  float pf   = cos(medAngle * PI/180.0);
  float P    = Vrms * Irms * pf;
  float S    = Vrms * Irms;
  float Q    = sqrt(S*S - P*P);

  Serial.printf("Raw Angle: %.1f°, Median Angle: %.1f°, PF: %.3f\n",
                angle, medAngle, pf);
  Serial.printf("Vrms: %.2f V, Irms: %.3f A | P: %.2f W, S: %.2f VA, Q: %.2f VAR\n",
                Vrms, Irms, P, S, Q);

  // --- No-load detection & reset ---
  if (Vrms < noLoadVoltage) {
    // Very low voltage - treat as no AC or calibration issue
    digitalWrite(CAP_RELAY, LOW);
    digitalWrite(IND_RELAY, LOW);
    currentState = MONITORING;
    lastSwitchTime = millis();
    Serial.println("Low voltage detected: resetting all banks");
    delay(2000);
    return;
  }

  // --- Check angle stability before making decisions ---
  if (!isAngleStable(angle, medAngle)) {
    Serial.println("Angle instability detected - skipping PFC update");
    stableReadingCount = 0; // Reset the counter
    delay(500);
    return;
  } else {
    stableReadingCount++;
    Serial.printf("Stable reading count: %d/%d\n", stableReadingCount, REQUIRED_STABLE_READINGS);
    
    // Only proceed with PFC update if we have enough consecutive stable readings
    if (stableReadingCount >= REQUIRED_STABLE_READINGS) {
      updatePFCState(medAngle);
    } else {
      Serial.println("Waiting for more stable readings before PFC update");
    }
  }

  delay(20);
}