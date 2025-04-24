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
const int   NUM_CYCLES = 100;
const float alpha      = 0.1;
float filteredVoltage  = 0;
float filteredCurrent  = 0;

// Power factor thresholds for hysteresis
// Capacitor: engage at PF<0.80, disengage at PF>0.95
// Inductor: engage when phase angle < -40°, disengage when phase angle > +5°
typedef struct { float engagePF; float disengagePF; } PFThresholds;
PFThresholds capPF = { 0.80, 0.95 };

// Convert PF thresholds to phase angles (degrees) for capacitor
const float ANG_CAP_ENGAGE    = acos(capPF.engagePF)    * 180.0 / PI;  // +36.87°
const float ANG_CAP_DISENGAGE = acos(capPF.disengagePF) * 180.0 / PI;  // +18.19°

// Explicit hysteresis angles for inductor (leading side)
const float ANG_IND_ENGAGE    = -40.0; // engage at -40°
const float ANG_IND_DISENGAGE =  5.0;  // disengage at +5°

// State machine for power factor correction
enum PFCState {
  MONITORING,
  CAP_CORRECTION,
  IND_CORRECTION,
  CALIBRATING
};
PFCState currentState = MONITORING;

void updatePFCState(float phaseAngle) {
  switch (currentState) {
    case MONITORING:
      // Inductive lag -- engage capacitor
      if (phaseAngle > ANG_CAP_ENGAGE) {
        digitalWrite(CAP_RELAY, HIGH);
        digitalWrite(IND_RELAY, LOW);
        currentState = CAP_CORRECTION;
        Serial.println("Capacitor Bank ENGAGED");
      }
      // Capacitive lead -- engage inductor
      else if (phaseAngle < ANG_IND_ENGAGE) {
        digitalWrite(IND_RELAY, HIGH);
        digitalWrite(CAP_RELAY, LOW);
        currentState = IND_CORRECTION;
        Serial.println("Inductor Bank ENGAGED");
      }
      break;

    case CAP_CORRECTION:
      // PF recovered above disengage threshold -- disengage capacitor
      if (phaseAngle < ANG_CAP_DISENGAGE) {
        digitalWrite(CAP_RELAY, LOW);
        currentState = MONITORING;
        Serial.println("Capacitor Bank DISENGAGED");
      }
      break;

    case IND_CORRECTION:
      // PF recovered above leading disengage threshold -- disengage inductor
      if (phaseAngle > ANG_IND_DISENGAGE) {
        digitalWrite(IND_RELAY, LOW);
        currentState = MONITORING;
        Serial.println("Inductor Bank DISENGAGED");
      }
      break;

    case CALIBRATING:
      // No action during calibration
      break;
  }
}

// Calibration of zero offsets
void calibrateSensors() {
  Serial.println("Starting calibration. Ensure NO LOAD is connected...");
  long voltageSum = 0, currentSum = 0;
  const int samples = 1000;
  for (int i = 0; i < samples; i++) {
    voltageSum += analogRead(voltagePin);
    currentSum += analogRead(currentPin);
    delay(1);
  }
  zeroOffsetVoltage = (float)voltageSum / samples;
  zeroOffsetCurrent = (float)currentSum / samples;
  Serial.print("Calibration complete. Voltage offset: ");
  Serial.print(zeroOffsetVoltage);
  Serial.print(", Current offset: ");
  Serial.println(zeroOffsetCurrent);
}

// Exponential moving average filter
void updateEMA(float &filteredValue, int rawValue, float a) {
  filteredValue = a * rawValue + (1 - a) * filteredValue;
}

// RMS measurement for voltage
float measureVoltageRMS(int pin, float offset, int count) {
  double sumSq = 0;
  for (int i = 0; i < count; i++) {
    int reading = analogRead(pin);
    float v = (reading - offset) * (voltageReference / ADCResolution);
    sumSq += v * v;
    delayMicroseconds(100);
  }
  float VrmsSensor = sqrt(sumSq / count);
  return VrmsSensor * voltageScalingFactor;
}

// RMS measurement for current
float measureCurrentRMS(int pin, float offset, int count) {
  double sumSq = 0;
  for (int i = 0; i < count; i++) {
    int reading = analogRead(pin);
    float v = (reading - offset) * (voltageReference / ADCResolution);
    sumSq += v * v;
    delayMicroseconds(100);
  }
  float VrmsSensor = sqrt(sumSq / count);
  return VrmsSensor / currentSensitivity;
}

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
  Serial.println("Enhanced Synchronous PF Correction Starting (With Leading Hysteresis)");
}

void loop() {
  long sumDeltaT = 0;
  int cycles = 0;
  while (cycles < NUM_CYCLES) {
    unsigned long tV = 0, tI = 0;
    bool vCrossed = false, iCrossed = false;
    int prevV = analogRead(voltagePin);
    updateEMA(filteredVoltage, prevV, alpha);
    while (!vCrossed) {
      int rawV = analogRead(voltagePin);
      updateEMA(filteredVoltage, rawV, alpha);
      if (prevV < zeroOffsetVoltage && filteredVoltage >= zeroOffsetVoltage) {
        tV = micros(); vCrossed = true;
      }
      prevV = filteredVoltage;
    }
    int prevI = analogRead(currentPin);
    updateEMA(filteredCurrent, prevI, alpha);
    while (!iCrossed) {
      int rawI = analogRead(currentPin);
      updateEMA(filteredCurrent, rawI, alpha);
      if (prevI < zeroOffsetCurrent && filteredCurrent >= zeroOffsetCurrent) {
        tI = micros(); iCrossed = true;
      }
      prevI = filteredCurrent;
    }
    sumDeltaT += (long)(tI - tV);
    cycles++;
    delay(2);
  }
  float avgDt     = (float)sumDeltaT / NUM_CYCLES;
  float phaseAngle = (avgDt / periodMicros) * 360.0;
  while (phaseAngle > 180)  phaseAngle -= 360;
  while (phaseAngle < -180) phaseAngle += 360;
  float Vrms = measureVoltageRMS(voltagePin, zeroOffsetVoltage, 1000);
  float Irms = measureCurrentRMS(currentPin, zeroOffsetCurrent, 1000);
  float pf   = cos(phaseAngle * PI / 180.0);
  float P    = Vrms * Irms * pf;
  float S    = Vrms * Irms;
  float Q    = sqrt(S * S - P * P);
  Serial.printf("Avg ΔT: %.2f µs, Angle: %.1f°, PF: %.3f\n", avgDt, phaseAngle, pf);
  Serial.printf("Vrms: %.2f V, Irms: %.3f A | P: %.2f W, S: %.2f VA, Q: %.2f VAR\n",
                Vrms, Irms, P, S, Q);
  if (Vrms < 0.05) {
    digitalWrite(CAP_RELAY, LOW);
    digitalWrite(IND_RELAY, LOW);
    currentState = MONITORING;
    Serial.println("AC OFF detected: Resetting PF correction state");
  } else {
    updatePFCState(phaseAngle);
  }
  delay(2000);
}
