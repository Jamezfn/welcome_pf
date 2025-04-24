#include <Arduino.h>
#include <math.h>

// ADC pins and configuration
const int voltagePin = 35;
const int currentPin = 34;
const float voltageReference = 3.3;
const int ADCResolution = 4095;

// Global calibration variables
float zeroOffsetVoltage = 0.0;
float zeroOffsetCurrent = 0.0;

// Scaling and sensitivity factors
const float voltageScalingFactor = 0.723;
const float currentSensitivity   = 0.185; 

// AC parameters
const float acFrequency = 50.0;  
const unsigned long periodMicros = (unsigned long)(1000000 / acFrequency);

// Measurement settings
const int   NUM_CYCLES = 100;
const float alpha      = 0.1;
float filteredVoltage  = 0;
float filteredCurrent  = 0;

// Relay pin
const int capacitorRelayPin = 25;

// PF thresholds (for computing angles)
const float PF_ENGAGE_THRESHOLD    = 0.80; 
const float PF_DISENGAGE_THRESHOLD = 1.00; 

// Convert PF thresholds to angles (in degrees)
const float ANG_ENGAGE    = acos(PF_ENGAGE_THRESHOLD)   * 180.0 / PI;  
const float ANG_DISENGAGE = acos(PF_DISENGAGE_THRESHOLD) * 180.0 / PI;

bool capacitorBankEngaged = false;

// ------------------ State Machine for PF Correction ------------------
enum PFCState {
  MONITORING,
  CORRECTION_ACTIVE,
  CALIBRATING
};

PFCState currentState = MONITORING;

void updatePFCState(float phaseAngle) {
  switch (currentState) {
    case MONITORING:
      // phaseAngle > 0: inductive lag. Engage when lag > ANG_ENGAGE
      if (phaseAngle > ANG_ENGAGE) {
        digitalWrite(capacitorRelayPin, HIGH);
        capacitorBankEngaged = true;
        currentState = CORRECTION_ACTIVE;
        Serial.println("Capacitor Bank ENGAGED");
      }
      break;

    case CORRECTION_ACTIVE:
      if (phaseAngle < ANG_DISENGAGE) {
        digitalWrite(capacitorRelayPin, LOW);
        capacitorBankEngaged = false;
        currentState = MONITORING;
        Serial.println("Capacitor Bank DISENGAGED");
      }
      break;

    case CALIBRATING:
      // no-op
      break;
  }
}

// ------------------ Calibration Function ------------------
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

// ------------------ EMA Filtering Function ------------------
void updateEMA(float &filteredValue, int rawValue, float alpha) {
  filteredValue = alpha * rawValue + (1 - alpha) * filteredValue;
}

// ------------------ RMS Measurement Functions ------------------
float measureVoltageRMS(int sensorPin, float zeroOffset, int sampleCount) {
  double sumSq = 0;
  for (int i = 0; i < sampleCount; i++) {
    int reading = analogRead(sensorPin);
    float v = (reading - zeroOffset) * (voltageReference / ADCResolution);
    sumSq += v * v;
    delayMicroseconds(100);
  }
  float VrmsSensor = sqrt(sumSq / sampleCount);
  return VrmsSensor * voltageScalingFactor;
}

float measureCurrentRMS(int sensorPin, float zeroOffset, int sampleCount) {
  double sumSq = 0;
  for (int i = 0; i < sampleCount; i++) {
    int reading = analogRead(sensorPin);
    float v = (reading - zeroOffset) * (voltageReference / ADCResolution);
    sumSq += v * v;
    delayMicroseconds(100);
  }
  float VrmsSensor = sqrt(sumSq / sampleCount);
  return VrmsSensor / currentSensitivity;
}

void setup() {
  Serial.begin(115200);
  analogSetAttenuation(ADC_11db);

  pinMode(capacitorRelayPin, OUTPUT);
  digitalWrite(capacitorRelayPin, LOW);

  calibrateSensors();

  filteredVoltage = analogRead(voltagePin);
  filteredCurrent = analogRead(currentPin);

  Serial.println("Enhanced Synchronous Power Factor & Power Calculation Starting...");
}

void loop() {
  long sumDeltaT = 0;
  int cycles = 0;

  // zero-cross timing over NUM_CYCLES
  while (cycles < NUM_CYCLES) {
    unsigned long tV = 0, tI = 0;
    bool vCrossed = false, iCrossed = false;

    // track voltage zero-cross
    int prevV = analogRead(voltagePin);
    updateEMA(filteredVoltage, prevV, alpha);
    while (!vCrossed) {
      int rawV = analogRead(voltagePin);
      updateEMA(filteredVoltage, rawV, alpha);
      if (prevV < zeroOffsetVoltage && filteredVoltage >= zeroOffsetVoltage) {
        tV = micros();
        vCrossed = true;
      }
      prevV = filteredVoltage;
    }

    // track current zero-cross
    int prevI = analogRead(currentPin);
    updateEMA(filteredCurrent, prevI, alpha);
    while (!iCrossed) {
      int rawI = analogRead(currentPin);
      updateEMA(filteredCurrent, rawI, alpha);
      if (prevI < zeroOffsetCurrent && filteredCurrent >= zeroOffsetCurrent) {
        tI = micros();
        iCrossed = true;
      }
      prevI = filteredCurrent;
    }

    sumDeltaT += (long)(tI - tV);
    cycles++;
    delay(2);
  }

  // compute phase angle and PF
  float avgDt     = (float)sumDeltaT / NUM_CYCLES;
  float phaseAngle = (avgDt / periodMicros) * 360.0;
  while (phaseAngle > 180) phaseAngle -= 360;
  while (phaseAngle < -180) phaseAngle += 360;

  float Vrms = measureVoltageRMS(voltagePin, zeroOffsetVoltage, 1000);
  float Irms = measureCurrentRMS(currentPin, zeroOffsetCurrent, 1000);
  float pf    = cos(phaseAngle * PI / 180.0);
  float P     = Vrms * Irms * pf;
  float S     = Vrms * Irms;
  float Q     = sqrt(S*S - P*P);

  // print
  Serial.printf("Avg DeltaT: %.2f µs, Phase Angle: %.1f°, PF: %.3f\n", avgDt, phaseAngle, pf);
  Serial.printf("Vrms: %.2f V, Irms: %.3f A | P: %.2f W, S: %.2f VA, Q: %.2f VAR\n",
                Vrms, Irms, P, S, Q);

  // reset on no AC
  if (Vrms < 0.05) {
    digitalWrite(capacitorRelayPin, LOW);
    capacitorBankEngaged = false;
    currentState = MONITORING;
    Serial.println("AC OFF detected: Resetting PF correction state");
  } else {
    updatePFCState(phaseAngle);
  }

  delay(2000);
}