#include <Arduino.h>
#include <math.h>

// ADC pins and configuration
const int voltagePin = 35;
const int currentPin = 34;
const float voltageReference = 3.3;
const int ADCResolution = 4095;

// Global calibration variables (set by calibrateSensors())
float zeroOffsetVoltage = 0.0;
float zeroOffsetCurrent = 0.0;

// Scaling and sensitivity factors
const float voltageScalingFactor = 0.723;
const float currentSensitivity = 0.185; 

const float acFrequency = 50.0;  
const unsigned long periodMicros = (unsigned long)(1000000 / acFrequency);

const int NUM_CYCLES = 100;
const float alpha = 0.1;
float filteredVoltage = 0;
float filteredCurrent = 0;

const int capacitorRelayPin = 25;
const float PF_ENGAGE_THRESHOLD = 0.80;
const float PF_DISENGAGE_THRESHOLD = 1.0;
bool capacitorBankEngaged = false;

// ------------------ State Machine for PF Correction ------------------
enum PFCState {
  MONITORING,
  CORRECTION_ACTIVE,
  CALIBRATING
};

PFCState currentState = MONITORING;

void updatePFCState(float powerFactor) {
  float absPF = fabs(powerFactor);
  
  switch (currentState) {
    case MONITORING:
      if (absPF < PF_ENGAGE_THRESHOLD) {
        digitalWrite(capacitorRelayPin, HIGH);
        capacitorBankEngaged = true;
        currentState = CORRECTION_ACTIVE;
        Serial.println("Capacitor Bank ENGAGED");
      }
      break;
      
    case CORRECTION_ACTIVE:
      if (absPF > PF_DISENGAGE_THRESHOLD) {
        digitalWrite(capacitorRelayPin, LOW);
        capacitorBankEngaged = false;
        currentState = MONITORING;
        Serial.println("Capacitor Bank DISENGAGED");
      }
      break;
      
    case CALIBRATING:
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
  filteredValue = (alpha * rawValue) + ((1 - alpha) * filteredValue);
}

// ------------------ RMS Measurement Functions ------------------
float measureVoltageRMS(int sensorPin, float zeroOffset, int sampleCount) {
  double sumSquares = 0;
  for (int i = 0; i < sampleCount; i++) {
    int reading = analogRead(sensorPin);
    float sensorVoltage = (reading - zeroOffset) * (voltageReference / ADCResolution);
    sumSquares += sensorVoltage * sensorVoltage;
    delayMicroseconds(100);
  }
  float VrmsSensor = sqrt(sumSquares / sampleCount);
  return VrmsSensor * voltageScalingFactor;
}

float measureCurrentRMS(int sensorPin, float zeroOffset, int sampleCount) {
  double sumSquares = 0;
  for (int i = 0; i < sampleCount; i++) {
    int reading = analogRead(sensorPin);
    float sensorVoltage = (reading - zeroOffset) * (voltageReference / ADCResolution);
    sumSquares += sensorVoltage * sensorVoltage;
    delayMicroseconds(100);
  }
  float VrmsSensor = sqrt(sumSquares / sampleCount);
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
  int cyclesCounted = 0;

  while (cyclesCounted < NUM_CYCLES) {
    unsigned long voltageZeroTime = 0;
    unsigned long currentZeroTime = 0;
    bool voltageCrossed = false;
    bool currentCrossed = false;

    int prevVoltage = analogRead(voltagePin);
    updateEMA(filteredVoltage, prevVoltage, alpha);
    while (!voltageCrossed) {
      int rawVoltage = analogRead(voltagePin);
      updateEMA(filteredVoltage, rawVoltage, alpha);
      if (prevVoltage < zeroOffsetVoltage && filteredVoltage >= zeroOffsetVoltage) {
        voltageZeroTime = micros();
        voltageCrossed = true;
      }
      prevVoltage = filteredVoltage;
    }

    int prevCurrent = analogRead(currentPin);
    updateEMA(filteredCurrent, prevCurrent, alpha);
    while (!currentCrossed) {
      int rawCurrent = analogRead(currentPin);
      updateEMA(filteredCurrent, rawCurrent, alpha);
      if (prevCurrent < zeroOffsetCurrent && filteredCurrent >= zeroOffsetCurrent) {
        currentZeroTime = micros();
        currentCrossed = true;
      }
      prevCurrent = filteredCurrent;
    }
    
    long deltaT = (long)currentZeroTime - (long)voltageZeroTime;
    sumDeltaT += deltaT;
    cyclesCounted++;

    delay(2);
  }
  
  float avgDeltaT = (float) sumDeltaT / NUM_CYCLES;
  
  float phaseAngle = (avgDeltaT / periodMicros) * 360.0;
  while (phaseAngle > 180) phaseAngle -= 360;
  while (phaseAngle < -180) phaseAngle += 360;

  float phaseAngleRad = phaseAngle * PI / 180.0;
  float powerFactor = cos(phaseAngleRad);

  float Vrms = measureVoltageRMS(voltagePin, zeroOffsetVoltage, 1000);
  float Irms = measureCurrentRMS(currentPin, zeroOffsetCurrent, 1000);

  float realPower = Vrms * Irms * powerFactor; 
  float apparentPower = Vrms * Irms;  
  float reactivePower = sqrt(apparentPower * apparentPower - realPower * realPower);
  
  Serial.print("Avg DeltaT: ");
  Serial.print(avgDeltaT, 2);
  Serial.print(" µs, Phase Angle: ");
  Serial.print(phaseAngle, 1);
  Serial.print(" deg, PF: ");
  Serial.println(powerFactor, 3);
  
  Serial.print("Vrms: ");
  Serial.print(Vrms, 2);
  Serial.print(" V, Irms: ");
  Serial.print(Irms, 3);
  Serial.println(" A");
  
  Serial.print("Real Power (P): ");
  Serial.print(realPower, 2);
  Serial.print(" W, Apparent Power (S): ");
  Serial.print(apparentPower, 2);
  Serial.print(" VA, Reactive Power (Q): ");
  Serial.print(reactivePower, 2);
  Serial.println(" VAR");

  if (Vrms < 0.05) {
    digitalWrite(capacitorRelayPin, LOW);
    capacitorBankEngaged = false;
    currentState = MONITORING;
    Serial.println("AC OFF detected: Resetting PF correction state");
  } else {
    updatePFCState(powerFactor);
  }
  
  delay(2000);
}
