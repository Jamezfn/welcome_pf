#include <Arduino.h>
#include <math.h>

const int voltagePin         = 35;
const int currentPin         = 34;
const float voltageReference = 3.3;
const int ADCResolution      = 4095;

enum RelayPins { CAP_RELAY = 25, IND_RELAY = 26 };

float zeroOffsetVoltage = 0.0;
float zeroOffsetCurrent = 0.0;

const float voltageScalingFactor = 0.723;
const float currentSensitivity   = 0.185;

const float acFrequency  = 50.0;
const unsigned long periodMicros = (unsigned long)(1000000 / acFrequency);

typedef struct { int cycles; float alpha; } MeasurementSettings;
MeasurementSettings meas = { 250, 0.1 };
float filteredVoltage = 0;
float filteredCurrent = 0;

const int ANGLE_MEDIAN_SIZE = 5;
float angleBuffer[ANGLE_MEDIAN_SIZE] = {0};
int   angleIndex = 0;

typedef struct { float engageAngle; float disengageAngle; } AngleThresholds;
AngleThresholds capThresh = { acos(0.80)*180.0/PI, acos(0.95)*180.0/PI };
AngleThresholds indThresh = { -40.0, 30.0 };

enum PFCState { MONITORING, CAP_CORRECTION, IND_CORRECTION, CALIBRATING };
PFCState currentState = MONITORING;

const float noLoadVoltage = 0.15;
const int REQUIRED_STABLE_READINGS = 5; 
int stableReadingCount = 0;

const unsigned long minSwitchInterval = 5000;
unsigned long lastSwitchTime = 0;


const unsigned long switchCooldownPeriod = 10000;
bool inCooldownPeriod = false;
unsigned long cooldownStartTime = 0;


float medianAngle(float *buf, int size) {
  float temp[ANGLE_MEDIAN_SIZE];
  memcpy(temp, buf, sizeof(temp));
  for(int i=1;i<size;i++){
    float key=temp[i];int j=i-1;
    while(j>=0&&temp[j]>key){temp[j+1]=temp[j];j--;}
    temp[j+1]=key;
  }
  return temp[size/2];
}

bool isAngleStable(float rawAngle,float medianAngle){
  float diff=fabs(rawAngle-medianAngle);
  if((rawAngle>-50&&rawAngle<-35)||(rawAngle>5&&rawAngle<15))
    return diff<20.0;
  return diff<45.0;
}

void engageCap(){
  digitalWrite(CAP_RELAY,HIGH);digitalWrite(IND_RELAY,LOW);
  lastSwitchTime=millis();cooldownStartTime=millis();inCooldownPeriod=true;
  currentState=CAP_CORRECTION;
  Serial.println("Capacitor Bank ENGAGED - entering cooldown period");
}

void disengageCap(){
  digitalWrite(CAP_RELAY,LOW);
  lastSwitchTime=millis();cooldownStartTime=millis();inCooldownPeriod=true;
  currentState=MONITORING;
  Serial.println("Capacitor Bank DISENGAGED - entering cooldown period");
}

void engageInd(){
  digitalWrite(IND_RELAY,HIGH);digitalWrite(CAP_RELAY,LOW);
  lastSwitchTime=millis();cooldownStartTime=millis();inCooldownPeriod=true;
  currentState=IND_CORRECTION;
  Serial.println("Inductor Bank ENGAGED - entering cooldown period");
}

void disengageInd(){
  digitalWrite(IND_RELAY,LOW);
  lastSwitchTime=millis();cooldownStartTime=millis();inCooldownPeriod=true;
  currentState=MONITORING;
  Serial.println("Inductor Bank DISENGAGED - entering cooldown period");
}

void updatePFCState(float angle){
  if(millis()-lastSwitchTime<minSwitchInterval)return;
  switch(currentState){
    case MONITORING:
      if(angle>capThresh.engageAngle)engageCap();
      else if(angle<indThresh.engageAngle)engageInd();
      break;
    case CAP_CORRECTION:
      if(angle<capThresh.disengageAngle)disengageCap();
      break;
    case IND_CORRECTION:
      if(angle>indThresh.disengageAngle)disengageInd();
      break;
    default:break;
  }
}

void calibrateSensors(){
  Serial.println("Starting calibration. Ensure NO LOAD is connected...");
  long vSum=0,iSum=0;const int samples=1000;
  for(int i=0;i<samples;i++){vSum+=analogRead(voltagePin);iSum+=analogRead(currentPin);delay(1);}  
  zeroOffsetVoltage=(float)vSum/samples;
  zeroOffsetCurrent=(float)iSum/samples;
  Serial.printf("Calibration complete. Voltage offset: %.2f, Current offset: %.2f\n",zeroOffsetVoltage,zeroOffsetCurrent);
}

void updateEMA(float &filt,int raw,float a){filt=a*raw+(1-a)*filt;}

float measureRMS(int pin,float offset,int count,bool isVoltage){
  double sumSq=0;
  for(int i=0;i<count;i++){
    int r=analogRead(pin);
    float v=(r-offset)*(voltageReference/ADCResolution);
    sumSq+=v*v;delayMicroseconds(100);
  }
  float rms=sqrt(sumSq/count);
  return isVoltage?rms*voltageScalingFactor:rms/currentSensitivity;
}

void setup(){
  Serial.begin(115200);
  analogSetAttenuation(ADC_11db);
  pinMode(CAP_RELAY,OUTPUT);pinMode(IND_RELAY,OUTPUT);
  digitalWrite(CAP_RELAY,LOW);digitalWrite(IND_RELAY,LOW);
  calibrateSensors();
  filteredVoltage=analogRead(voltagePin);
  filteredCurrent=analogRead(currentPin);
  Serial.println("Enhanced Synchronous PF Correction Starting");
}

void loop(){
  if(inCooldownPeriod){
    if(millis()-cooldownStartTime<switchCooldownPeriod){
      Serial.printf("In cooldown period: %lu/%lu ms\n",millis()-cooldownStartTime,switchCooldownPeriod);
      delay(1000);return;
    }else{inCooldownPeriod=false;Serial.println("Cooldown period ended");}
  }

  long sumDt=0;
  for(int c=0;c<meas.cycles;c++){
    unsigned long tV=0,tI=0;bool vCross=false,iCross=false;
    int pV=analogRead(voltagePin);updateEMA(filteredVoltage,pV,meas.alpha);
    while(!vCross){int rV=analogRead(voltagePin);updateEMA(filteredVoltage,rV,meas.alpha);
      if(pV<zeroOffsetVoltage && filteredVoltage>=zeroOffsetVoltage){tV=micros();vCross=true;}pV=filteredVoltage;}
    int pI=analogRead(currentPin);updateEMA(filteredCurrent,pI,meas.alpha);
    while(!iCross){int rI=analogRead(currentPin);updateEMA(filteredCurrent,rI,meas.alpha);
      if(pI<zeroOffsetCurrent && filteredCurrent>=zeroOffsetCurrent){tI=micros();iCross=true;}pI=filteredCurrent;}
    sumDt+=(long)(tI-tV);delay(2);
  }
  float avgDt=(float)sumDt/meas.cycles;
  float angle=(avgDt/periodMicros)*360.0;
  while(angle>180)angle-=360;while(angle<-180)angle+=360;
  angleBuffer[angleIndex]=angle;angleIndex=(angleIndex+1)%ANGLE_MEDIAN_SIZE;
  float medAngle=medianAngle(angleBuffer,ANGLE_MEDIAN_SIZE);


  float Vrms=measureRMS(voltagePin,zeroOffsetVoltage,1000,true);
  float Irms=measureRMS(currentPin,zeroOffsetCurrent,1000,false);
  float pf=cos(medAngle*PI/180.0);
  float P=Vrms*Irms*pf;
  float S=Vrms*Irms;
  float Q=sqrt(S*S-P*P);

  Serial.printf("Raw Angle: %.1f°, Median Angle: %.1f°, PF: %.3f\n",angle,medAngle,pf);
  Serial.printf("Vrms: %.2f V, Irms: %.3f A | P: %.2f W, S: %.2f VA, Q: %.2f VAR\n",Vrms,Irms,P,S,Q);


  if(Vrms<noLoadVoltage){
    digitalWrite(CAP_RELAY,LOW);digitalWrite(IND_RELAY,LOW);
    currentState=MONITORING;lastSwitchTime=millis();
    Serial.println("Low voltage detected: resetting all banks");
    delay(2000);return;
  }


  if(!isAngleStable(angle,medAngle)){
    Serial.println("Angle instability detected - skipping PFC update");
    stableReadingCount=0;delay(500);return;
  }
  stableReadingCount++;
  Serial.printf("Stable reading count: %d/%d\n",stableReadingCount,REQUIRED_STABLE_READINGS);
  if(stableReadingCount>=REQUIRED_STABLE_READINGS) updatePFCState(medAngle);
  else Serial.println("Waiting for more stable readings before PFC update");

  delay(20);
}
