#include <Arduino.h>
#include <math.h>

//–– ADC & timing consts
constexpr int VIN_PIN   = 35;
constexpr int IIN_PIN   = 34;
constexpr float VREF    = 3.3f;
constexpr int   ADC_MAX = 4095;
constexpr float ADC2V   = VREF / ADC_MAX;
constexpr float ANGLE_F = 360.0f / (1000000.0f / 50.0f);  // =360° per period

//–– Relays
constexpr int CAP_RELAY = 25;
constexpr int IND_RELAY = 26;

//–– Calibration
static float zeroV = 0, zeroI = 0;

//–– Scaling factors
constexpr float V_SCALE = 0.723f;
constexpr float I_SENS  = 0.185f;

//–– Filtering & median
struct Ms { int cycles; float alpha; };
static const Ms MS = { 250, 0.1f };
static float filtV = 0, filtI = 0;
constexpr int MED_SZ = 5;
static float angles[MED_SZ] = {0};
static int   aidx = 0;

//–– Thresholds (deg)
static const float CAP_E  = acos(0.80f) * 180/PI;
static const float CAP_D  = acos(0.95f) * 180/PI;
static const float IND_E  = -40.0f;
static const float IND_D  =   5.0f;

//–– State
enum State { MON, CAP, IND } state = MON;

//–– Simple in-place insertion sort + median
static float getMedian() {
  float buf[MED_SZ];
  memcpy(buf, angles, sizeof buf);
  for (int i=1; i<MED_SZ; i++){
    float k=buf[i]; int j=i-1;
    while(j>=0 && buf[j]>k){ buf[j+1]=buf[j]; j--; }
    buf[j+1]=k;
  }
  return buf[MED_SZ/2];
}

//–– Single-pass EMA
static inline void ema(float &f, float raw){
  f = MS.alpha * raw + (1 - MS.alpha) * f;
}

//–– Zero-cross time diff over many cycles
static float measurePhase() {
  uint32_t dtSum = 0;
  for(int c=0; c<MS.cycles; c++){
    // wait for V zero‐cross
    uint32_t t0;
    float prev = analogRead(VIN_PIN);
    ema(filtV, prev);
    while(true) {
      float v = analogRead(VIN_PIN);
      ema(filtV, v);
      if (prev < zeroV && filtV >= zeroV) { t0 = micros(); break; }
      prev = filtV;
      yield();
    }
    // wait for I zero‐cross
    uint32_t t1;
    prev = analogRead(IIN_PIN);
    ema(filtI, prev);
    while(true) {
      float i = analogRead(IIN_PIN);
      ema(filtI, i);
      if (prev < zeroI && filtI >= zeroI) { t1 = micros(); break; }
      prev = filtI;
      yield();
    }
    dtSum += (t1 - t0);
    yield();
  }
  float avgDt = float(dtSum) / MS.cycles;
  float angle = avgDt * ANGLE_F;
  // wrap to [-180,180]
  if (angle > 180) angle -= 360;
  else if (angle < -180) angle += 360;
  return angle;
}

//–– Batch RMS measurement
static float measureRMS(int pin, float offset, bool isV) {
  uint32_t sumSq = 0;
  for(int i=0; i<1000; i++){
    int r = analogRead(pin);
    float v = (r - offset) * ADC2V;
    uint32_t sq = uint32_t(v * v * 1e6);  // scale to avoid float ops inside loop
    sumSq += sq;
  }
  float rms = sqrt(sumSq / 1000.0f * 1e-6f);
  return isV ? rms * V_SCALE : rms / I_SENS;
}

//–– Helpers
static void updateRelays(float angle) {
  switch(state) {
    case MON:
      if (angle > CAP_E) { digitalWrite(CAP_RELAY, HIGH); digitalWrite(IND_RELAY, LOW); state=CAP; Serial.println("Cap ON"); }
      else if (angle < IND_E) { digitalWrite(IND_RELAY, HIGH); digitalWrite(CAP_RELAY, LOW); state=IND; Serial.println("Ind ON"); }
      break;
    case CAP:
      if (angle < CAP_D) { digitalWrite(CAP_RELAY, LOW); state=MON; Serial.println("Cap OFF"); }
      break;
    case IND:
      if (angle > IND_D) { digitalWrite(IND_RELAY, LOW); state=MON; Serial.println("Ind OFF"); }
      break;
  }
}

//–– Calibration
static void calibrate() {
  Serial.println("Calibrating...");
  uint32_t vSum=0, iSum=0;
  for(int i=0;i<1000;i++){
    vSum += analogRead(VIN_PIN);
    iSum += analogRead(IIN_PIN);
    delay(1);
  }
  zeroV = vSum / 1000.0f;
  zeroI = iSum / 1000.0f;
  Serial.printf("Zero offsets: V=%.2f I=%.2f\n", zeroV, zeroI);
}

void setup(){
  Serial.begin(115200);
  analogSetAttenuation(ADC_11db);
  pinMode(CAP_RELAY, OUTPUT);
  pinMode(IND_RELAY, OUTPUT);
  digitalWrite(CAP_RELAY, LOW);
  digitalWrite(IND_RELAY, LOW);
  calibrate();
  filtV = zeroV;
  filtI = zeroI;
}

void loop(){
  float angle = measurePhase();
  angles[aidx] = angle;
  aidx = (aidx + 1) % MED_SZ;
  float medA = getMedian();

  float Vr = measureRMS(VIN_PIN, zeroV, true);
  float Ir = measureRMS(IIN_PIN, zeroI, false);
  float pf = cosf(medA * PI/180.0f);
  float P  = Vr * Ir * pf;
  float S  = Vr * Ir;
  float Q  = sqrtf(S*S - P*P);

  Serial.printf("Ang: %.1f° Med: %.1f° PF: %.3f\n", angle, medA, pf);
  Serial.printf("Vr=%.2fV Ir=%.3fA P=%.2fW S=%.2fVA Q=%.2fVAR\n", Vr, Ir, P, S, Q);

  if (Vr < 0.1f) {
    digitalWrite(CAP_RELAY, LOW);
    digitalWrite(IND_RELAY, LOW);
    state = MON;
    Serial.println("AC OFF → reset");
  } else {
    updateRelays(medA);
  }

  delay(2000);
}
