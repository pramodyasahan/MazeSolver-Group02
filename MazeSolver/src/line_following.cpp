#include <Arduino.h>

/* ========================= Pins ========================= */
#define L_RPWM 4
#define L_LPWM 5
#define L_REN 6
#define L_LEN 7

#define R_RPWM 2
#define R_LPWM 3
#define R_REN 9
#define R_LEN 8

// 8 digital reflectance sensors, 0 = far left ... 7 = far right
const uint8_t sensorPins[8] = {22, 23, 24, 25, 26, 27, 28, 29};

/* ============== (Optional) Encoders for crisp 90° ============== */
// If you have them, uncomment these 6 lines and wire as in your wall-follower.
// #define L_ENC_A 18
// #define L_ENC_B 19
// #define R_ENC_A 20
// #define R_ENC_B 21
// volatile long encoderCountL = 0, encoderCountR = 0;
// void countEncoderL(){ int A=digitalRead(L_ENC_A), B=digitalRead(L_ENC_B); if (A==B) encoderCountL++; else encoderCountL--; }
// void countEncoderR(){ int A=digitalRead(R_ENC_A), B=digitalRead(R_ENC_B); if (A==B) encoderCountR--; else encoderCountR++; }

/* ========================= PID ========================= */
// Tune on your robot. Units are relative to sensor weighting below.
float Kp = 40.0f;
float Ki = 0.0f;
float Kd = 4.0f;

int   baseSpeed = 60;     // forward PWM at zero error
int   maxPWM    = 100;    // clamp each wheel
int   maxCorr   = 120;    // clamp |correction| contribution
float emaAlpha  = 0.25f;  // smoothing on error (0..1). 0.25 = mild smoothing

// PID state
float err = 0, errEMA = 0, lastPos = 0;
float integ = 0;

/* ==================== Corner & Search ==================== */
enum LFState { FOLLOW, HARD_LEFT, HARD_RIGHT, SEARCH };
LFState state = FOLLOW;

int   turnPWM        = 120; // in-place pivot PWM for hard corners/search
int   reacquireCount = 3;   // require N consecutive "center seen" ticks to exit a turn
int   lineLostSpinMs = 20;  // spin step while searching
bool  lastWasLeft    = false; // for search direction

// If using encoders, you can do encoder-closed-loop 90°. Otherwise we pivot
// until center sensors reacquire line.
const bool USE_ENCODERS_FOR_90 = false; // set true if you enabled the encoder code above

// Geometry for optional encoder-based 90° (fill if you want perfect angles)
const float WHEEL_DIAMETER_CM = 6.5f;
const float WHEEL_BASE_CM     = 17.0f;
const int   countsPerRev      = 440; // your gear*channels*2
const float WHEEL_CIRC_CM     = PI * WHEEL_DIAMETER_CM;
const float TICKS_PER_DEG     = (((PI * WHEEL_BASE_CM) / WHEEL_CIRC_CM) * countsPerRev) / 360.0f;
const long  TICKS_90          = (long)(90.0f * TICKS_PER_DEG);

/* ===================== Utilities ===================== */
static inline int clamp(int v, int lo, int hi){ return (v<lo)?lo: (v>hi)?hi: v; }

void stopMotors(){
  analogWrite(R_RPWM, 0); analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, 0); analogWrite(L_LPWM, 0);
}

void setMotorSpeeds(int leftPWM, int rightPWM){
  leftPWM  = clamp(leftPWM,  0, 255);
  rightPWM = clamp(rightPWM, 0, 255);
  // Right forward = R_RPWM
  analogWrite(R_RPWM, rightPWM); analogWrite(R_LPWM, 0);
  // Left forward  = L_LPWM
  analogWrite(L_RPWM, 0);        analogWrite(L_LPWM, leftPWM);
}

/* Read sensors -> 8-bit mask, bit i = 1 if sensor i sees BLACK */
uint8_t readSensorsMask(){
  uint8_t mask = 0;
  for(int i=0;i<8;i++){
    int v = digitalRead(sensorPins[i]); // HIGH means black per your code
    if (v == HIGH) mask |= (1 << i);
  }
  return mask;
}

/* Weighted position in [-3.5, +3.5], like your code (center around 0) */
bool computePosition(float &posOut){
  static const float W[8] = {-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5};
  uint8_t m = readSensorsMask();
  float num = 0.0f, den = 0.0f;
  for(int i=0;i<8;i++){
    int on = (m >> i) & 1;
    if (on){ num += W[i]; den += 1.0f; }
  }
  if (den <= 0.0f) return false;
  posOut = num / den;
  return true;
}

/* Pattern detectors (tuned for digital array)
   - Center seen? => any of sensors 3 or 4 on.
   - Hard Left: left cluster on, center off
   - Hard Right: right cluster on, center off
   - T / Cross: many sensors on across the array (we default to straight) */
bool centerSeen(uint8_t m){ return (m & (1<<3)) || (m & (1<<4)); }
int  popcount8(uint8_t m){ int c=0; for(;m;m&=(m-1)) c++; return c; }

bool isHardLeft(uint8_t m){
  // leftmost sensors active, center off, right mostly off
  bool leftCluster  = ( (m & 0b00000111) != 0 );           // some of S0..S2
  bool centerOff    = !centerSeen(m);
  bool rightMostly0 = ( (m & 0b11100000) == 0 );          // S5..S7 off
  return leftCluster && centerOff && rightMostly0;
}

bool isHardRight(uint8_t m){
  bool rightCluster = ( (m & 0b11100000) != 0 );          // some of S5..S7
  bool centerOff    = !centerSeen(m);
  bool leftMostly0  = ( (m & 0b00000111) == 0 );          // S0..S2 off
  return rightCluster && centerOff && leftMostly0;
}

bool isJunction(uint8_t m){
  // many sensors black => likely cross/T. Keep going straight by PID (or choose policy).
  return popcount8(m) >= 5;
}

/* ================== Corner turning primitives ================== */
void pivotLeftTimed(){  // used if not using encoders
  analogWrite(R_RPWM, turnPWM); analogWrite(R_LPWM, 0);   // right fwd
  analogWrite(L_RPWM, turnPWM); analogWrite(L_LPWM, 0);   // left back
}
void pivotRightTimed(){
  analogWrite(L_RPWM, 0);        analogWrite(L_LPWM, turnPWM); // left fwd
  analogWrite(R_RPWM, 0);        analogWrite(R_LPWM, turnPWM); // right back
}

// If you enabled encoders above, you can do a precise 90° then small settle.
// Otherwise we pivot until center sensors reacquire the line.
void turnLeft90(){
  if (USE_ENCODERS_FOR_90){
    // noInterrupts(); encoderCountL=encoderCountR=0; interrupts();
    // while ( (abs(encoderCountL)+abs(encoderCountR))/2 < TICKS_90 ){
    //   pivotLeftTimed();
    //   delay(6);
    // }
    // stopMotors(); delay(60);
    // return;
  }
  // Timed/condition-based: spin until center seen (with debounce)
  int okCount = 0;
  uint32_t watchdog = millis() + 1500; // safety timeout
  while (millis() < watchdog){
    pivotLeftTimed();
    uint8_t m = readSensorsMask();
    if (centerSeen(m)) { if (++okCount >= reacquireCount) break; }
    else okCount = 0;
    delay(5);
  }
  stopMotors(); delay(60);
}

void turnRight90(){
  if (USE_ENCODERS_FOR_90){
    // noInterrupts(); encoderCountL=encoderCountR=0; interrupts();
    // while ( (abs(encoderCountL)+abs(encoderCountR))/2 < TICKS_90 ){
    //   pivotRightTimed();
    //   delay(6);
    // }
    // stopMotors(); delay(60);
    // return;
  }
  int okCount = 0;
  uint32_t watchdog = millis() + 1500;
  while (millis() < watchdog){
    pivotRightTimed();
    uint8_t m = readSensorsMask();
    if (centerSeen(m)) { if (++okCount >= reacquireCount) break; }
    else okCount = 0;
    delay(5);
  }
  stopMotors(); delay(60);
}

/* ========================= Setup ========================= */
void setup(){
  Serial.begin(115200);

  pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT);
  pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT);
  pinMode(R_REN, OUTPUT);  pinMode(R_LEN, OUTPUT);
  pinMode(L_REN, OUTPUT);  pinMode(L_LEN, OUTPUT);
  digitalWrite(R_REN, HIGH); digitalWrite(R_LEN, HIGH);
  digitalWrite(L_REN, HIGH); digitalWrite(L_LEN, HIGH);

  for (int i=0;i<8;i++) pinMode(sensorPins[i], INPUT);

  // If you enabled encoders:
  // pinMode(L_ENC_A, INPUT_PULLUP); pinMode(L_ENC_B, INPUT_PULLUP);
  // pinMode(R_ENC_A, INPUT_PULLUP); pinMode(R_ENC_B, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(L_ENC_A), countEncoderL, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(R_ENC_A), countEncoderR, CHANGE);

  stopMotors();
  Serial.println("Line follower: PID + hard-corner handling ready.");
}

/* ========================= Loop ========================= */
void loop(){
  uint8_t m = readSensorsMask();

  /* ----- Lost line? go SEARCH ----- */
  if (m == 0){
    state = SEARCH;
  } else {
    // Decide state if not searching
    if (isJunction(m)){
      // Simple policy: treat as straight; PID will keep you centered.
      state = FOLLOW;
    } else if (isHardLeft(m)){
      state = HARD_LEFT;
      lastWasLeft = true;
    } else if (isHardRight(m)){
      state = HARD_RIGHT;
      lastWasLeft = false;
    } else {
      state = FOLLOW;
    }
  }

  switch (state){
    case FOLLOW: {
      // Compute position and run PID
      float pos;
      if (!computePosition(pos)){
        // defensive: nothing seen, drop to search
        state = SEARCH;
        break;
      }

      // Derivative on measurement (smoother than on error for noisy sensors)
      float posEMA = errEMA*(1.0f-emaAlpha) + pos*emaAlpha;
      float dpos   = posEMA - lastPos;
      lastPos = posEMA;

      // PID
      err = -posEMA; // error = desired(0) - pos
      errEMA = errEMA*(1.0f-emaAlpha) + err*emaAlpha;

      // integral anti-windup
      integ += errEMA;
      integ  = clamp(integ, -1000, 1000);

      float u  = Kp*errEMA + Ki*integ - Kd*dpos;  // note the '-' with dpos (derivative on measurement)
      int   du = (int)clamp((int)u, -maxCorr, maxCorr);

      int left  = clamp(baseSpeed + du, 0, maxPWM);
      int right = clamp(baseSpeed - du, 0, maxPWM);

      setMotorSpeeds(left, right);
      break;
    }

    case HARD_LEFT: {
      // Commit to a left pivot until center sensors reacquire
      turnLeft90();
      state = FOLLOW;
      // small roll-in helps stabilize
      setMotorSpeeds(baseSpeed, baseSpeed);
      delay(60);
      break;
    }

    case HARD_RIGHT: {
      turnRight90();
      state = FOLLOW;
      setMotorSpeeds(baseSpeed, baseSpeed);
      delay(60);
      break;
    }

    case SEARCH: {
      // spin toward last known side of line
      if (lastWasLeft){
        pivotLeftTimed();
      } else {
        pivotRightTimed();
      }
      delay(lineLostSpinMs);
      // stop briefly and check again
      stopMotors();
      delay(5);
      // if center seen, immediately resume FOLLOW
      uint8_t mm = readSensorsMask();
      if (centerSeen(mm)) state = FOLLOW;
      break;
    }
  }

  // Optional: slow main loop a touch (digital sensors are fast)
  delay(5);
}
