#include <Arduino.h>

// ==================== Declarations ====================
void stopMotors();
long readUltrasonic(int trigPin, int echoPin);
void countEncoderL();
void countEncoderR();
void smoothTurnLeft();
void smoothTurnRight();
void moveForward(int pwmValR, int pwmValL);
void moveBackward(int pwmVal);
void pivotLeft(int pwmVal);
void pivotRight(int pwmVal);
void pivot180(int pwmVal);
bool isStuck();
void moveForwardDistance(int distance_cm, int pwmVal); // <<< NEW: Declaration for moving a set distance

// ==================== Pin Definitions ====================
#define TRIG_FRONT 48
#define ECHO_FRONT 49
#define TRIG_RIGHT 50
#define ECHO_RIGHT 51
#define TRIG_LEFT  52
#define ECHO_LEFT  53

#define L_RPWM 4
#define L_LPWM 5
#define L_REN  6
#define L_LEN  7

#define R_RPWM 2
#define R_LPWM 3
#define R_REN  9
#define R_LEN  8

#define L_ENC_A 18
#define L_ENC_B 19
#define R_ENC_A 20
#define R_ENC_B 21

// ==================== Robot Physical Constants ====================
const float WHEEL_DIAMETER_CM = 6.5f;   // wheel diameter
const float WHEEL_BASE_CM     = 17.0f;  // distance between wheels (contact patch to patch)

// Encoders: interrupt on CHANGE (rising + falling) -> counts/rev doubles
const int pulsesPerMotorRev = 11;
const int gearRatio         = 20;
const int countsPerRev      = pulsesPerMotorRev * gearRatio * 2; // 11*20*2 = 440

// ==================== Turning Configuration ====================
const int   TURN_PWM       = 80;     // outer wheel PWM during arc
const float TURN_RADIUS_CM = 10.0f;  // robot center arc radius; if too small we pivot
const int   PIVOT_180_PWM  = 85;

// ==================== Navigation Thresholds ====================
const int OBSTACLE_THRESHOLD      = 12; // start turn when front < 16 cm
const int OPEN_SPACE_THRESHOLD_CM = 50; // side "very open" heuristic
const int DEAD_END_THRESHOLD      = 12;
const int NO_WALL_THRESHOLD       = 30; // Threshold to determine if there's no wall on the left
const int CORNER_CLEARANCE_CM     = 5; // <<< NEW: Distance to move forward before turning at an open corner

// ==================== Wall Following ====================
const int   BASE_PWM_STRAIGHT = 80;
const float Kp                = 1.5f; // P gain for wall following
const int   MAX_CORRECTION    = 30;
const int   WALL_DETECT_RANGE = 25;

// ==================== Alignment ====================
const int ALIGN_PWM         = 45;
const int ALIGN_DURATION_MS = 50;
const int ALIGN_TOLERANCE_CM= 1;

// =================================================================//
//                Stuck Detection & Recovery                       //
// =================================================================//
volatile long encoderCountL = 0;
volatile long encoderCountR = 0;

long lastEncoderCountL = 0;
long lastEncoderCountR = 0;
unsigned long lastStuckCheckTime = 0;
int stuckCounter = 0;

// Global variables to track commanded motor PWM
int currentPwmL = 0;
int currentPwmR = 0;

const int STUCK_CHECK_INTERVAL_MS  = 250;
const long ENCODER_STUCK_THRESHOLD = 8;
const int STUCK_CONFIRMATION_COUNT = 3;

// ================

// ==================== Encoders ====================
volatile long encoderCountL = 0;
volatile long encoderCountR = 0;

// Helpers
inline void stopRight(){ analogWrite(R_RPWM, 0); analogWrite(R_LPWM, 0); }
inline void stopLeft() { analogWrite(L_RPWM, 0); analogWrite(L_LPWM, 0); }
inline long absl(long x){ return (x < 0) ? -x : x; }

void setup() {
  Serial.begin(9600);

  // Ultrasonic
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT,  INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  // Motors
  pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT);
  pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT);
  pinMode(R_REN, OUTPUT);  pinMode(R_LEN, OUTPUT);
  pinMode(L_REN, OUTPUT);  pinMode(L_LEN, OUTPUT);
  digitalWrite(R_REN, HIGH); digitalWrite(R_LEN, HIGH);
  digitalWrite(L_REN, HIGH); digitalWrite(L_LEN, HIGH);

  // Encoders
  pinMode(L_ENC_A, INPUT_PULLUP); pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP); pinMode(R_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), countEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), countEncoderR, CHANGE);

  Serial.println("Robot Initialized...");
}

// ===============================================================
//                  MAIN LOOP
// ===============================================================
void loop() {
  long dFront = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long dLeft  = readUltrasonic(TRIG_LEFT,  ECHO_LEFT);
  long dRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  bool frontValid = dFront > 0;
  bool leftValid  = dLeft  > 0;
  bool rightValid = dRight > 0;

  Serial.print("F:"); Serial.print(dFront);
  Serial.print(" L:"); Serial.print(dLeft);
  Serial.print(" R:"); Serial.println(dRight);

  // ---------- STATE 1: Dead End / Trapped ----------
  if (frontValid && dFront < DEAD_END_THRESHOLD &&
      leftValid  && dLeft  < DEAD_END_THRESHOLD &&
      rightValid && dRight < DEAD_END_THRESHOLD)
  {
      Serial.println("--- DEAD END: Executing 180 turn ---");
      stopMotors();
      delay(200);
      pivot180(PIVOT_180_PWM);
      stopMotors();
      delay(200);
  }
  // ---------- STATE 2: Obstacle ahead ----------
  else if (frontValid && dFront < OBSTACLE_THRESHOLD) {
    stopMotors();
    delay(150);

    bool turnLeft = (dLeft > dRight);

    bool isUTurn = false;
    if (turnLeft && dLeft > OPEN_SPACE_THRESHOLD_CM)   isUTurn = true;
    if (!turnLeft && dRight > OPEN_SPACE_THRESHOLD_CM) isUTurn = true;

    if (!isUTurn && leftValid && rightValid && (abs(dLeft - dRight) > ALIGN_TOLERANCE_CM)) {
      Serial.println("--- Aligning ---");
      if (dLeft < dRight) { pivotLeft(ALIGN_PWM); }
      else                { pivotRight(ALIGN_PWM); }
      delay(ALIGN_DURATION_MS);
      stopMotors();
      delay(200);
    } else if (isUTurn) {
      Serial.println("--- U-Turn: skipping align ---");
    }

    if (turnLeft) smoothTurnLeft();
    else          smoothTurnRight();

    stopMotors();
    delay(150);
  }
  // ---------- STATE 3: No wall on the left, so turn left ----------
  else if (leftValid && dLeft > NO_WALL_THRESHOLD) {
    Serial.println("--- No left wall. Clearing corner... ---");
    // Move forward a set distance to ensure the robot's center is past the corner
    moveForwardDistance(CORNER_CLEARANCE_CM, BASE_PWM_STRAIGHT);
    
    Serial.println("--- Corner cleared. Initiating left turn. ---");
    stopMotors(); // Ensure motors are stopped before turning
    delay(10);
    smoothTurnLeft();
    stopMotors();
    delay(150);
  }
  // ---------- STATE 4: Path clear (wall following) ----------
  else {
    int leftSpeed, rightSpeed;

    if (leftValid && rightValid && dLeft < WALL_DETECT_RANGE && dRight < WALL_DETECT_RANGE) {
      int error = (int)(dLeft - dRight);
      int correction = (int)(Kp * error);
      correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
      leftSpeed  = constrain(BASE_PWM_STRAIGHT - correction, 0, 200);
      rightSpeed = constrain(BASE_PWM_STRAIGHT + correction, 0, 200);
      Serial.print("Dual Wall Follow | Error: "); Serial.println(error);
    }
    else if (leftValid && dLeft < 10) {
      int targetDist = 6;
      int error = (int)(dLeft - targetDist);
      int correction = (int)(Kp * error);
      correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
      leftSpeed  = constrain(BASE_PWM_STRAIGHT + correction, 0, 100);
      rightSpeed = constrain(BASE_PWM_STRAIGHT - correction, 0, 100);
      Serial.print("Left Wall Follow | Error: "); Serial.println(error);
    }
    else if (rightValid && dRight < 10) {
      int targetDist = 6;
      int error = (int)(targetDist - dRight);
      int correction = (int)(Kp * error);
      correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
      leftSpeed  = constrain(BASE_PWM_STRAIGHT - correction, 0, 100);
      rightSpeed = constrain(BASE_PWM_STRAIGHT + correction, 0, 100);
      Serial.print("Right Wall Follow | Error: "); Serial.println(error);
    }
    else {
      leftSpeed = BASE_PWM_STRAIGHT;
      rightSpeed = BASE_PWM_STRAIGHT;
      Serial.println("No wall detected -> Straight");
    }
    moveForward(rightSpeed, leftSpeed);
  }
  
  delay(20);
}

// ===============================================================
//             NEW: Encoder-Based Forward Movement
// ===============================================================
void moveForwardDistance(int distance_cm, int pwmVal) {
  // Calculate target encoder counts
  const float wheelCircumference = PI * WHEEL_DIAMETER_CM;
  const long targetCounts = (long)(((float)distance_cm / wheelCircumference) * countsPerRev);

  Serial.print("Moving forward ");
  Serial.print(distance_cm);
  Serial.print("cm (");
  Serial.print(targetCounts);
  Serial.println(" counts)");

  // Reset encoders safely
  noInterrupts();
  encoderCountL = 0;
  encoderCountR = 0;
  interrupts();

  moveForward(pwmVal, pwmVal);

  // Wait until the average distance of both wheels is met
  while (true) {
    noInterrupts();
    long currentL = encoderCountL;
    long currentR = encoderCountR;
    interrupts();
    
    if ((absl(currentL) + absl(currentR)) / 2 >= targetCounts) {
      break;
    }
    delay(10);
  }

  stopMotors();
  Serial.println("Target distance reached.");
}


// ===============================================================
//             180 Degree Encoder-Based Pivot
// ===============================================================
void pivot180(int pwmVal) {
  const float wheelCircumference = PI * WHEEL_DIAMETER_CM;
  const float turnDistance = PI * (WHEEL_BASE_CM / 2.0f);
  const long targetCounts = (long)((turnDistance / wheelCircumference) * countsPerRev);

  Serial.print("Target Counts for 180 pivot: "); Serial.println(targetCounts);

  encoderCountL = 0;
  encoderCountR = 0;

  pivotRight(pwmVal);

  while(absl(encoderCountL) < targetCounts || absl(encoderCountR) < targetCounts) {
    if (absl(encoderCountL) >= targetCounts) { stopLeft(); }
    if (absl(encoderCountR) >= targetCounts) { stopRight(); }
    delay(5);
  }
  
  stopMotors();
  Serial.println("180 turn complete.");
}


// ===============================================================
//                  TURNING (Arc with sync + stop-on-reach)
// ===============================================================
static void smoothTurnArc90(bool leftTurn, float Rc_cm, int pwmOuterMax) {
  const float B = WHEEL_BASE_CM;
  const float R_inner = Rc_cm - (B / 2.0f);
  const float R_outer = Rc_cm + (B / 2.0f);

  if (R_inner <= 0.5f) {
    Serial.println("Arc too tight -> pivot");
    if (leftTurn) pivotLeft(pwmOuterMax);
    else          pivotRight(pwmOuterMax);
    delay(220);
    stopRight(); stopLeft();
    return;
  }

  const float wheelC  = PI * WHEEL_DIAMETER_CM;
  const float s_inner = (PI * R_inner) / 2.0f;
  const float s_outer = (PI * R_outer) / 2.0f;

  const long cnt_inner = (long)((s_inner / wheelC) * countsPerRev);
  const long cnt_outer = (long)((s_outer / wheelC) * countsPerRev);

  int pwmOuter = pwmOuterMax;
  int pwmInner = (int)((float)pwmOuterMax * (s_inner / s_outer));
  const int pwmMin = 38;
  pwmInner = constrain(pwmInner, pwmMin, pwmOuter);

  encoderCountL = 0;
  encoderCountR = 0;

  if (leftTurn) {
    analogWrite(L_RPWM, 0);        analogWrite(L_LPWM, pwmInner);
    analogWrite(R_RPWM, pwmOuter); analogWrite(R_LPWM, 0);
  } else {
    analogWrite(R_RPWM, pwmInner); analogWrite(R_LPWM, 0);
    analogWrite(L_RPWM, 0);        analogWrite(L_LPWM, pwmOuter);
  }

  const int kSync = 2;

  while (true) {
    long aL = absl(encoderCountL);
    long aR = absl(encoderCountR);
    long tgtL = leftTurn ? cnt_inner : cnt_outer;
    long tgtR = leftTurn ? cnt_outer : cnt_inner;
    bool L_done = (aL >= tgtL);
    bool R_done = (aR >= tgtR);
    if (L_done && R_done) break;
    long remL = (aL >= tgtL) ? 0 : (tgtL - aL);
    long remR = (aR >= tgtR) ? 0 : (tgtR - aR);
    long remMax = (remL > remR) ? remL : remR;
    int baseOuter = pwmOuter;
    int baseInner = pwmInner;
    long tgtMax = (tgtL > tgtR) ? tgtL : tgtR;
    if (remMax < (tgtMax / 4)) {
      baseOuter = (baseOuter > pwmMin + 10) ? (baseOuter - 10) : pwmMin;
      baseInner = (baseInner > pwmMin + 10) ? (baseInner - 10) : pwmMin;
    }
    int errCounts = (int)(aR - aL);
    int adj = errCounts * kSync;
    if (adj > 10) adj = 10;
    if (adj < -10) adj = -10;
    if (!L_done) {
      int pwmL = leftTurn ? (baseInner + adj) : (baseOuter - adj);
      if (pwmL < pwmMin) pwmL = pwmMin;
      if (pwmL > 255)    pwmL = 255;
      analogWrite(L_RPWM, 0);
      analogWrite(L_LPWM, pwmL);
    } else { stopLeft(); }
    if (!R_done) {
      int pwmR = leftTurn ? (baseOuter - adj) : (baseInner + adj);
      if (pwmR < pwmMin) pwmR = pwmMin;
      if (pwmR > 255)    pwmR = 255;
      analogWrite(R_RPWM, pwmR);
      analogWrite(R_LPWM, 0);
    } else { stopRight(); }
    delay(5);
  }
  stopRight(); stopLeft();
}

void smoothTurnLeft()  { smoothTurnArc90(true,  TURN_RADIUS_CM, TURN_PWM); }
void smoothTurnRight() { smoothTurnArc90(false, TURN_RADIUS_CM, TURN_PWM); }

// ===============================================================
//                  HELPERS
// ===============================================================
void stopMotors() {
  stopRight(); stopLeft();
}

void moveForward(int pwmValR, int pwmValL) {
  analogWrite(R_RPWM, pwmValR); analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, 0);       analogWrite(L_LPWM, pwmValL);
}

void moveBackward(int pwmVal) {
  analogWrite(R_RPWM, 0);       analogWrite(R_LPWM, pwmVal);
  analogWrite(L_RPWM, pwmVal);  analogWrite(L_LPWM, 0);
}

void pivotLeft(int pwmVal) {
  analogWrite(R_RPWM, pwmVal); analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, pwmVal); analogWrite(L_LPWM, 0);
}

void pivotRight(int pwmVal) {
  analogWrite(L_RPWM, 0);      analogWrite(L_LPWM, pwmVal);
  analogWrite(R_RPWM, 0);      analogWrite(R_LPWM, pwmVal);
}

long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 25000);
  if (duration == 0) return -1;
  long distance = duration * 0.034 / 2;
  return distance;
}

void countEncoderL() {
  int A = digitalRead(L_ENC_A);
  int B = digitalRead(L_ENC_B);
  if (A == B) { encoderCountL++; }
  else        { encoderCountL--; }
}

void countEncoderR() {
  int A = digitalRead(R_ENC_A);
  int B = digitalRead(R_ENC_B);
  if (A == B) { encoderCountR--; } 
  else        { encoderCountR++; }
}
