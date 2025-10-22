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
void pivot180(int pwmVal); // <<< NEW: Declaration for the 180-degree pivot function

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
const int   TURN_PWM       = 50;     // outer wheel PWM during arc
const float TURN_RADIUS_CM = 10.0f;  // robot center arc radius; if too small we pivot
const int   PIVOT_180_PWM  = 55;     // <<< NEW: PWM for the 180-degree turn

// ==================== Navigation Thresholds ====================
const int OBSTACLE_THRESHOLD      = 12; // start turn when front < 16 cm
const int OPEN_SPACE_THRESHOLD_CM = 50; // side "very open" heuristic
const int DEAD_END_THRESHOLD      = 12; // <<< NEW: Threshold for detecting a dead end on all sides

// ==================== Wall Following ====================
const int   BASE_PWM_STRAIGHT = 50;
const float Kp                = 1.5f; // P gain for wall following
const int   MAX_CORRECTION    = 20;
const int   WALL_DETECT_RANGE = 25;

// ==================== Alignment ====================
const int ALIGN_PWM         = 45;
const int ALIGN_DURATION_MS = 50;
const int ALIGN_TOLERANCE_CM= 1;

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

  // ---------- NEW STATE 1: Dead End / Trapped ----------
  // This must be checked first, as it's the highest priority.
  if (frontValid && dFront < DEAD_END_THRESHOLD &&
      leftValid  && dLeft  < DEAD_END_THRESHOLD &&
      rightValid && dRight < DEAD_END_THRESHOLD)
  {
      Serial.println("--- DEAD END: Executing 180 turn ---");
      stopMotors();
      delay(200);
      pivot180(PIVOT_180_PWM); // Call the new 180-degree turn function
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
  // ---------- STATE 3: Path clear (wall following) ----------
  else {
    if (leftValid && rightValid && dLeft < WALL_DETECT_RANGE && dRight < WALL_DETECT_RANGE) {
      int error = (int)(dLeft - dRight);
      int correction = (int)(Kp * error);
      correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

      int leftSpeed  = constrain(BASE_PWM_STRAIGHT - correction, 0, 100);
      int rightSpeed = constrain(BASE_PWM_STRAIGHT + correction, 0, 100);

      moveForward(rightSpeed, leftSpeed);
    } else {
      Serial.println("Straight");
      moveForward(BASE_PWM_STRAIGHT, BASE_PWM_STRAIGHT);
    }
  }

  delay(20);
}

// ===============================================================
//             NEW: 180 Degree Encoder-Based Pivot
// ===============================================================
void pivot180(int pwmVal) {
  // --- Calculate the required encoder counts for a 180-degree turn ---
  // The distance each wheel must travel is half the circumference of the circle
  // defined by the robot's wheelbase.
  // Distance = PI * (WHEEL_BASE_CM / 2)
  const float wheelCircumference = PI * WHEEL_DIAMETER_CM;
  const float turnDistance = PI * (WHEEL_BASE_CM / 2.0f);
  
  // Convert distance to encoder counts
  const long targetCounts = (long)((turnDistance / wheelCircumference) * countsPerRev);

  Serial.print("Target Counts for 180 pivot: "); Serial.println(targetCounts);

  // Reset encoders to 0
  encoderCountL = 0;
  encoderCountR = 0;

  // Start the pivot (e.g., to the right)
  pivotRight(pwmVal);

  // Loop until both wheels have traveled the required distance
  while(absl(encoderCountL) < targetCounts || absl(encoderCountR) < targetCounts) {
    // You could add sync logic here, but for a simple pivot it's often not needed.
    // Stop individual motors if they overshoot, to let the other catch up.
    if (absl(encoderCountL) >= targetCounts) {
      stopLeft();
    }
    if (absl(encoderCountR) >= targetCounts) {
      stopRight();
    }
    delay(5);
  }
  
  // Ensure motors are stopped
  stopMotors();
  Serial.println("180 turn complete.");
}


// ===============================================================
//                  TURNING (Arc with sync + stop-on-reach)
// ===============================================================
// Note: This function remains unchanged.
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

// Pin Mapping Reminder:
// R_RPWM = Right Forward, R_LPWM = Right Backward
// L_LPWM = Left Forward,  L_RPWM = Left Backward
void moveForward(int pwmValR, int pwmValL) {
  analogWrite(R_RPWM, pwmValR); analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, 0);       analogWrite(L_LPWM, pwmValL);
}

void moveBackward(int pwmVal) {
  analogWrite(R_RPWM, 0);       analogWrite(R_LPWM, pwmVal); // right backward
  analogWrite(L_RPWM, pwmVal);  analogWrite(L_LPWM, 0);      // left backward
}

void pivotLeft(int pwmVal) {
  // CCW: right forward, left backward
  analogWrite(R_RPWM, pwmVal); analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, pwmVal); analogWrite(L_LPWM, 0); // <<< CORRECTED: Was L_LPWM, now correctly L_RPWM for backward
}

void pivotRight(int pwmVal) {
  // CW: left forward, right backward
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

// Encoders
void countEncoderL() {
  int A = digitalRead(L_ENC_A);
  int B = digitalRead(L_ENC_B);
  if (A == B) { encoderCountL++; }
  else        { encoderCountL--; }
}

void countEncoderR() {
  int A = digitalRead(R_ENC_A);
  int B = digitalRead(R_ENC_B);
  // Assuming encoders are mounted mirrored, one will spin the opposite way.
  // This inverts the count so both count positive for forward motion.
  // If your robot pivots the wrong way, swap the ++ and -- here.
  if (A == B) { encoderCountR--; } 
  else        { encoderCountR++; }
}
