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
void moveForwardDistance(int distance_cm, int pwmVal);
void wallFollowingMode(); // <<< NEW FUNCTION

void readSensors();
float getLineError();               
void setMotorSpeeds(int leftSpeed, int rightSpeed);
bool anyOnLine();
bool centerOnLine();
uint8_t countBlk(const uint8_t idxs[], uint8_t n);
bool strongLeftFeature();
bool strongRightFeature();
void brake(uint16_t ms);
void pivotLeftUntilCenter();
void pivotRightUntilCenter();
void searchWhenLost();
void lineFollowingMode();

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
const float WHEEL_DIAMETER_CM = 6.5f;
const float WHEEL_BASE_CM     = 17.0f;
const int pulsesPerMotorRev = 11;
const int gearRatio         = 20;
const int countsPerRev      = pulsesPerMotorRev * gearRatio * 2;

// ==================== Turning Configuration ====================
const int   TURN_PWM       = 50;
const float TURN_RADIUS_CM = 10.0f;
const int   PIVOT_180_PWM  = 55;

// ==================== Navigation Thresholds ====================
const int OBSTACLE_THRESHOLD      = 12;
const int OPEN_SPACE_THRESHOLD_CM = 1000;
const int DEAD_END_THRESHOLD      = 12;
const int NO_WALL_THRESHOLD       = 25;
const int CORNER_CLEARANCE_CM     = 6;

// ==================== Wall Following ====================
const int   BASE_PWM_STRAIGHT = 60;
const float Kp_Wall                = 4.0f;
const int   MAX_CORRECTION    = 20;
const int   WALL_DETECT_RANGE = 20;

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


// ====================== Line Sensors ======================
const uint8_t sensorPins[8] = {22, 23, 24, 25, 26, 27, 28, 29};
// After readSensors(): sensorVal[i] == 1 means "black", 0 means "white"
uint8_t sensorVal[8];
uint8_t sensorBits = 0; // bit i = 1 if sensor i sees black

// Set this to false if your board reads LOW on black.
const bool BLACK_IS_HIGH = true;

// Convenience groupings (0 = far-left, 7 = far-right)
const uint8_t CENTER_IDX[2] = {3, 4};
const uint8_t LEFT_IDX[3]   = {0, 1, 2};
const uint8_t RIGHT_IDX[3]  = {5, 6, 7};

// How strong a “left/right feature” must be to treat as a corner
const uint8_t LEFT_STRONG_MIN  = 2;
const uint8_t RIGHT_STRONG_MIN = 2;

// Pivot behavior at corners
const int TURN_PWM_LINE = 140;               // strong pivot power
const uint16_t PIVOT_TIMEOUT_MS = 600;  // safety stop if something goes wrong
const uint16_t BRAKE_MS = 30;

// ====================== PID Parameters ======================
float Kp = 40.0;
float Ki = 0.0;
float Kd = 4.0;

int baseSpeed = 50;   // cruise speed
int maxPWM    = 100;  // clamps for smoothness

// ====================== Variables ======================
float error = 0, lastError = 0;
float integral = 0, derivative = 0, correction = 0;

// ==================== Mode State ====================
bool wallMode = true;

// ==================== Setup ====================
void setup() {
  Serial.begin(9600);

  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT,  INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT);
  pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT);
  pinMode(R_REN, OUTPUT);  pinMode(R_LEN, OUTPUT);
  pinMode(L_REN, OUTPUT);  pinMode(L_LEN, OUTPUT);
  digitalWrite(R_REN, HIGH); digitalWrite(R_LEN, HIGH);
  digitalWrite(L_REN, HIGH); digitalWrite(L_LEN, HIGH);

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

    if (wallMode) {
    wallFollowingMode();

    // Check if all sensors detect white (surface)
    int whiteCount = 0;
    for (int i = 0; i < 8; i++) {
      int val = digitalRead(sensorPins[i]);
      if (val == LOW) whiteCount++;  // LOW = white
    }

    if (whiteCount >= 7) {  // all white detected
      Serial.println("All white detected! Switching to LINE FOLLOWING MODE...");
      delay(1000);
      stopMotors();
      wallMode = false;  // Switch mode
    }
  } else {
    lineFollowingMode();
  }

}

// ===============================================================
//                 WALL FOLLOWING MODE FUNCTION
// ===============================================================
void wallFollowingMode() {
  long dFront = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long dLeft  = readUltrasonic(TRIG_LEFT,  ECHO_LEFT);
  long dRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  bool frontValid = dFront > 0;
  bool leftValid  = dLeft  > 0;
  bool rightValid = dRight > 0;

  Serial.print("F:"); Serial.print(dFront);
  Serial.print(" L:"); Serial.print(dLeft);
  Serial.print(" R:"); Serial.println(dRight);

  // ---------- STATE 1: Dead End ----------
  if (frontValid && dFront < DEAD_END_THRESHOLD &&
      leftValid  && dLeft  < DEAD_END_THRESHOLD &&
      rightValid && dRight < DEAD_END_THRESHOLD)
  {
      Serial.println("--- DEAD END: Executing 180 turn ---");
      stopMotors();
      delay(100);
      pivot180(PIVOT_180_PWM);
      stopMotors();
      delay(100);
  }
  // ---------- STATE 2: Obstacle Ahead ----------
  else if (frontValid && dFront < OBSTACLE_THRESHOLD) {
    stopMotors();
    delay(100);

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
      delay(100);
    } else if (isUTurn) {
      Serial.println("--- U-Turn: skipping align ---");
    }

    if (turnLeft) smoothTurnLeft();
    else          smoothTurnRight();

    stopMotors();
    delay(100);
  }
  // ---------- STATE 3: No Wall on Left ----------
  else if (leftValid && dLeft > NO_WALL_THRESHOLD) {
    Serial.println("--- No left wall. Clearing corner... ---");
    moveForwardDistance(CORNER_CLEARANCE_CM, BASE_PWM_STRAIGHT);
    
    Serial.println("--- Corner cleared. Initiating left turn. ---");
    stopMotors();
    delay(10);
    smoothTurnLeft();
    stopMotors();
    delay(150);
  }
  // ---------- STATE 4: Path Clear (Follow Wall) ----------
  else {
    int leftSpeed, rightSpeed;

    if (leftValid && rightValid && dLeft < WALL_DETECT_RANGE && dRight < WALL_DETECT_RANGE) {
      int error = (int)(dLeft - dRight);
      int correction = (int)(Kp * error);
      correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
      leftSpeed  = constrain(BASE_PWM_STRAIGHT - correction, 0, 100);
      rightSpeed = constrain(BASE_PWM_STRAIGHT + correction, 0, 100);
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
      int correction = (int)(Kp_Wall * error);
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

  delay(10);
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
  else        { encoderCountR++;}
}

// ===============================================================
//                  Function Definitions for Line Following
// ===============================================================

void lineFollowingMode() {
  readSensors();

  // 1) If we see a strong corner on one side and center is not on line, snap pivot
  if (strongLeftFeature() && !centerOnLine()) {
    brake(BRAKE_MS);
    pivotLeftUntilCenter();
    brake(BRAKE_MS);
  } 
  else if (strongRightFeature() && !centerOnLine()) {
    brake(BRAKE_MS);
    pivotRightUntilCenter();
    brake(BRAKE_MS);
  } 
  else {
    // 2) Normal PID tracking (with smooth recovery when line is momentarily lost)
    if (!anyOnLine()) {
      searchWhenLost();
      return;
    }

    error = getLineError();
    integral += error;
    derivative = error - lastError;
    correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
    lastError = error;

    int leftSpeed  = constrain((int)(baseSpeed + correction),  0, maxPWM);
    int rightSpeed = constrain((int)(baseSpeed - correction), 0, maxPWM);

    setMotorSpeeds(leftSpeed, rightSpeed);
  }

  delay(5);
}

// ====================== Sensor and Motor Functions ======================
void readSensors() {
  sensorBits = 0;
  for (int i = 0; i < 8; i++) {
    int raw = digitalRead(sensorPins[i]);
    bool isBlack = BLACK_IS_HIGH ? (raw == HIGH) : (raw == LOW);
    sensorVal[i] = isBlack ? 1 : 0;
    if (isBlack) sensorBits |= (1u << i);
  }
}

float getLineError() {
  static const float weight[8] = {-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5};
  float sumW = 0.0f, sum = 0.0f;

  for (int i = 0; i < 8; i++) {
    if (sensorVal[i]) { sumW += weight[i]; sum += 1.0f; }
  }

  if (sum == 0.0f) return lastError;
  return sumW / sum;
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  analogWrite(R_RPWM, rightSpeed);
  analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, leftSpeed);
}

void stopMotors() {
  stopLeft(); 
  stopRight();
}

// ====================== Corner / Recovery Helpers ======================
bool anyOnLine() { return sensorBits != 0; }
bool centerOnLine() { return sensorVal[CENTER_IDX[0]] || sensorVal[CENTER_IDX[1]]; }

uint8_t countBlk(const uint8_t idxs[], uint8_t n) {
  uint8_t c = 0;
  for (uint8_t k = 0; k < n; k++) c += sensorVal[idxs[k]] ? 1 : 0;
  return c;
}

bool strongLeftFeature() {
  uint8_t L = countBlk(LEFT_IDX, 3);
  uint8_t R = countBlk(RIGHT_IDX, 3);
  return (L >= LEFT_STRONG_MIN) && (R <= 1);
}

bool strongRightFeature() {
  uint8_t L = countBlk(LEFT_IDX, 3);
  uint8_t R = countBlk(RIGHT_IDX, 3);
  return (R >= RIGHT_STRONG_MIN) && (L <= 1);
}

void brake(uint16_t ms) { stopMotors(); delay(ms); }

void pivotLeftUntilCenter() {
  unsigned long t0 = millis();
  while (millis() - t0 < PIVOT_TIMEOUT_MS) {
    analogWrite(R_RPWM, TURN_PWM_LINE);
    analogWrite(R_LPWM, 0);
    analogWrite(L_RPWM, TURN_PWM_LINE);
    analogWrite(L_LPWM, 0);
    readSensors();
    if (centerOnLine()) break;
    delay(4);
  }
}

void pivotRightUntilCenter() {
  unsigned long t0 = millis();
  while (millis() - t0 < PIVOT_TIMEOUT_MS) {
    analogWrite(R_RPWM, 0);
    analogWrite(R_LPWM, TURN_PWM_LINE);
    analogWrite(L_RPWM, 0);
    analogWrite(L_LPWM, TURN_PWM_LINE);
    readSensors();
    if (centerOnLine()) break;
    delay(4);
  }
}



void pivotLeft90UntilLine(int pwmVal) {
  // Calculate the target encoder counts for a full 90-degree turn
  const float wheelCircumference = PI * WHEEL_DIAMETER_CM;
  const float turnDistance = (PI * WHEEL_BASE_CM) / 4.0f; // 1/4 of the full circle circumference
  const long targetCounts = (long)((turnDistance / wheelCircumference) * countsPerRev);

  Serial.println("Pivoting left (max 90 deg) until line is found...");

  // Reset encoders safely
  noInterrupts();
  encoderCountL = 0;
  encoderCountR = 0;
  interrupts();

  // Start pivoting left
  pivotLeft(pwmVal);

  bool lineWasFound = false;

  // Loop until EITHER the 90-degree turn is complete OR a line is detected
  while (absl(encoderCountL) < targetCounts || absl(encoderCountR) < targetCounts) {
    
    // Check the line sensors in every loop iteration
    readSensors();
    if (anyOnLine()) {
      lineWasFound = true;
      break; // Exit the loop immediately if a line is found
    }

    // If one wheel reaches the target count first, stop it to prevent overshooting
    if (absl(encoderCountL) >= targetCounts) { stopLeft(); }
    if (absl(encoderCountR) >= targetCounts) { stopRight(); }
    
    delay(5); // Small delay to allow motors to run and not overwhelm the CPU
  }

  stopMotors(); // Stop all movement

  if (lineWasFound) {
    Serial.println("Line found! Stopping pivot.");
  } else {
    Serial.println("90-degree pivot completed, line was not found.");
  }
}


/**
 * @brief Pivots the robot up to 90 degrees to the right, stopping early
 *        if a line is detected.
 * 
 * @param pwmVal The motor speed (0-255) to use for the pivot.
 */
void pivotRight90UntilLine(int pwmVal) {
  // Calculation is identical to the left pivot
  const float wheelCircumference = PI * WHEEL_DIAMETER_CM;
  const float turnDistance = (PI * WHEEL_BASE_CM) / 4.0f;
  const long targetCounts = (long)((turnDistance / wheelCircumference) * countsPerRev);

  Serial.println("Pivoting right (max 90 deg) until line is found...");

  // Reset encoders safely
  noInterrupts();
  encoderCountL = 0;
  encoderCountR = 0;
  interrupts();

  // Start pivoting right
  pivotRight(pwmVal);

  bool lineWasFound = false;

  // Loop until EITHER the 90-degree turn is complete OR a line is detected
  while (absl(encoderCountL) < targetCounts || absl(encoderCountR) < targetCounts) {
    
    // Check the line sensors in every loop iteration
    readSensors();
    if (anyOnLine()) {
      lineWasFound = true;
      break; // Exit the loop immediately if a line is found
    }

    // If one wheel reaches the target count first, stop it
    if (absl(encoderCountL) >= targetCounts) { stopLeft(); }
    if (absl(encoderCountR) >= targetCounts) { stopRight(); }
    
    delay(5);
  }

  stopMotors();

  if (lineWasFound) {
    Serial.println("Line found! Stopping pivot.");
  } else {
    Serial.println("90-degree pivot completed, line was not found.");
  }
}

void searchWhenLost() {
  pivotLeft90UntilLine(50);
  pivotRight90UntilLine(50);
  pivotRight90UntilLine(50);
}
