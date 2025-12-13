#include <Arduino.h>

// ===================================================================================
// 1. PIN DEFINITIONS
// ===================================================================================
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

#define SEARCH_MODE 33 

// ===================================================================================
// 2. CONSTANTS & TUNING (COPIED EXACTLY FROM YOUR PROVIDED CODE)
// ===================================================================================
const float WHEEL_DIAMETER_CM = 6.5f;
const int pulsesPerMotorRev   = 11;
const int gearRatio           = 20;
const int countsPerRev        = pulsesPerMotorRev * gearRatio * 2;

// --- Turning Configuration ---
const int   TURN_PWM       = 50;
const float TURN_RADIUS_CM = 11.0f;
const int   PIVOT_180_PWM  = 55;

// --- Navigation Thresholds ---
const int OBSTACLE_THRESHOLD      = 8;
const int OPEN_SPACE_THRESHOLD_CM = 40;
const int DEAD_END_THRESHOLD      = 10;
const int NO_WALL_THRESHOLD       = 25;
const int CORNER_CLEARANCE_CM     = 10; // Crucial for timing turns

// --- Wall Following ---
const int   BASE_PWM_STRAIGHT = 60;
const float Kp_Wall           = 3.5f;
const int   MAX_CORRECTION    = 20;
const int   WALL_DETECT_RANGE = 20;

// --- Alignment ---
const int ALIGN_PWM         = 45;
const int ALIGN_DURATION_MS = 50;
const int ALIGN_TOLERANCE_CM= 1;

// --- Globals ---
volatile long encoderCountL = 0;
volatile long encoderCountR = 0;

char path[100];       
int pathLength = 0;   
int readIndex = 0;    

enum RobotState {
  STATE_MAPPING,
  STATE_FINISHED,
  STATE_WAITING_FOR_SWITCH,
  STATE_SOLVING
};

RobotState currentState = STATE_MAPPING;
const uint8_t sensorPins[8] = {22, 23, 24, 25, 26, 27, 28, 29};

// ===================================================================================
// 3. FUNCTION PROTOTYPES
// ===================================================================================
void stopMotors(); 
long readUltrasonic(int trigPin, int echoPin);
void countEncoderL();
void countEncoderR();
void moveForward(int pwmValR, int pwmValL);
void moveForwardDistance(int distance_cm, int pwmVal);
void pivotTurn90(bool leftTurn, int pwmOuterMax);
void pivot180(int pwmVal);
void pivotLeft(int pwmVal);
void pivotRight(int pwmVal);
void smoothTurnLeft();
void smoothTurnRight();

// Logic
void mazeMapping();
void mazeSolve();
void PID_Logic(long dLeft, long dRight);
void recordPath(char direction);
void simplifyPath();
void printPathToBluetooth();

inline void stopRight(){ analogWrite(R_RPWM, 0); analogWrite(R_LPWM, 0); }
inline void stopLeft() { analogWrite(L_RPWM, 0); analogWrite(L_LPWM, 0); }
inline long absl(long x){ return (x < 0) ? -x : x; }

// ===================================================================================
// 4. SETUP
// ===================================================================================
void setup() {
  Serial.begin(9600);
  Serial3.begin(9600); 

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

  pinMode(SEARCH_MODE, INPUT_PULLUP); 

  stopMotors();
  Serial3.println("BT: Robot Ready.");
}

// ===================================================================================
// 5. MAIN LOOP
// ===================================================================================
void loop() {
  switch (currentState) {
    case STATE_MAPPING:
      mazeMapping(); 
      break;

    case STATE_FINISHED:
      stopMotors();
      printPathToBluetooth();
      currentState = STATE_WAITING_FOR_SWITCH;
      break;

    case STATE_WAITING_FOR_SWITCH:
      stopMotors();
      if (digitalRead(SEARCH_MODE) == LOW) {
        Serial3.println("BT: SOLVING MODE");
        delay(2000); 
        currentState = STATE_SOLVING;
      }
      break;

    case STATE_SOLVING:
      mazeSolve();
      break;
  }
}

// ===================================================================================
// 6. MAZE MAPPING (ADAPTED FROM YOUR "WALL FOLLOWING MODE")
// ===================================================================================
void mazeMapping() {
  // --- Finish Line Check ---
  int whiteCount = 0;
  for (int i = 0; i < 8; i++) if (digitalRead(sensorPins[i]) == LOW) whiteCount++;
  if (whiteCount >= 7) { 
    stopMotors(); delay(1000);
    moveForward(BASE_PWM_STRAIGHT, BASE_PWM_STRAIGHT); delay(1000); // Cross line
    currentState = STATE_FINISHED; 
    return; 
  }

  // --- Read Sensors (Using your Logic) ---
  long dFront = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long dLeft  = readUltrasonic(TRIG_LEFT,  ECHO_LEFT);
  long dRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  // Validate readings ( -1 check)
  bool frontValid = (dFront > 0);
  bool leftValid  = (dLeft  > 0);
  bool rightValid = (dRight > 0);

  // ============================================================
  // STATE 1: DEAD END
  // ============================================================
  if (frontValid && dFront < OBSTACLE_THRESHOLD &&
      leftValid  && dLeft  < DEAD_END_THRESHOLD &&
      rightValid && dRight < DEAD_END_THRESHOLD)
  {
      stopMotors(); delay(100);
      pivot180(PIVOT_180_PWM);
      recordPath('B'); 
      stopMotors(); delay(100);
      return;
  }

  // ============================================================
  // STATE 2: OBSTACLE AHEAD (Wall in Front)
  // ============================================================
  else if (frontValid && dFront < OBSTACLE_THRESHOLD) {
    stopMotors(); delay(100);

    // --- ALIGNMENT (Your Code) ---
    // If not a U-turn scenario, check alignment
    bool turnLeft = (dLeft > dRight);
    
    // Check if we need to align before turning
    if (leftValid && rightValid && (abs(dLeft - dRight) > ALIGN_TOLERANCE_CM)) {
      if (dLeft < dRight) { pivotLeft(ALIGN_PWM); }
      else                { pivotRight(ALIGN_PWM); }
      delay(ALIGN_DURATION_MS);
      stopMotors(); delay(100);
    }

    // --- TURN DECISION ---
    // We override standard wall following slightly to ensure recording
    bool realLeftOpen = (leftValid && dLeft > NO_WALL_THRESHOLD);
    
    // If Left is open, we turn Left (unless it's a Dead End U-turn which State 1 catches)
    // If Left is blocked, we turn Right.
    if (realLeftOpen) {
       smoothTurnLeft();
       recordPath('L');
    } else {
       smoothTurnRight();
       recordPath('R');
    }
    stopMotors(); delay(100);
    return;
  }

  // ============================================================
  // STATE 3: NO WALL ON LEFT (Left Turn Available)
  // ============================================================
  else if (leftValid && dLeft > NO_WALL_THRESHOLD) {
    // This is the CRITICAL part for timing turns
    moveForwardDistance(CORNER_CLEARANCE_CM, BASE_PWM_STRAIGHT);
    
    stopMotors(); delay(10);
    smoothTurnLeft();
    recordPath('L'); 
    stopMotors(); delay(150); // Your delay for stability
    return;
  }

  // ============================================================
  // STATE 4: PATH CLEAR (PID Follow + Straight Detection)
  // ============================================================
  else {
    // We are going Straight.
    // Check if we are passing a Right Opening. If so, record 'S'.
    if (rightValid && dRight > NO_WALL_THRESHOLD) {
        // Simple check: If we are not turning Right, and Right is open, we chose Straight.
        // We add a check to ensure we haven't just recorded this.
        if (dFront > OPEN_SPACE_THRESHOLD_CM) {
             // We are committing to straight
             recordPath('S');
             // Drive a bit to prevent re-triggering immediately
             moveForwardDistance(15, BASE_PWM_STRAIGHT); 
             return;
        }
    }

    PID_Logic(dLeft, dRight);
  }
}

// ===================================================================================
// 7. MAZE SOLVING
// ===================================================================================
void mazeSolve() {
  int whiteCount = 0;
  for (int i = 0; i < 8; i++) if (digitalRead(sensorPins[i]) == LOW) whiteCount++;
  if (whiteCount >= 7) { 
    stopMotors(); 
    Serial3.println("BT: FINISHED MAZE!");
    while(1); 
  }

  long dFront = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long dLeft  = readUltrasonic(TRIG_LEFT,  ECHO_LEFT);
  long dRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  bool leftOpen = (dLeft > NO_WALL_THRESHOLD);
  bool rightOpen = (dRight > NO_WALL_THRESHOLD);
  bool frontBlocked = (dFront > 0 && dFront < OBSTACLE_THRESHOLD);

  if (leftOpen || rightOpen || frontBlocked) {
    stopMotors(); delay(100);

    // Alignment Logic during solve (Crucial!)
    if (frontBlocked && dLeft > 0 && dRight > 0 && abs(dLeft - dRight) > ALIGN_TOLERANCE_CM) {
       if (dLeft < dRight) pivotLeft(ALIGN_PWM); else pivotRight(ALIGN_PWM);
       delay(ALIGN_DURATION_MS);
       stopMotors();
    }

    if (readIndex < pathLength) {
      char move = path[readIndex];
      readIndex++;
      Serial3.print("BT: Doing -> "); Serial3.println(move);

      if (move == 'L') {
        moveForwardDistance(CORNER_CLEARANCE_CM, BASE_PWM_STRAIGHT); 
        stopMotors(); delay(10);
        smoothTurnLeft();
        stopMotors(); delay(100);
      } 
      else if (move == 'R') {
        moveForwardDistance(CORNER_CLEARANCE_CM, BASE_PWM_STRAIGHT); 
        stopMotors(); delay(10);
        smoothTurnRight();
        stopMotors(); delay(100);
      } 
      else if (move == 'S') {
        moveForwardDistance(15, BASE_PWM_STRAIGHT);
      }
    } else {
       PID_Logic(dLeft, dRight);
    }
  } 
  else {
     PID_Logic(dLeft, dRight);
  }
}

// ===================================================================================
// 8. HELPERS (EXACT COPIES FROM YOUR CODE)
// ===================================================================================
void PID_Logic(long dLeft, long dRight) {
  int leftSpeed, rightSpeed;
  bool leftValid = (dLeft > 0);
  bool rightValid = (dRight > 0);

  if (leftValid && rightValid && dLeft < WALL_DETECT_RANGE && dRight < WALL_DETECT_RANGE) {
    int error = (int)(dLeft - dRight);
    int correction = (int)(Kp_Wall * error);
    correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
    leftSpeed  = constrain(BASE_PWM_STRAIGHT - correction, 0, 100);
    rightSpeed = constrain(BASE_PWM_STRAIGHT + correction, 0, 100);
  }
  else if (leftValid && dLeft < 10) {
    int targetDist = 6;
    int error = (int)(dLeft - targetDist);
    int correction = (int)(Kp_Wall * error);
    correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
    leftSpeed  = constrain(BASE_PWM_STRAIGHT + correction, 0, 100);
    rightSpeed = constrain(BASE_PWM_STRAIGHT - correction, 0, 100);
  }
  else if (rightValid && dRight < 10) {
    int targetDist = 6;
    int error = (int)(targetDist - dRight);
    int correction = (int)(Kp_Wall * error);
    correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
    leftSpeed  = constrain(BASE_PWM_STRAIGHT - correction, 0, 100);
    rightSpeed = constrain(BASE_PWM_STRAIGHT + correction, 0, 100);
  }
  else {
    leftSpeed = BASE_PWM_STRAIGHT;
    rightSpeed = BASE_PWM_STRAIGHT;
  }
  moveForward(rightSpeed, leftSpeed);
  delay(10);
}

void recordPath(char direction) {
  path[pathLength] = direction;
  pathLength++;
  simplifyPath();
}

void simplifyPath() {
  if (pathLength < 3 || path[pathLength-2] != 'B') return;
  char prev = path[pathLength-3];
  char curr = path[pathLength-1];
  char newMove = '?';

  if (prev == 'L' && curr == 'L') newMove = 'S';
  else if (prev == 'L' && curr == 'S') newMove = 'R';
  else if (prev == 'R' && curr == 'L') newMove = 'B';
  else if (prev == 'S' && curr == 'L') newMove = 'R';
  else if (prev == 'S' && curr == 'S') newMove = 'B';
  else if (prev == 'L' && curr == 'R') newMove = 'B';

  if (newMove != '?') {
    path[pathLength-3] = newMove;
    pathLength -= 2;
    Serial3.print("BT: Optimized to "); Serial3.println(newMove);
  }
}

void printPathToBluetooth() {
  Serial.println("Mapping Finished. Path:");
  Serial3.println("\n--- MAPPING DONE ---");
  Serial3.print("PATH: ");
  for(int i=0; i<pathLength; i++) {
    Serial3.print(path[i]); 
    Serial3.print(" ");
  }
  Serial3.println("\nConnect Pin 33 to GND to start solving.");
}

void moveForward(int pwmValR, int pwmValL) {
  analogWrite(R_RPWM, pwmValR); analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, 0);       analogWrite(L_LPWM, pwmValL);
}

void moveForwardDistance(int distance_cm, int pwmVal) {
  const float wheelCircumference = PI * WHEEL_DIAMETER_CM;
  const long targetCounts = (long)(((float)distance_cm / wheelCircumference) * countsPerRev);

  noInterrupts();
  encoderCountL = 0; encoderCountR = 0;
  interrupts();

  moveForward(pwmVal, pwmVal);

  while (true) {
    noInterrupts();
    long currentL = encoderCountL;
    long currentR = encoderCountR;
    interrupts();
    if ((absl(currentL) + absl(currentR)) / 2 >= targetCounts) break;
    delay(10); // KEEPING YOUR EXACT DELAY
  }
  stopMotors();
}

void pivot180(int pwmVal) {
  const int targetCounts = 550; 
  encoderCountL = 0; encoderCountR = 0;
  pivotRight(pwmVal);
  while(absl(encoderCountL) < targetCounts || absl(encoderCountR) < targetCounts) {
    if (absl(encoderCountL) >= targetCounts) stopLeft();
    if (absl(encoderCountR) >= targetCounts) stopRight();
    delay(5);
  }
  stopMotors();
}

void pivotTurn90(bool leftTurn, int pwmOuterMax) {
  const int TICKS_90_DEG = 560;   
  int pwm = constrain(pwmOuterMax, 40, 255);
  encoderCountL = 0; encoderCountR = 0;
  
  if (leftTurn) {
    analogWrite(L_RPWM, 0);  analogWrite(L_LPWM, 0);
    analogWrite(R_RPWM, pwm);    analogWrite(R_LPWM, 0);
  } else {
    analogWrite(L_RPWM, 0);    analogWrite(L_LPWM, pwm);
    analogWrite(R_RPWM, 0);  analogWrite(R_LPWM, 0);
  }
  
  while (true) {
    long aL = absl(encoderCountL);
    long aR = absl(encoderCountR);
    if (aL >= TICKS_90_DEG || aR >= TICKS_90_DEG) break;
    delay(5);
  }
  stopMotors();
}

void pivotLeft(int pwmVal) {
  analogWrite(R_RPWM, pwmVal); analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, pwmVal); analogWrite(L_LPWM, 0);
}

void pivotRight(int pwmVal) {
  analogWrite(L_RPWM, 0);      analogWrite(L_LPWM, pwmVal);
  analogWrite(R_RPWM, 0);      analogWrite(R_LPWM, pwmVal);
}

void smoothTurnLeft()  { pivotTurn90(true, TURN_PWM); }
void smoothTurnRight() { pivotTurn90(false, TURN_PWM); }

void stopMotors() { stopLeft(); stopRight(); }

long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 25000); 
  if (duration == 0) return -1; // Using your Logic (-1 for timeout)
  return duration * 0.034 / 2;
}

void countEncoderL() {
  if (digitalRead(L_ENC_A) == digitalRead(L_ENC_B)) encoderCountL++; else encoderCountL--;
}
void countEncoderR() {
  if (digitalRead(R_ENC_A) == digitalRead(R_ENC_B)) encoderCountR--; else encoderCountR++;
}