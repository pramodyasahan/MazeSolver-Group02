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

// Mode Switch Pin (Connect to GND to start Solve)
#define SEARCH_MODE 33 

// ===================================================================================
// 2. CONSTANTS & GLOBALS
// ===================================================================================
const float WHEEL_DIAMETER_CM = 6.5f;
const int pulsesPerMotorRev = 11;
const int gearRatio         = 20;
const int countsPerRev      = pulsesPerMotorRev * gearRatio * 2;

// --- SPEED SETTINGS ---
const int   TURN_PWM       = 55;
const int   PIVOT_180_PWM  = 60;
const int   BASE_PWM_STRAIGHT = 65;

// --- CRITICAL THRESHOLDS ---
const int OBSTACLE_THRESHOLD      = 8;  // Distance to front wall to stop
const int FRONT_WALL_TRIGGER      = 15; // Distance to front wall to consider it "blocked"
const int OPEN_SPACE_THRESHOLD_CM = 20; // If distance > 20, consider it an open path
const int INTERSECTION_DELAY      = 200;
const int WALL_DETECT_RANGE       = 20; // For PID centering

const float Kp_Solve              = 5.0f; 
const int   MAX_CORRECTION        = 30;   
const int   TARGET_WALL_DIST      = 7;    

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
void smoothTurnLeft();
void smoothTurnRight();
void moveForward(int pwmValR, int pwmValL);
void pivotLeft(int pwmVal);
void pivotRight(int pwmVal);
void pivot180(int pwmVal);
void moveForwardDistance(int distance_cm, int pwmVal);
void mazeMapping();
void mazeSolve();
void recordPath(char direction);
void simplifyPath();
void keepCentered(); 
void printPathToBluetooth();
void pivotTurn90(bool leftTurn, int pwmOuterMax);

// Inline Helpers
inline void stopRight(){ analogWrite(R_RPWM, 0); analogWrite(R_LPWM, 0); }
inline void stopLeft() { analogWrite(L_RPWM, 0); analogWrite(L_LPWM, 0); }
inline long absl(long x){ return (x < 0) ? -x : x; }

// ===================================================================================
// 4. SETUP (WITH DEBUG DIAGNOSTICS)
// ===================================================================================
// ===================================================================================
// 4. SETUP (With Stability Fix)
// ===================================================================================
void setup() {
  // 1. Initialize Serial Communication
  Serial.begin(9600);
  Serial3.begin(9600); // Bluetooth

  // 2. Sensor Pin Configuration
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT,  INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  // 3. Motor Pin Configuration
  pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT);
  pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT);
  pinMode(R_REN, OUTPUT);  pinMode(R_LEN, OUTPUT);
  pinMode(L_REN, OUTPUT);  pinMode(L_LEN, OUTPUT);
  
  // Enable Motor Drivers
  digitalWrite(R_REN, HIGH); digitalWrite(R_LEN, HIGH);
  digitalWrite(L_REN, HIGH); digitalWrite(L_LEN, HIGH);

  // 4. Encoder Pin Configuration
  pinMode(L_ENC_A, INPUT_PULLUP); pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP); pinMode(R_ENC_B, INPUT_PULLUP);
  
  // Attach Interrupts for Encoders
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), countEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), countEncoderR, CHANGE);

  // 5. Button Configuration
  pinMode(SEARCH_MODE, INPUT_PULLUP); 

  // ---------------------------------------------------------
  // THE FIX: SENSOR WARM-UP / BURN-IN
  // ---------------------------------------------------------
  stopMotors(); // Ensure motors are OFF
  Serial.println("=========================================");
  Serial.println("SYSTEM STARTUP. STABILIZING VOLTAGE...");
  
  // We loop 20 times (takes approx 1.5 seconds) to let electricity flow 
  // stable to the sensors and clear out the initial '999' glitches.
  for(int i=0; i<20; i++) {
     readUltrasonic(TRIG_LEFT, ECHO_LEFT);
     readUltrasonic(TRIG_FRONT, ECHO_FRONT);
     readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);
     delay(50); // Wait 50ms between reads
  }

  // Double check: Print the "Stable" values once before starting
  long l = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
  long f = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long r = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  Serial.print("SENSORS READY. Initial Views -> L:"); 
  Serial.print(l); Serial.print(" F:"); Serial.print(f); Serial.print(" R:"); Serial.println(r);
  Serial.println("=========================================");
  
  Serial3.println("BT: Robot Ready. Starting Mapping...");
  delay(500); // Short pause before action
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
      // Check Pin 33 (Ground it to start solving)
      if (digitalRead(SEARCH_MODE) == LOW) {
        Serial.println("Solving...");
        Serial3.println("BT: Button Pressed -> SOLVING MODE");
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
// 6. MAZE LOGIC (WITH SERIAL PRINTS)
// ===================================================================================

void mazeMapping() {
  // Check Finish Line (IR Array)
  int whiteCount = 0;
  for (int i = 0; i < 8; i++) if (digitalRead(sensorPins[i]) == LOW) whiteCount++;
  if (whiteCount >= 7) { 
    Serial.println("FINISH LINE DETECTED!");
    currentState = STATE_FINISHED; 
    return; 
  }

  // Read Sensors
  long dFront = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long dLeft  = readUltrasonic(TRIG_LEFT,  ECHO_LEFT);
  long dRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  // DEBUG PRINT
  Serial.print("L:"); Serial.print(dLeft);
  Serial.print(" F:"); Serial.print(dFront);
  Serial.print(" R:"); Serial.print(dRight);

  bool leftOpen = (dLeft > OPEN_SPACE_THRESHOLD_CM);
  bool frontOpen = (dFront > OBSTACLE_THRESHOLD);
  bool rightOpen = (dRight > OPEN_SPACE_THRESHOLD_CM);

  // 1. Dead End
  if (!leftOpen && !frontOpen && !rightOpen) {
    Serial.println(" -> DEAD END (Turning 180)"); 
    stopMotors(); delay(100); pivot180(PIVOT_180_PWM); recordPath('B'); return;
  }

  // 2. Left Turn Available (Priority 1)
  if (leftOpen) {
    Serial.println(" -> LEFT OPEN (Turning Left)"); 
    moveForwardDistance(5, BASE_PWM_STRAIGHT); 
    stopMotors(); delay(INTERSECTION_DELAY);
    smoothTurnLeft(); 
    
    // Logic to record path
    if (frontOpen || rightOpen) recordPath('L');
    else if (pathLength > 0 && path[pathLength-1] == 'B') recordPath('L'); 
    
    moveForwardDistance(5, BASE_PWM_STRAIGHT);
  }
  // 3. Straight (Priority 2)
  else if (frontOpen) {
    if (rightOpen) {
      Serial.println(" -> FRONT+RIGHT OPEN (Going Straight)");
      moveForwardDistance(5, BASE_PWM_STRAIGHT); 
      recordPath('S');
    } else {
      Serial.println(" -> WALL FOLLOW (Centering)");
      keepCentered(); 
    }
  }
  // 4. Right Turn (Priority 3)
  else if (rightOpen) {
    Serial.println(" -> RIGHT TURN");
    stopMotors(); delay(INTERSECTION_DELAY);
    smoothTurnRight();
    recordPath('R'); 
    moveForwardDistance(5, BASE_PWM_STRAIGHT);
  }
}

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

  bool leftOpen = (dLeft > OPEN_SPACE_THRESHOLD_CM);
  bool rightOpen = (dRight > OPEN_SPACE_THRESHOLD_CM);
  bool frontBlocked = (dFront > 0 && dFront < FRONT_WALL_TRIGGER);

  // Intersection or Front Wall detected
  if (leftOpen || rightOpen || frontBlocked) {
    stopMotors();
    delay(100);

    if (!frontBlocked) {
       moveForwardDistance(5, BASE_PWM_STRAIGHT); 
       stopMotors();
    }
    
    if (readIndex < pathLength) {
      char move = path[readIndex];
      readIndex++;
      
      Serial3.print("BT: Move -> "); Serial3.println(move);

      if (move == 'L') {
        smoothTurnLeft();
        moveForwardDistance(8, BASE_PWM_STRAIGHT); 
      } 
      else if (move == 'R') {
        smoothTurnRight();
        moveForwardDistance(8, BASE_PWM_STRAIGHT); 
      } 
      else if (move == 'S') {
        if (!frontBlocked) {
           moveForwardDistance(15, BASE_PWM_STRAIGHT);
        } else {
           keepCentered();
        }
      }
    } else {
      keepCentered(); // End of path
    }
  } 
  else {
    keepCentered(); // No intersection
  }
}

// ===================================================================================
// 7. HELPER FUNCTIONS
// ===================================================================================

void printPathToBluetooth() {
  Serial.println("Mapping Finished. Path:");
  Serial3.println("\n--- MAPPING DONE ---");
  Serial3.print("PATH: ");
  for(int i=0; i<pathLength; i++) {
    Serial.print(path[i]);
    Serial3.print(path[i]); 
    Serial3.print(" ");
  }
  Serial.println();
  Serial3.println("\n--------------------");
  Serial3.println("Connect Pin 33 to GND to start solving.");
}

void keepCentered() {
  long dFront = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long dLeft  = readUltrasonic(TRIG_LEFT,  ECHO_LEFT);
  long dRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  if (dFront > 0 && dFront < 5) {
    stopMotors();
    return;
  }

  int correction = 0;

  if (dLeft < WALL_DETECT_RANGE && dRight < WALL_DETECT_RANGE) {
    int error = dLeft - dRight; 
    correction = error * Kp_Solve;
  }
  else if (dLeft < WALL_DETECT_RANGE) {
    int error = dLeft - TARGET_WALL_DIST;
    correction = error * Kp_Solve;
  }
  else if (dRight < WALL_DETECT_RANGE) {
    int error = TARGET_WALL_DIST - dRight;
    correction = error * Kp_Solve;
  }

  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
  moveForward(BASE_PWM_STRAIGHT + correction, BASE_PWM_STRAIGHT - correction);
}

// --- PATH OPTIMIZATION ---
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

void stopMotors() { stopLeft(); stopRight(); }

void moveForwardDistance(int distance_cm, int pwmVal) {
  const float wheelCircumference = PI * WHEEL_DIAMETER_CM;
  const long targetCounts = (long)(((float)distance_cm / wheelCircumference) * countsPerRev);
  noInterrupts(); encoderCountL = 0; encoderCountR = 0; interrupts();
  moveForward(pwmVal, pwmVal);
  while (true) {
    noInterrupts(); long cL = encoderCountL; long cR = encoderCountR; interrupts();
    if ((absl(cL) + absl(cR)) / 2 >= targetCounts) break;
    delay(5);
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
    if (absl(encoderCountL) >= TICKS_90_DEG || absl(encoderCountR) >= TICKS_90_DEG) break;
    delay(5);
  }
  stopMotors();
}

void smoothTurnLeft()  { pivotTurn90(true, TURN_PWM); }
void smoothTurnRight() { pivotTurn90(false, TURN_PWM); }

void moveForward(int pwmValR, int pwmValL) {
  analogWrite(R_RPWM, pwmValR); analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, 0);       analogWrite(L_LPWM, pwmValL);
}

void pivotLeft(int pwmVal) {
  analogWrite(R_RPWM, pwmVal); analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, pwmVal); analogWrite(L_LPWM, 0);
}

void pivotRight(int pwmVal) {
  analogWrite(L_RPWM, 0);      analogWrite(L_LPWM, pwmVal);
  analogWrite(R_RPWM, 0);      analogWrite(R_LPWM, pwmVal);
}

// --------------------------------------------------------
// READ ULTRASONIC SENSOR
// --------------------------------------------------------
long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Timeout increased to 30ms to prevent false 0 readings
  long duration = pulseIn(echoPin, HIGH, 30000); 
  
  if (duration == 0) return 999; // Returns 999 if no echo (or wire broken)
  return duration * 0.034 / 2;
}

void countEncoderL() {
  if (digitalRead(L_ENC_A) == digitalRead(L_ENC_B)) encoderCountL++; else encoderCountL--;
}
void countEncoderR() {
  if (digitalRead(R_ENC_A) == digitalRead(R_ENC_B)) encoderCountR--; else encoderCountR++;
}