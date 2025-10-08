#include <Arduino.h>

void rotateOneCycle(int pwmVal, bool clockwise);
void stopMotors();
long readUltrasonic(int trigPin, int echoPin);
void countEncoderL();
void countEncoderR();
void turnLeft90(int pwmVal, float wheelBase, float wheelDiameter);
void turnRight90(int pwmVal, float wheelBase, float wheelDiameter);
void moveForward(int pwmValR, int pwmValL);
void moveBackward(int pwmVal);
void readSensors();
int getLineError();
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void stopMotors();



// Motor Pins Left
#define L_RPWM 4
#define L_LPWM 5
#define L_REN 6
#define L_LEN 7

// Motor Pins Right
#define R_RPWM 2
#define R_LPWM 3
#define R_REN 9
#define R_LEN 8

// Encoder Pins
#define L_ENC_A 18
#define L_ENC_B 19
#define R_ENC_A 20
#define R_ENC_B 21

// Ultrasonic Pins
#define TRIG_FRONT 22
#define ECHO_FRONT 23
#define TRIG_LEFT 38
#define ECHO_LEFT 39
#define TRIG_RIGHT 26
#define ECHO_RIGHT 27

// Motor Control Parameters
const int pulsesPerMotorRev = 11;    // Encoder ticks per motor shaft revolution
const int gearRatio = 20;           // Example gear ratio, adjust for your motor
const int countsPerRev = pulsesPerMotorRev * gearRatio; // Full output shaft revolution

// Encoder Variables
volatile long encoderCountL = 0;
volatile long encoderCountR = 0;


// IR Sensor Array 
const int sensorPins[8] = {22, 23, 24, 25, 26, 27, 28, 29}; 
int sensorValues[8];


// ====================== PID Parameters ======================
float Kp = 10.0;     // Proportional gain
float Ki = 0.0;      // Integral gain
float Kd = 2.0;      // Derivative gain

int baseSpeed = 30;  // Base motor speed (PWM)
int maxPWM = 60;    // Maximum allowed PWM

// ====================== Variables for PID ======================
float error = 0, lastError = 0;
float integral = 0;
float derivative = 0;
float correction = 0;


// Mode Switch (optional)
//#define MODE_SWITCH 40  // HIGH = Line Following, LOW = Wall Following



void setup() {

  // Ultrasonic setup
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  // IR Sensor setup
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_CENTER, INPUT);
  pinMode(IR_RIGHT, INPUT);
  
  // Motor pins setup
  pinMode(R_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);
  pinMode(R_REN, OUTPUT);
  pinMode(R_LEN, OUTPUT);

  pinMode(L_RPWM, OUTPUT);
  pinMode(L_LPWM, OUTPUT);
  pinMode(L_REN, OUTPUT);
  pinMode(L_LEN, OUTPUT);

  digitalWrite(R_REN, HIGH);
  digitalWrite(R_LEN, HIGH);
  digitalWrite(L_REN, HIGH);
  digitalWrite(L_LEN, HIGH);

  // Mode switch
  //pinMode(MODE_SWITCH, INPUT_PULLUP);
  pinMode(L_ENC_A, INPUT_PULLUP);
  pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP);
  pinMode(R_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(L_ENC_A), countEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), countEncoderR, CHANGE);
  Serial.begin(9600);
  Serial.println("Starting rotation control...");

    Serial.begin(9600);


  // --- Sensor setup ---
  for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  Serial.println("PID Line Following Robot Initialized...");

  // --- Sensor setup ---
  // for (int i = 0; i < 8; i++) {
  //   pinMode(sensorPins[i], INPUT);
  // }

  // Serial.println("8-Channel IR Line Following Initialized...");
}



void loop() {
  //bool mode = digitalRead(MODE_SWITCH); // HIGH = Line, LOW = Wall
  //if (mode) {
   // lineFollowing();
  //} else {
   // wallFollowing();
  //}
  //Read ultrasonic distances
  long dFront = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long dLeft  = readUltrasonic(TRIG_LEFT,  ECHO_LEFT);
  long dRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  Serial.print("Front: "); Serial.print(dFront);
  delay(1000);
  Serial.print(" | Left: "); Serial.print(dLeft);
  delay(1000);
  Serial.print(" | Right: "); Serial.println(dRight);
  delay(1000);

  if (dFront < obstacleDist) {
    stopMotors();
    delay(1000);
    if (dLeft > dRight+10) {
      Serial.println("Obstacle ahead → Turning Left");
      turnLeft90(50, 17, 6.5); // Example wheelbase and diameter
      stopMotors();
      delay(1000);
    }
    else if (dRight > dLeft+10) {
      Serial.println("Obstacle ahead → Turning Right");
        turnRight90(50, 17, 6.5); // Example wheelbase and diameter
        stopMotors();
        delay(1000);
    }
    else {
      Serial.println("Dead end → Turning Around");
      //turnLeftCounts(speedPWM, 180);  // precise 180°
    }
  }
  else {
    Serial.println("Path clear → Moving Forward");
    if (dLeft < 4) {
      moveForward(30,60);
      delay(100);
    }
    else if (dRight < 4) {
      moveForward(60,30);
      delay(100);
    }
    else {
      moveForward(30,30);
    }
    
  //    // Step 1: Read all sensors
  // readSensors();

  // // Step 2: Compute line error
  // int error = getLineError();

  // // Step 3: Apply proportional control
  // int correction = Kp * error;

  // int leftSpeed = baseSpeed - correction;
  // int rightSpeed = baseSpeed + correction;

  // leftSpeed = constrain(leftSpeed, 0, maxPWM);
  // rightSpeed = constrain(rightSpeed, 0, maxPWM);

  // // Step 4: Drive motors
  // moveForward(leftSpeed, rightSpeed);

  // // Debug output
  // Serial.print("Error: "); Serial.print(error);
  // Serial.print(" | L_PWM: "); Serial.print(leftSpeed);
  // Serial.print(" | R_PWM: "); Serial.println(rightSpeed);

  // delay(10);
 
  }

  // Step 1: Read all sensors
  readSensors();

  // Step 2: Compute line error
  error = getLineError();

  // Step 3: PID Calculations
  integral += error;                // Accumulate error
  derivative = error - lastError;   // Change in error
  correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

  lastError = error;

  // Step 4: Compute motor speeds
  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  leftSpeed = constrain(leftSpeed, 0, maxPWM);
  rightSpeed = constrain(rightSpeed, 0, maxPWM);

  // Step 5: Apply speeds
  setMotorSpeeds(leftSpeed, rightSpeed);
}




// ---------- Read IR Sensor Values ----------
void readSensors() {
  for (int i = 0; i < 8; i++) {
    sensorValues[i] = digitalRead(sensorPins[i]);
  }
}

// ---------- Compute Line Error ----------
int getLineError() {
  long weightedSum = 0;
  long total = 0;

  for (int i = 0; i < 8; i++) {
    int value = (sensorValues[i] == LOW) ? 1 : 0;  // LOW = black line
    weightedSum += (long)value * i * 1000;
    total += value;
  }

  if (total == 0) {
    // Line lost → Stop motors and hold last known position
    stopMotors();
    Serial.println("Line lost! Stopping...");
    delay(200);
    return lastError; // Maintain previous error to recover
  }

  long position = weightedSum / total;
  int currentError = position - (3.5 * 1000);  // Center = 3.5 index
  return currentError / 1000;
}

// ---------- Set Motor Speeds ----------
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Right motor forward
  analogWrite(R_RPWM, rightSpeed);
  analogWrite(R_LPWM, 0);

  // Left motor forward
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, leftSpeed);
}

// ---------- Stop Motors ----------
void stopMotors() {
  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, 0);
 }