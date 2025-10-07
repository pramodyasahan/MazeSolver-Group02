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


// IR Line Sensor Pins
#define IR_LEFT A0
#define IR_CENTER A1
#define IR_RIGHT A2

// Mode Switch (optional)
//#define MODE_SWITCH 40  // HIGH = Line Following, LOW = Wall Following



void setMotor(int leftSpeed, int rightSpeed) {
  analogWrite(ENA, abs(leftSpeed));
  analogWrite(ENB, abs(rightSpeed));

  digitalWrite(IN1, leftSpeed > 0);
  digitalWrite(IN2, leftSpeed < 0);
  digitalWrite(IN3, rightSpeed > 0);
  digitalWrite(IN4, rightSpeed < 0);
}

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
    
    
  }
}

void lineFollowing() {
  int left = analogRead(IR_LEFT);
  int center = analogRead(IR_CENTER);
  int right = analogRead(IR_RIGHT);

  // Thresholds may need tuning
  bool onLeft = left < 500;
  bool onCenter = center < 500;
  bool onRight = right < 500;

  if (onCenter && !onLeft && !onRight) {
    setMotor(150, 150); // Forward
  } else if (onLeft) {
    setMotor(100, 150); // Turn right
  } else if (onRight) {
    setMotor(150, 100); // Turn left
  } else {
    setMotor(0, 0); // Stop or search
  }
}

void wallFollowing() {
  long front = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long left = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
  long right = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  if (front < 15) {
    setMotor(-100, -100); // Back up
    delay(300);
    setMotor(100, -100); // Turn
    delay(300);
  } else if (left < 10) {
    setMotor(150, 100); // Turn right slightly
  } else if (right < 10) {
    setMotor(100, 150); // Turn left slightly
  } else {
    setMotor(150, 150); // Move forward
  }
}

// ==================== Ultrasonic ====================
long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  long distance = duration * 0.034 / 2;
  return distance;
}

// Stop motor
void stopMotors() {
  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, 0);
  Serial.println("Motors stopped.");
}

//---------------------------------------------------------------TURNING FUNCTIONS---------------------------------------------------------------------

// ---- Turn Left 90 Degrees ----
void turnLeft90(int pwmVal, float wheelBase, float wheelDiameter) {
  encoderCountL = 0;
  encoderCountR = 0;

  float wheelCircumference = PI * wheelDiameter;
  float arcLength = (PI * wheelBase) / 4.0;   // distance each wheel must travel
  float rotations = arcLength / wheelCircumference*2;
  long counts90 = (long)(rotations * countsPerRev);

  Serial.print("Target counts for 90 deg turn: ");
  Serial.println(counts90);

  // Run wheels opposite directions
  analogWrite(R_RPWM, pwmVal);
  analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, pwmVal);
  analogWrite(L_LPWM, 0);

  // Wait until both wheels have moved enough
  while (abs(encoderCountR) < counts90 && abs(encoderCountL) < counts90) {
    Serial.print("L: "); Serial.print(encoderCountL);
    Serial.print(" | R: "); Serial.println(encoderCountR);
  }

  stopMotors();
  Serial.println("Turn complete.");
}


// ---- Turn Right 90 Degrees ----

void turnRight90(int pwmVal, float wheelBase, float wheelDiameter) {
  encoderCountL = 0;
  encoderCountR = 0;

  float wheelCircumference = PI * wheelDiameter;
  float arcLength = (PI * wheelBase) / 4.0;   // distance each wheel must travel
  float rotations = arcLength / wheelCircumference*2;
  long counts90 = (long)(rotations * countsPerRev);

  Serial.print("Target counts for 90 deg turn: ");
  Serial.println(counts90);

  // Run wheels opposite directions
  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, pwmVal);
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, pwmVal);

  // Wait until both wheels have moved enough
  while (abs(encoderCountR) < counts90 && abs(encoderCountL) < counts90) {
    Serial.print("L: "); Serial.print(encoderCountL);
    Serial.print(" | R: "); Serial.println(encoderCountR);
  }

  stopMotors();
  Serial.println("Turn complete.");
}

//Rotate motor for one output shaft revolution
void rotateOneCycle(int pwmVal, bool clockwise) {
  encoderCount = 0; // Reset encoder count
  if (clockwise) {
    analogWrite(R_RPWM, pwmVal);
    analogWrite(R_LPWM, 0);
    Serial.println("Rotating Clockwise...");
  } else {
    analogWrite(R_RPWM, 0);
    analogWrite(R_LPWM, pwmVal);
    Serial.println("Rotating Counter-Clockwise...");
  }

  while (encoderCount < countsPerRev) {
    Serial.print("Count L : "); Serial.print(encoderCount);
    // Wait for one full rotation
  }
  void countEncoderL() {
  int A = digitalRead(L_ENC_A);
  int B = digitalRead(L_ENC_B);

  if (A == B) {
    encoderCountL++;
  } else {
    encoderCountL--;
  }
}

// ---- ISR for Right Encoder ----
void countEncoderR() {
  int A = digitalRead(R_ENC_A);
  int B = digitalRead(R_ENC_B);

  if (A == B) {
    encoderCountR++;
  } else {
    encoderCountR--;
  }


}
void moveBackward(int pwmVal) {
  analogWrite(R_RPWM, 0); 
  analogWrite(R_LPWM, pwmVal);
  analogWrite(L_RPWM, pwmVal); 
  analogWrite(L_LPWM, 0);
}

void moveForward(int pwmValR, int pwmValL) {

  analogWrite(R_RPWM, pwmValR); 
  analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, 0); 
  analogWrite(L_LPWM, pwmValL);
}



}

