#include <Arduino.h>

// ====================== Motor Pins ======================
#define L_RPWM 4
#define L_LPWM 5
#define L_REN 6
#define L_LEN 7

#define R_RPWM 2
#define R_LPWM 3
#define R_REN 9
#define R_LEN 8

const int sensorPins[8] = {22, 23, 24, 25, 26, 27, 28, 29};
int sensorValues[8];

// ====================== PID Parameters ======================
float Kp = 40.0;     // ↑ More responsive turns
float Ki = 0.0;
float Kd = 4 ;      // ↑ Stronger correction on quick changes

int baseSpeed = 50;  // Base motor speed
int maxPWM = 100;    // Limit to prevent overshoot

// ====================== Variables ======================
float error = 0, lastError = 0;
float integral = 0, derivative = 0, correction = 0;

// ====================== Function Prototypes ======================
void readSensors();
float getLineError();
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void stopMotors();

void setup() {
  Serial.begin(9600);

  // --- Motor setup ---
  pinMode(R_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);
  pinMode(L_RPWM, OUTPUT);
  pinMode(L_LPWM, OUTPUT);

  pinMode(R_REN, OUTPUT);
  pinMode(R_LEN, OUTPUT);
  pinMode(L_REN, OUTPUT);
  pinMode(L_LEN, OUTPUT);

  digitalWrite(R_REN, HIGH);
  digitalWrite(R_LEN, HIGH);
  digitalWrite(L_REN, HIGH);
  digitalWrite(L_LEN, HIGH);

  // --- Sensor setup ---
  for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  Serial.println("PID Line Following Robot - Fast Turn Version Initialized");
}

void loop() {
  readSensors();
  error = getLineError();

  // PID control
  integral += error;
  derivative = error - lastError;
  correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  leftSpeed = constrain(leftSpeed, 0, maxPWM);
  rightSpeed = constrain(rightSpeed, 0, maxPWM);

  setMotorSpeeds(leftSpeed, rightSpeed);

  // Debugging output
  // Serial.print("Error: ");
  // Serial.print(error);
  // Serial.print(" | Correction: ");
  // Serial.print(correction);
  // Serial.print(" | Left Speed: ");
  // Serial.print(leftSpeed);
  // Serial.print(" | Right Speed: ");
  // Serial.println(rightSpeed);
  // delay(100);  
  
}

// ===============================================================
//                  Function Definitions
// ===============================================================

void readSensors() {
  for (int i = 0; i < 8; i++) {
    sensorValues[i] = digitalRead(sensorPins[i]);
  }
}

// ---------- Compute Line Error (Weighted for faster turn detection) ----------
float getLineError() {
  long weightedSum = 0;
  long total = 0;

  // Give outer sensors slightly higher weight (so turns are detected earlier)
  const float weight[8] = {-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5};

  for (int i = 0; i < 8; i++) {
    int value = (sensorValues[i] == HIGH) ? 1 : 0; // HIGH = black line
    weightedSum += value * (weight[i] * 1000);
    total += value;
  }

  if (total == 0) {
    // Line lost
    stopMotors();
    return lastError; // maintain previous error to recover
  }

  float position = (float)weightedSum / total;
  return position / 1000.0f; // normalized error
}

// ---------- Set Motor Speeds ----------
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  analogWrite(R_RPWM, rightSpeed);
  analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, leftSpeed);
}

// ---------- Stop Motors ----------
void stopMotors() {
  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, 0);
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, 0);
}