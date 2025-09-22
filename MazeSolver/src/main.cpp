#include <Arduino.h>
void rotateOneCycle(int pwmVal, bool clockwise);
void stopMotor();
long readUltrasonic(int trigPin, int echoPin);
void countEncoder();
// Ultrasonic Pins
#define TRIG_A 22
#define ECHO_A 23
#define TRIG_B 24
#define ECHO_B 25
// Motor Pins
#define RPWM 5
#define LPWM 6
#define R_EN 7
#define L_EN 8
// Encoder Pins
#define ENC_A 2
#define ENC_B 3
volatile long encoderCount = 0;
const int pulsesPerMotorRev = 11; // Encoder ticks per motor shaft revolution
const int gearRatio = 20; // Example gear ratio, adjust for your motor
const int countsPerRev = pulsesPerMotorRev * gearRatio; // Full output shaft revolution
void setup() {
 Serial.begin(9600);
 // Ultrasonic pins
 pinMode(TRIG_A, OUTPUT);
 pinMode(ECHO_A, INPUT);
 pinMode(TRIG_B, OUTPUT);
 pinMode(ECHO_B, INPUT);
 // Motor pins
 pinMode(RPWM, OUTPUT);
 pinMode(LPWM, OUTPUT);
 pinMode(R_EN, OUTPUT);
 pinMode(L_EN, OUTPUT);
 digitalWrite(R_EN, HIGH);
 digitalWrite(L_EN, HIGH);
 // Encoder interrupts
 pinMode(ENC_A, INPUT_PULLUP);
 attachInterrupt(digitalPinToInterrupt(ENC_A), countEncoder, RISING);
 Serial.println("Starting rotation control...");
}
void loop() {
 // Step 1: Measure distances
 long distA = readUltrasonic(TRIG_A, ECHO_A);
 long distB = readUltrasonic(TRIG_B, ECHO_B);
 Serial.print("Distance A: "); Serial.print(distA);
 Serial.print("| Distance B: "); Serial.println(distB);
 // Step 2: Compare & decide direction
 bool clockwise = false;
 if (distA > distB) {
 clockwise = true; // CW if A > B
 } else if (distB > distA) {
 clockwise = false; // CCW if B > A
 } else {
 stopMotor();
 delay(1000);
 return; // Distances are equal → stop
 }
 // Step 3: Set speed proportional to difference
 long difference = abs(distA - distB);
 int pwmVal = map(difference, 0, 100, 100, 255); // speed scaling from 100-255
 pwmVal = constrain(pwmVal, 100, 255);
 Serial.print("Difference: "); Serial.print(difference);
 Serial.print(" | PWM: "); Serial.println(pwmVal);
 // Step 4: Rotate for 1 cycle
 rotateOneCycle(pwmVal, clockwise);
 // Step 5: Pause
 stopMotor();
 delay(1000);
}
// Read ultrasonic distance in cm
long readUltrasonic(int trigPin, int echoPin) {
 digitalWrite(trigPin, LOW);
 delayMicroseconds(2);
 digitalWrite(trigPin, HIGH);
 delayMicroseconds(10);
 digitalWrite(trigPin, LOW);
 long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
 long distance = duration * 0.034 / 2; // cm
 return distance;
}
// Rotate motor for one output shaft revolution
void rotateOneCycle(int pwmVal, bool clockwise) {
 encoderCount = 0; // Reset encoder count
 if (clockwise) {
 analogWrite(RPWM, pwmVal);
 analogWrite(LPWM, 0);
 Serial.println("Rotating Clockwise...");
 } else {
 analogWrite(RPWM, 0);
 analogWrite(LPWM, pwmVal);
 Serial.println("Rotating Counter-Clockwise...");
 }
 while (encoderCount < countsPerRev) {
 // Wait for one full rotation
 }
}
// Stop motor
void stopMotor() {
 analogWrite(RPWM, 0);
 analogWrite(LPWM, 0);
 Serial.println("Motor Stopped.");
}
// Encoder ISR
void countEncoder() {
 encoderCount++;
}
