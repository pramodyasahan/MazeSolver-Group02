#include <Arduino.h>

// Motor Pins
#define ENA 5
#define IN1 6
#define IN2 7
#define ENB 9
#define IN3 10
#define IN4 11

// Ultrasonic Pins
#define TRIG_FRONT 22
#define ECHO_FRONT 23
#define TRIG_LEFT 24
#define ECHO_LEFT 25
#define TRIG_RIGHT 26
#define ECHO_RIGHT 27

// IR Line Sensor Pins
#define IR_LEFT A0
#define IR_CENTER A1
#define IR_RIGHT A2

// Mode Switch (optional)
#define MODE_SWITCH 2  // HIGH = Line Following, LOW = Wall Following


long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); 
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH) / 58; // cm
}

void setMotor(int leftSpeed, int rightSpeed) {
  analogWrite(ENA, abs(leftSpeed));
  analogWrite(ENB, abs(rightSpeed));

  digitalWrite(IN1, leftSpeed > 0);
  digitalWrite(IN2, leftSpeed < 0);
  digitalWrite(IN3, rightSpeed > 0);
  digitalWrite(IN4, rightSpeed < 0);
}

void setup() {
  // Motor pins
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // Ultrasonic pins
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT); pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  // IR sensors
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_CENTER, INPUT);
  pinMode(IR_RIGHT, INPUT);

  // Mode switch
  pinMode(MODE_SWITCH, INPUT_PULLUP);

  Serial.begin(9600);
}

void loop() {
  bool mode = digitalRead(MODE_SWITCH); // HIGH = Line, LOW = Wall
  if (mode) {
    lineFollowing();
  } else {
    wallFollowing();
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


// // Define pins
// const int trigPin = 2;
// const int echoPin = 3;

// // Variables for duration and distance
// long duration;
// float distance;

// void setup() {
//   // Set pin mode
//   pinMode(trigPin, OUTPUT);
//   pinMode(echoPin, INPUT);

//   // Start Serial communication
//   Serial.begin(9600);
//   while (!Serial) { /* wait for native USB boards (e.g., Leonardo, Micro) */ }

//   Serial.println("Ultrasonic distance meter ready.");
// }

// void loop() {
//    // Calculate distance in cm (speed of sound = 343 m/s)
//   distance = duration * 0.0343 / 2;

//   // Print the distance on Serial Monitor
//   Serial.print("Distance: ");
//   Serial.print(distance);
//   Serial.println(" cm");

//   delay(500); // Delay for readability

// }

// void loop() {
//   // Clear the trigPin
//   digitalWrite(trigPin, LOW);
//   delayMicroseconds(2);

//   // Send 10µs HIGH pulse
//   digitalWrite(trigPin, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(trigPin, LOW);

//   // Read echo time (30ms timeout ~ 5m)
//   duration = pulseIn(echoPin, HIGH, 30000UL);

//   // Optional: calculate distance (already your friend’s issue)
//   distance = (duration * 0.0343) / 2;

//   delay(500); // readability
// }


