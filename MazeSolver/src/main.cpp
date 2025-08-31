#include <Arduino.h>

// Define pins
const int trigPin = 2;
const int echoPin = 3;

// Variables for duration and distance
long duration;
float distance;

void setup() {

}

void loop() {
   // Calculate distance in cm (speed of sound = 343 m/s)
  distance = duration * 0.0343 / 2;

  // Print the distance on Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(500); // Delay for readability

}

void loop() {
  // Clear the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Send 10µs HIGH pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read echo time (30ms timeout ~ 5m)
  duration = pulseIn(echoPin, HIGH, 30000UL);

  // Optional: calculate distance (already your friend’s issue)
  distance = (duration * 0.0343) / 2;

  delay(500); // readability
}

