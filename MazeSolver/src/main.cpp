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
