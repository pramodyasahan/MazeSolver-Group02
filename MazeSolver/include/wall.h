#ifndef WALL_H
#define WALL_H

#include <Arduino.h>

// Prototypes for wall-related functions
void wallFollowingMode();
bool moveForwardWallFollow(int distance_cm, int pwmVal);
void pivot180(int pwmVal);
void smoothTurnLeft();
void smoothTurnRight();
void pivotTurn90(bool leftTurn, int pwmOuterMax);

// Motor helpers
void moveForward(int pwmValR, int pwmValL);
void moveBackward(int pwmVal);
void pivotLeft(int pwmVal);
void pivotRight(int pwmVal);

// Ultrasonic and encoder callbacks (definitions in wall.cpp)
long readUltrasonic(int trigPin, int echoPin);
void countEncoderL();
void countEncoderR();
void stopMotors();

// Extern globals from main.cpp used in wall.cpp
extern const float WHEEL_DIAMETER_CM;
extern const float WHEEL_BASE_CM;
extern const int countsPerRev;
extern volatile long encoderCountL;
extern volatile long encoderCountR;

#endif // WALL_H