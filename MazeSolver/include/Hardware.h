// Hardware.h
#pragma once
#include "Config.h"

// ============================================================================
// Motors
// ============================================================================
void initMotors();
void stopMotors();
void moveForward(int pwmRight, int pwmLeft);
void pivotLeft(int pwmVal);
void pivotRight(int pwmVal);
void pivot180(int pwmVal);
void moveForwardDistance(int distance_cm, int pwmVal);
void pivotTurn90(bool leftTurn, int pwmOuterMax);
void smoothTurnLeft();
void smoothTurnRight();

// ============================================================================
// Encoders
// ============================================================================
extern volatile long encoderCountL;
extern volatile long encoderCountR;

void initEncoders();
void resetEncoders();
long getEncoderAvg();
long absl(long x);

// ============================================================================
// Ultrasonic
// ============================================================================
void initUltrasonic();
long readUltrasonic(int trigPin, int echoPin);
long readFrontDistance();
long readLeftDistance();
long readRightDistance();
