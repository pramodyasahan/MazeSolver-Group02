#ifndef LINE_H
#define LINE_H

#include <Arduino.h>

// Prototypes for line-following functions
void lineFollowingMode();
void readSensors();
float getLineError();
void setMotorSpeeds(int leftSpeed, int rightSpeed);
bool anyOnLine();
bool centerOnLine();
uint8_t countBlk(const uint8_t idxs[], uint8_t n);
bool strongLeftFeature();
bool strongRightFeature();
void brake(uint16_t ms);
void pivotLeftUntilCenter();
void pivotRightUntilCenter();
void searchWhenLost();
void pivotLeft90UntilLine(int pwmVal);
void pivotRight90UntilLine(int pwmVal);

// Extern globals used in line.cpp
extern const uint8_t sensorPins[8];
extern uint8_t sensorVal[8];
extern uint8_t sensorBits;
extern const bool BLACK_IS_HIGH;
extern const uint8_t CENTER_IDX[2];
extern const uint8_t LEFT_IDX[3];
extern const uint8_t RIGHT_IDX[3];
extern const uint8_t LEFT_STRONG_MIN;
extern const uint8_t RIGHT_STRONG_MIN;
extern const int TURN_PWM_LINE;
extern const uint16_t PIVOT_TIMEOUT_MS;
extern const uint16_t BRAKE_MS;

extern volatile long encoderCountL;
extern volatile long encoderCountR;
extern const float WHEEL_DIAMETER_CM;
extern const float WHEEL_BASE_CM;
extern const int countsPerRev;

#endif // LINE_H
