#ifndef COMMON_H
#define COMMON_H

#include <Arduino.h>

// -------------------- Pin definitions --------------------
#define TRIG_FRONT 48
#define ECHO_FRONT 49
#define TRIG_RIGHT 50
#define ECHO_RIGHT 51
#define TRIG_LEFT  52
#define ECHO_LEFT  53

#define L_RPWM 4
#define L_LPWM 5
#define L_REN  6
#define L_LEN  7

#define R_RPWM 2
#define R_LPWM 3
#define R_REN  9
#define R_LEN  8

#define L_ENC_A 18
#define L_ENC_B 19
#define R_ENC_A 20
#define R_ENC_B 21

#define SEARCH_MODE 33
#define RIGHT_MODE 38
#define LEFT_MODE 39

// -------------------- Robot physical / tuning --------------------
extern const float WHEEL_DIAMETER_CM;
extern const float WHEEL_BASE_CM;
extern const int pulsesPerMotorRev;
extern const int gearRatio;
extern const int countsPerRev;

extern const int TURN_PWM;
extern const float TURN_RADIUS_CM;
extern const int PIVOT_180_PWM;

extern const int OBSTACLE_THRESHOLD;
extern const int OPEN_SPACE_THRESHOLD_CM;
extern const int DEAD_END_THRESHOLD;
extern const int NO_WALL_THRESHOLD;
extern const int CORNER_CLEARANCE_CM;

extern const int BASE_PWM_STRAIGHT;
extern const float Kp_Wall;
extern const int MAX_CORRECTION;
extern const int WALL_DETECT_RANGE;

extern const int ALIGN_PWM;
extern const int ALIGN_DURATION_MS;
extern const int ALIGN_TOLERANCE_CM;

extern const int TURN_PWM_LINE;
extern const uint16_t PIVOT_TIMEOUT_MS;
extern const uint16_t BRAKE_MS;

// -------------------- Line sensor shared arrays & flags --------------------
extern const uint8_t sensorPins[8];
extern uint8_t sensorVal[8];
extern uint8_t sensorBits;
extern const bool BLACK_IS_HIGH;
extern const uint8_t CENTER_IDX[2];
extern const uint8_t LEFT_IDX[3];
extern const uint8_t RIGHT_IDX[3];
extern const uint8_t LEFT_STRONG_MIN;
extern const uint8_t RIGHT_STRONG_MIN;

// -------------------- PID & motion globals --------------------
extern float Kp;
extern float Ki;
extern float Kd;

extern int baseSpeed;
extern int maxPWM;

extern float error;
extern float lastError;
extern float integral;
extern float derivative;
extern float correction;

// -------------------- Encoder counts --------------------
extern volatile long encoderCountL;
extern volatile long encoderCountR;

// -------------------- Helper inline functions --------------------
// Define stopLeft/stopRight/absl here so every translation unit can use them.
// These are small and should be inlined by the compiler.
inline void stopRight(){ analogWrite(R_RPWM, 0); analogWrite(R_LPWM, 0); }
inline void stopLeft() { analogWrite(L_RPWM, 0); analogWrite(L_LPWM, 0); }
inline long absl(long x){ return (x < 0) ? -x : x; }

// -------------------- Prototypes for commonly used functions (defined elsewhere) ----
// (You can add more prototypes here if needed.)
long readUltrasonic(int trigPin, int echoPin);
void countEncoderL();
void countEncoderR();
void moveForwardDistance(int distance_cm, int pwmVal);
void pivot180(int pwmVal);
void smoothTurnLeft();
void smoothTurnRight();
void pivotLeft(int pwmVal);
void pivotRight(int pwmVal);
void moveForward(int pwmValR, int pwmValL);
void stopMotors();
void readSensors();

#endif // COMMON_H