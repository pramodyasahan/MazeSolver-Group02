#include <Arduino.h>
#include "common.h"
#include "line.h"
#include "wall.h"
#include "flood.h"

// ==================== Pin Definitions ====================
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

// ==================== Robot Physical Constants ====================
const float WHEEL_DIAMETER_CM = 6.5f;
const float WHEEL_BASE_CM     = 17.0f;
const int pulsesPerMotorRev = 11;
const int gearRatio         = 20;
const int countsPerRev      = pulsesPerMotorRev * gearRatio * 2;

// ==================== Turning Configuration ====================
const int   TURN_PWM       = 50;
const float TURN_RADIUS_CM = 11.0f;
const int   PIVOT_180_PWM  = 55;

// ==================== Navigation Thresholds ====================
const int OBSTACLE_THRESHOLD      = 8;
const int OPEN_SPACE_THRESHOLD_CM = 40;
const int DEAD_END_THRESHOLD      = 10;
const int NO_WALL_THRESHOLD       = 25;
const int CORNER_CLEARANCE_CM     = 10;

// ==================== Wall Following ====================
const int   BASE_PWM_STRAIGHT = 40;
const float Kp_Wall           = 3.5f;
const int   MAX_CORRECTION    = 20;
const int   WALL_DETECT_RANGE = 20;

// ==================== Alignment ====================
const int ALIGN_PWM         = 45;
const int ALIGN_DURATION_MS = 50;
const int ALIGN_TOLERANCE_CM= 1;

// ==================== Encoders ====================
volatile long encoderCountL = 0;
volatile long encoderCountR = 0;


// ====================== Line Sensors ======================
const uint8_t sensorPins[8] = {22, 23, 24, 25, 26, 27, 28, 29};
// After readSensors(): sensorVal[i] == 1 means "black", 0 means "white"
uint8_t sensorVal[8];
uint8_t sensorBits = 0; // bit i = 1 if sensor i sees black

// Set this to false if your board reads LOW on black.
const bool BLACK_IS_HIGH = true;

// Convenience groupings (0 = far-left, 7 = far-right)
const uint8_t CENTER_IDX[2] = {3, 4};
const uint8_t LEFT_IDX[3]   = {0, 1, 2};
const uint8_t RIGHT_IDX[3]  = {5, 6, 7};

// How strong a “left/right feature” must be to treat as a corner
const uint8_t LEFT_STRONG_MIN  = 2;
const uint8_t RIGHT_STRONG_MIN = 2;

// Pivot behavior at corners
const int TURN_PWM_LINE = 140;               // strong pivot power
const uint16_t PIVOT_TIMEOUT_MS = 600;  // safety stop if something goes wrong
const uint16_t BRAKE_MS = 30;

// ====================== PID Parameters ======================
float Kp = 50.0;
float Ki = 0.0;
float Kd = 4.0;

int baseSpeed = 50;   // cruise speed
int maxPWM    = 100;  // clamps for smoothness

// ====================== Variables ======================
float error = 0, lastError = 0;
float integral = 0, derivative = 0, correction = 0;

// ==================== Mode State ====================
bool wallMode = false; // false = line mode, true = wall mode

void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);

  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT); pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT);
  pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT);
  pinMode(R_REN, OUTPUT); pinMode(R_LEN, OUTPUT);
  pinMode(L_REN, OUTPUT); pinMode(L_LEN, OUTPUT);
  digitalWrite(R_REN, HIGH); digitalWrite(R_LEN, HIGH);
  digitalWrite(L_REN, HIGH); digitalWrite(L_LEN, HIGH);

  pinMode(L_ENC_A, INPUT_PULLUP); pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP); pinMode(R_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), countEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), countEncoderR, CHANGE);

  pinMode(SEARCH_MODE, INPUT_PULLUP);
  pinMode(RIGHT_MODE, INPUT_PULLUP);
  pinMode(LEFT_MODE, INPUT_PULLUP);
}

void loop() {
  // Flood controller will read SEARCH_MODE pin: LOW => scan, HIGH => solve
  lineFollowingMode();

  // keep loop idle until user toggles mode or robot finishes
  delay(200);
}