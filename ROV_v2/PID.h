#ifndef PID_H
#define PID_H

#include "Motors.h"
#include "Command.h"
#include "IMU_ROV.h"

/* Uncomment to Print PID Values */
//#define DEBUG_PID

/* PID Parameters */
#define KP_ROLL 5
#define KI_ROLL 1
#define KD_ROLL 1

#define KP_PITCH 5
#define KI_PITCH 1
#define KD_PITCH 1

#define KP_YAW 5
#define KI_YAW 1
#define KD_YAW 1

#define PID_MAX 5760
#define INTEGRAL_MAX 160
#define ANGLE_SCALE 16.0f
#define ANGLE_OFFSET 180
#define MAX_ANGLE 360
#define MAX_BYTE 255

#define PITCH_SET 720
#define YAW_SET 96
#define YAW_LIMIT 2880
#define YAW_MAX 5760

#define YAW_DATA 0
#define ROLL_DATA 1
#define PITCH_DATA 2

#define BATTERY_PIN 0
#define MIN_VOLTAGE 3.0f
#define MAX_VOLTAGE 5.0f
#define MAX_ANALOG 1023
#define MAX_PERCENT 100.0f
#define BATTERY_SAMPLES 50

/* Angular Setpoints */
typedef struct Setpoints {
  int16_t roll_sp;
  int16_t pitch_sp;
  int16_t yaw_sp;
} Setpoints;

/* Angular Errors (PV - SP) */
typedef struct Errors {
  int16_t roll_err;
  int16_t pitch_err;
  int16_t yaw_err;
}Errors;

/* PID Correction Values (THRUST) */
/* Values Already Halved (Dual Motor Symmetry) */
typedef struct PID_Out {
  float roll_corr;
  float pitch_corr;
  float yaw_corr;
}PID_Out;

extern volatile PID_Out pid_output;
extern volatile Setpoints setpoints;
extern volatile Errors errors;
extern volatile int16_t eul_angles[3];
extern volatile int16_t prev_angles[3];
extern volatile int16_t error_sum[3];
extern volatile int battery_percent;
extern volatile int battery_sum;
extern volatile int battery_count;

/* Initialize PID Controller */
void init_pid();

/* Calculates Roll, Pitch, and Yaw Setpoints */
void calculate_setpoints(User_Commands user_commands);

/* Thresholds Integral Error Sum */
int16_t threshold_integral_error(int16_t error);

/* Calculate Angular Errors */
void calculate_errors();

/* Convert Correction Values to Thrust */
float pid_to_thrust(float pid_value);

/* Calculate Correction Values */
void pid_calculate(User_Commands user_commands);

#endif