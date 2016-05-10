#ifndef PID_H
#define PID_H

#include "Motors.h"
#include "Command.h"
#include "IMU_ROV.h"
#include "Depth.h"
#include "Battery.h"

/* Uncomment to Print PID Values */
//#define DEBUG_PID
//#define DEBUG_BATTERY
//#define DEBUG_DEPTH

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

#define KP_DEPTH 10
#define KI_DEPTH 2
#define KD_DEPTH 1

#define PID_MAX 5760
#define INTEGRAL_MAX 160
#define DEPTH_INTEGRAL_MAX 100
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

/* Setpoints */
typedef struct Setpoints {
  int16_t roll_sp;
  int16_t pitch_sp;
  int16_t yaw_sp;
  float depth_sp;
} Setpoints;

/* Angular Errors (PV - SP) */
typedef struct Errors {
  int16_t roll_err;
  int16_t pitch_err;
  int16_t yaw_err;
  float depth_err;
}Errors;

/* PID Correction Values (THRUST) */
/* Values Already Halved (Dual Motor Symmetry) */
typedef struct PID_Out {
  float roll_corr;
  float pitch_corr;
  float yaw_corr;
  float depth_corr;
}PID_Out;

extern volatile PID_Out pid_output;
extern volatile Setpoints setpoints;
extern volatile Errors errors;
extern volatile int16_t eul_angles[3];
extern volatile int16_t prev_angles[3];
extern volatile int16_t error_sum[3];

extern volatile float prev_depth = 0;
extern volatile float depth_error_sum = 0;

/* Initialize Depth Sensor */
void init_depth();

/* Initialize PID Controller */
void init_pid();

/* Sends IMU Data to BBB */
void send_system_data(int16_t *imu_data);

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
