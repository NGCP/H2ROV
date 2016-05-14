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
//#define DEBUG_ANGLES

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
#define DEPTH_DATA 3
#define NUM_GAINS 3

/* PID Tuning Definitions */
#define INITIAL_STEP 1
#define STEP_THRESHOLD 0.0001f
#define STEP_SCALE_DOWN 0.9
#define STEP_SCALE_UP 1.1
#define P_PARAM 0
#define I_PARAM 1
#define D_PARAM 2

#define FLUID_DENSITY 1029 // Saltwater: 1029  |  Freshwater: 1000

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

typedef struct PID_Gains {
  float kp_roll;
  float ki_roll;
  float kd_roll;
  float kp_pitch;
  float ki_pitch;
  float kd_pitch;
  float kp_yaw;
  float kp_depth;
  float ki_depth;
  float kd_depth;
}PID_Gains;

extern volatile PID_Out pid_output;
extern volatile Setpoints setpoints;
extern volatile Errors errors;
extern volatile int16_t eul_angles[3];
extern volatile int16_t prev_angles[3];
extern volatile int16_t error_sum[3];

extern volatile float prev_depth;
extern volatile float depth_error_sum;

extern volatile PID_Gains pid_gains;

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

/* Run System */
void run_system(User_Commands user_commands);

/* Initialize PID Tuning */
void init_tuning(float **gain_steps);

/* Return Summation of Gain Step Array */
float sum_gain_steps(float *steps);

float get_error(int PID_id);

/* Tune PID Gains */
void tune_PID(User_Commands user_commands, float **params, float **steps, int PID_id);

/* Tune All PID Controllers */
void tune_all(User_Commands user_commands);

#endif
