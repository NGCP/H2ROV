#ifndef PID_H
#define PID_H

#include "Motors.h"

/* PID Parameters */
#define KP_ROLL 1
#define KI_ROLL 1
#define KD_ROLL 1

#define KP_PITCH 1
#define KI_PITCH 1
#define KD_PITCH 1

#define KP_YAW 1
#define KI_YAW 1
#define KD_YAW 1

#define PID_MAX 5760
#define ANGLE_SCALE 16.0f

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
extern volatile Errors errors;
extern volatile int16_t eul_angles[3];

//Setpoints calc_setpoint();
//Errors calc_error();

/* Initialize PID Controller */
void init_pid();

/* Calculate Correction Values */
void pid_calculate();

#endif