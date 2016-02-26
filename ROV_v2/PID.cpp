#include "PID.h"

volatile PID_Out pid_output = {0, 0, 0};
volatile Errors errors = {0, 0, 0};
volatile int16_t eul_angles[3] = {0, 0, 0};

void init_pid() {

}

float pid_to_thrust(float pid_value) {
  float thrust;

  thrust = 0.5 * pid_value * ((float)MAX_THRUST_ABS / (float)PID_MAX);

  return thrust;
}

void pid_calculate() {
  float roll_pid, pitch_pid, yaw_pid;

  roll_pid = KP_ROLL * errors.roll_err;
  pitch_pid = KP_PITCH * errors.pitch_err;
  yaw_pid = KP_YAW * errors.yaw_err;

  pid_output.roll_corr = pid_to_thrust(roll_pid);
  pid_output.pitch_corr = pid_to_thrust(pitch_pid);
  pid_output.yaw_corr = pid_to_thrust(yaw_pid);
}