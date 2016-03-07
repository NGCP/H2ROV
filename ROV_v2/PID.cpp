#include "PID.h"

volatile PID_Out pid_output = {0, 0, 0};
volatile Setpoints setpoints = {0, 0, 0};
volatile Setpoints ref_sp = {0, 0, 0};
volatile Errors errors = {0, 0, 0};
volatile int16_t eul_angles[3] = {0, 0, 0};

/* Initialize PID Controller */
void init_pid() {
  int16_t imu_data[3] = {0, 0, 0};
  
  /* Read Initial IMU Data */
  readEulData(imu_data);
  
  /* Calibrate IMU */
  ref_sp.roll_sp = imu_data[ROLL_DATA];
  ref_sp.pitch_sp = imu_data[PITCH_DATA];
  
  /* Initialize Yaw Setpoint */
  setpoints.yaw_sp = imu_data[YAW_DATA];
}

/* Calculates Roll, Pitch, and Yaw Setpoints */
void calculate_setpoints(User_Commands user_commands) {
  int16_t imu_data[3] = {0, 0, 0};
  
  /* Read IMU Data */
  readEulData(imu_data);
  
  /* Save IMU Data */
  for (int i = 0; i < 3; i++) {
    eul_angles[i] = imu_data[i];
  }
  
  /* Roll Setpoint */
  setpoints.roll_sp = ref_sp.roll_sp;
  
  /* Pitch Setpoint (0 or +/- 45 Degrees) */
  if (user_commands.pitch_up) {
    setpoints.pitch_sp = PITCH_SET;
  }
  else if (user_commands.pitch_down) {
    setpoints.pitch_sp = -PITCH_SET;
  }
  else {
    setpoints.pitch_sp = ref_sp.pitch_sp;
  }
  
  /* Yaw Setpoint */
  if (user_commands.yaw_right) {
    setpoints.yaw_sp = eul_angles[YAW_DATA] + YAW_SET;
  }
  else if (user_commands.yaw_left) {
    setpoints.yaw_sp = eul_angles[YAW_DATA] - YAW_SET;
  }
  
  /* Limit Yaw Setpoint (0 -> 360 Degrees) */
  if (setpoints.yaw_sp > YAW_MAX) {
    setpoints.yaw_sp -= YAW_MAX;
  }
  else if (setpoints.yaw_sp < 0) {
    setpoints.yaw_sp += YAW_MAX;
  }
}

/* Calculate Angular Errors */
void calculate_errors() {
  
  /* Roll Error Calculation */
  errors.roll_err = setpoints.roll_sp - eul_angles[ROLL_DATA];
  
  /* Pitch Error Calculation */
  errors.pitch_err = setpoints.pitch_sp - eul_angles[PITCH_DATA];
  
  /* Yaw Error Calculation */
  errors.yaw_err = setpoints.yaw_sp - eul_angles[YAW_DATA];
  
  /* Yaw Thresholding */
  if (errors.yaw_err < -YAW_LIMIT) {
    errors.yaw_err += YAW_MAX;
  }
  else if (errors.yaw_err > YAW_LIMIT) {
    errors.yaw_err -= YAW_MAX;
  }
}

/* Convert Correction Values to Thrust */
float pid_to_thrust(float pid_value) {
  float thrust;

  thrust = 0.5 * pid_value * ((float)MAX_THRUST_ABS / (float)PID_MAX);

  return thrust;
}

/* Calculate Correction Values */
void pid_calculate(User_Commands user_commands) {
  float roll_pid, pitch_pid, yaw_pid;
  
  calculate_setpoints(user_commands);
  calculate_errors();

  roll_pid = KP_ROLL * errors.roll_err;
  pitch_pid = KP_PITCH * errors.pitch_err;
  yaw_pid = KP_YAW * errors.yaw_err;
  
//  Serial.print(roll_pid);
//  Serial.print("   ");
//  Serial.print(pitch_pid);
//  Serial.print("   ");
//  Serial.print(yaw_pid);
//  Serial.println();

  pid_output.roll_corr = pid_to_thrust(roll_pid);
  pid_output.pitch_corr = pid_to_thrust(pitch_pid);
  pid_output.yaw_corr = pid_to_thrust(yaw_pid);
}
