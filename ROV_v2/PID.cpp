#include "PID.h"

volatile PID_Out pid_output = {0, 0, 0};
volatile Setpoints setpoints = {0, 0, 0};
volatile Setpoints ref_sp = {0, 0, 0};
volatile Errors errors = {0, 0, 0};
volatile int16_t eul_angles[3] = {0, 0, 0};
volatile int16_t prev_angles[3] = {0, 0, 0};
volatile int16_t error_sum[3] = {0, 0, 0};
extern volatile int battery_percent = 0;
volatile int battery_sum = 0;
volatile int battery_count = 0;

/* Initialize PID Controller */
void init_pid() {
  int16_t imu_data[3] = {0, 0, 0};
  
  /* Read Initial IMU Data */
  readEulData(imu_data);

  /* Save IMU Data */
  for (int i = 0; i < 3; i++) {
    eul_angles[i] = imu_data[i];
  }
  
  /* Calibrate IMU */
  ref_sp.roll_sp = imu_data[ROLL_DATA];
  ref_sp.pitch_sp = imu_data[PITCH_DATA];
  
  /* Initialize Yaw Setpoint */
  setpoints.yaw_sp = imu_data[YAW_DATA];
}

/* Get Battery Level */
uint8_t get_battery_level() {
  int percent, analog_input;
  float input_voltage;
 
  /* Read Voltage Divider Input */
  analog_input = analogRead(BATTERY_PIN);
  
  /* Convert Analog Values to Voltage */
  input_voltage = analog_input * (MAX_VOLTAGE / MAX_ANALOG);
  
  if (input_voltage < MIN_VOLTAGE) {
    input_voltage = MIN_VOLTAGE;
  }
  
  /* Calculate Battery Percentage */
  percent = (input_voltage - MIN_VOLTAGE) * (MAX_PERCENT / (MAX_VOLTAGE - MIN_VOLTAGE));
  
  /* Keep Track of Samples for Running Average */
  battery_sum += percent;
  battery_count++;

  /* Calculate Battery Percentage Over Sample Period */
  if (battery_count == BATTERY_SAMPLES) {
    percent = battery_sum / battery_count;
    battery_percent = percent;
    battery_sum = 0;
    battery_count = 0;
  }
  
  return (uint8_t)battery_percent;
}

/* Sends IMU Data to BBB */
void send_IMU_data(int16_t *imu_data) {
  uint8_t roll, pitch, yaw, battery;
  uint8_t rpy[4] = {0, 0, 0, 0};
  
  roll = ((imu_data[ROLL_DATA] / ANGLE_SCALE) + ANGLE_OFFSET) * ((float)MAX_BYTE / (float)MAX_ANGLE);
  pitch = ((imu_data[PITCH_DATA] / ANGLE_SCALE) + ANGLE_OFFSET) * ((float)MAX_BYTE / (float)MAX_ANGLE);
  yaw = ((imu_data[YAW_DATA] / ANGLE_SCALE)) * ((float)MAX_BYTE / (float)MAX_ANGLE);
  
  battery = get_battery_level();
  
  Serial1.write(roll);
  Serial1.write(pitch);
  Serial1.write(yaw);
  Serial1.write(battery);
  //Serial.println(battery);
}

/* Calculates Roll, Pitch, and Yaw Setpoints */
void calculate_setpoints(User_Commands user_commands) {
  int16_t imu_data[3] = {0, 0, 0};
  
  /* Read IMU Data */
  readEulData(imu_data);
  
  /* Send Angle Data Over Serial */
  send_IMU_data(imu_data);
  
  /* Save Previous IMU Data */
  for (int i = 0; i < 3; i++) {
    prev_angles[i] = eul_angles[i];
  }

  /* Save New IMU Data */
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

/* Thresholds Integral Error Sum */
int16_t threshold_integral_error(int16_t error) {
  int16_t output = error;

  if (error > INTEGRAL_MAX) {
    output = INTEGRAL_MAX;
  }
  else if (error < -INTEGRAL_MAX) {
    output = -INTEGRAL_MAX;
  }

  return output;
}

/* Calculate Angular Errors */
void calculate_errors() {
  
  /* Roll Error Calculation */
  errors.roll_err = setpoints.roll_sp - eul_angles[ROLL_DATA];
  error_sum[ROLL_DATA] += errors.roll_err;
  error_sum[ROLL_DATA] = threshold_integral_error(error_sum[ROLL_DATA]);
  
  /* Pitch Error Calculation */
  errors.pitch_err = setpoints.pitch_sp - eul_angles[PITCH_DATA];
  error_sum[PITCH_DATA] += errors.pitch_err;
  error_sum[PITCH_DATA] = threshold_integral_error(error_sum[PITCH_DATA]);
  
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
  float roll_deriv, pitch_deriv, yaw_deriv;
  
  /* Proportional Calculations */
  calculate_setpoints(user_commands);
  calculate_errors();

  /* Derivative Calculations */
  roll_deriv = eul_angles[ROLL_DATA] - prev_angles[ROLL_DATA];
  pitch_deriv = eul_angles[PITCH_DATA] - prev_angles[PITCH_DATA];

  /* Roll PID Calculation */
  roll_pid = KP_ROLL * errors.roll_err +
             KI_ROLL * error_sum[ROLL_DATA] -
             KD_ROLL * roll_deriv;

  /* Pitch PID Calculation */
  pitch_pid = KP_PITCH * errors.pitch_err +
              KI_PITCH * error_sum[PITCH_DATA] -
              KD_PITCH * pitch_deriv;

  yaw_pid = KP_YAW * errors.yaw_err;
  
#ifdef DEBUG_PID
  Serial.print(roll_pid);
  Serial.print("   ");
  Serial.print(pitch_pid);
  Serial.print("   ");
  Serial.print(yaw_pid);
  Serial.println();
#endif

  pid_output.roll_corr = pid_to_thrust(roll_pid);
  pid_output.pitch_corr = pid_to_thrust(pitch_pid);
  pid_output.yaw_corr = pid_to_thrust(yaw_pid);
}
