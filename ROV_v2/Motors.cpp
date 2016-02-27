#include "Motors.h"

volatile Motors motor_values;
Servo escP1;
Servo escP2;
Servo escY1;
Servo escY2;
Servo escR1;
Servo escR2;

/* Convert Thrust Values to PWM */
int thrust_to_pwm(float thrust) {
  int pwm = ZERO_THRUST;

  if (thrust > MIN_THRUST_ABS) {
    pwm = FWD_SLOPE * thrust + FWD_OFFSET;
  }
  else if (thrust < -MIN_THRUST_ABS) {
    pwm = REV_SLOPE * thrust + REV_OFFSET;
  }

  return pwm;
}

/* Initialize Motors and PWM Data */
void init_motors() {
  /* Assign ESCs to Pins */
  escP1.attach(M1_PIN);
  escP2.attach(M2_PIN);
  escY1.attach(M3_PIN);
  escY2.attach(M4_PIN);
  escR1.attach(M5_PIN);
  escR2.attach(M6_PIN);

  /* Initialize Motor Values to Zero-Thrust */
  motor_values.m1_pwm = ZERO_THRUST;
  motor_values.m2_pwm = ZERO_THRUST;
  motor_values.m3_pwm = ZERO_THRUST;
  motor_values.m4_pwm = ZERO_THRUST;
  motor_values.m5_pwm = ZERO_THRUST;
  motor_values.m6_pwm = ZERO_THRUST;

  /* Initialize ESCs with Zero-Thrust Command */
  escP1.writeMicroseconds(ZERO_THRUST);
  escP2.writeMicroseconds(ZERO_THRUST);
  escY1.writeMicroseconds(ZERO_THRUST);
  escY2.writeMicroseconds(ZERO_THRUST);
  escR1.writeMicroseconds(ZERO_THRUST);
  escR2.writeMicroseconds(ZERO_THRUST);
  delay(1000);
}

/* Verifies PWM is Within Range */
int verify_pwm(int pwm) {
  int output = pwm;

  if (pwm < MIN_PWM) {
    output = MIN_PWM;
  }
  else if (pwm > MAX_PWM) {
    output = MAX_PWM;
  }

  return output;
}

/* Verifies Thrust is Within Range */
float verify_thrust(float thrust) {
  float output = thrust;

  if (thrust < -MAX_THRUST_ABS) {
    output = -MAX_THRUST_ABS;
  }
  else if (thrust > MAX_THRUST_ABS) {
    output = MAX_THRUST_ABS;
  }
  else if (thrust < MIN_THRUST_ABS || thrust > -MIN_THRUST_ABS) {
    output = 0;
  }

  return output;
}

/* Converts User Speed to Thrust */
float speed_to_thrust(int speed) {
  float thrust;

  if (speed < MAX_SPEED && speed > MIN_SPEED) {
    thrust = speed * (MAX_USER_THRUST / NUM_SPEEDS);
  }
  else {
    thrust = 0;
  }

  return thrust;
}

/* Calculates Compensation for Pitch */
float pitch_stabilization(float m1, float m2, int16_t pitch_angle) {
  float thrust;

  thrust = 0.5 * (m1 + m2) * tan((float)eul_angles[1] / ANGLE_SCALE);
}

/* Calculate Thrust Based on User Commands */
float lateral_thrust(int positive, int negative, int speed) {
  float thrust;

  if (positive) {
    thrust = speed_to_thrust(speed);
  }
  else if (negative) {
    thrust = -speed_to_thrust(speed);
  }
  else {
    thrust = 0;
  }

  return thrust;
}

/* Calculate Desired Thrust and PWM Values */
void motor_calculation(User_Commands user_commands) {
  float m1, m2, m3, m4, m5, m6;
  float surge_thrust, sway_thrust, heave_thrust;
  float depth_ref = BUOY * sin((float)eul_angles[1] / ANGLE_SCALE) / (float)2;
  float pitch_correction, pitch_angle;
  int speed = user_commands.speed;

  /* Calculate Thrust Based on User Commands */
  surge_thrust = lateral_thrust(user_commands.forward, user_commands.backward, speed);
  sway_thrust = lateral_thrust(user_commands.right, user_commands.left, speed);

  /* Heave */
  if (user_commands.up) {
    heave_thrust = -(MAX_USER_THRUST - depth_ref);
  }
  else if (user_commands.down) {
    heave_thrust = MAX_USER_THRUST - depth_ref;
  }
  else {
    heave_thrust = 0;
  }

  /* Calculate Thrust Values */
  m1 = depth_ref + heave_thrust - pid_output.pitch_corr; // NEED DEPTH CALCULATION
  m2 = depth_ref + heave_thrust + pid_output.pitch_corr; // NEED DEPTH CALCULATION

  pitch_correction = pitch_stabilization(m1, m2, eul_angles[1]);

  m3 = surge_thrust + pid_output.yaw_corr - pitch_correction;
  m4 = surge_thrust - pid_output.yaw_corr - pitch_correction;
  m5 = sway_thrust - pid_output.roll_corr;
  m6 = sway_thrust + pid_output.roll_corr;

  /* Verifies Thrust Values are Within Range */
  m1 = verify_thrust(m1);
  m2 = verify_thrust(m2);
  m3 = verify_thrust(m3);
  m4 = verify_thrust(m4);
  m5 = verify_thrust(m5);
  m6 = verify_thrust(m6);

  /* Calculate PWM Values Based On Thrust */
  motor_values.m1_pwm = verify_pwm(thrust_to_pwm(m1));
  motor_values.m2_pwm = verify_pwm(thrust_to_pwm(m2));
  motor_values.m3_pwm = verify_pwm(thrust_to_pwm(m3));
  motor_values.m4_pwm = verify_pwm(thrust_to_pwm(m4));
  motor_values.m5_pwm = verify_pwm(thrust_to_pwm(m5));
  motor_values.m6_pwm = verify_pwm(thrust_to_pwm(m6));
}

/* Send PWM Values to ESCs */
void set_motor_speed() {
  escP1.writeMicroseconds(motor_values.m1_pwm);
  escP2.writeMicroseconds(motor_values.m2_pwm);
  escY1.writeMicroseconds(motor_values.m3_pwm);
  escY2.writeMicroseconds(motor_values.m4_pwm);
  escR1.writeMicroseconds(motor_values.m5_pwm);
  escR2.writeMicroseconds(motor_values.m6_pwm);
}
