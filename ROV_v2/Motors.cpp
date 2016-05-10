#include "Motors.h"

volatile Motors motor_values;
Servo escP1;
Servo escP2;
Servo escY1;
Servo escY2;
Servo escR1;
Servo escR2;

/* Convert Thrust Values to PWM */
int thrust_to_pwm(float thrust, int rev) {
  int pwm = ZERO_THRUST;

  if (thrust > MIN_THRUST_ABS) {
    pwm = FWD_SLOPE * thrust + FWD_OFFSET;
  }
  else if (thrust < -MIN_THRUST_ABS) {
    pwm = REV_SLOPE * thrust + REV_OFFSET;
  }
  
  if (rev) {
    pwm = (2 * ZERO_THRUST) - pwm;
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
  else if (thrust < MIN_THRUST_ABS && thrust > -MIN_THRUST_ABS) {
    output = 0;
  }

  return output;
}

/* Converts User Speed to Thrust */
float speed_to_thrust(int user_speed) {
  float thrust;

  if (user_speed <= MAX_SPEED && user_speed >= MIN_SPEED) {
    thrust = user_speed * (MAX_USER_THRUST / (float)NUM_SPEEDS);
  }
  else {
    thrust = 0;
  }

  return thrust;
}

/* Calculates Compensation for Pitch */
float pitch_stabilization(float m1, float m2, int16_t pitch_angle) {
  float thrust, theta;
  
  theta = (float)pitch_angle / ANGLE_SCALE;
  theta = deg_to_rad(theta);

  thrust = 0.5 * (m1 + m2) * tan(theta);
  
  return thrust;
}

/* Calculate Thrust Based on User Commands */
float lateral_thrust(int positive, int negative, int user_speed) {
  float thrust;

  if (positive) {
    thrust = speed_to_thrust(user_speed);
  }
  else if (negative) {
    thrust = -speed_to_thrust(user_speed);
  }
  else {
    thrust = 0;
  }

  return thrust;
}

/* Convert Radians to Degrees */
float rad_to_deg(float rad) {
  return rad * 180.0f / M_PI;
}

/* Convert Degrees to Radians */
float deg_to_rad(float deg) {
  return (deg * M_PI) / 180.0f;
}

/* Calculates Pitch Motor Thrust for Vertical Stabilization */
float buoy_compensation() {
  float thrust, theta, compensation;

  theta = (float)eul_angles[PITCH_DATA] / ANGLE_SCALE;
  theta = deg_to_rad(theta);

  compensation = (tan(theta) * sin(theta) + cos(theta));

  thrust = (BUOY / compensation) / 2.0f;

  return thrust;
}

/* Calculate Desired Thrust and PWM Values */
void motor_calculation(User_Commands user_commands) {
  float m1, m2, m3, m4, m5, m6;
  float surge_thrust, sway_thrust, heave_thrust;
  float depth_ref = buoy_compensation();
  float pitch_correction, pitch_angle;
  int user_speed = user_commands.user_speed;

  /* Calculate Thrust Based on User Commands */
  surge_thrust = lateral_thrust(user_commands.forward, user_commands.backward, user_speed);
  //Serial.println(surge_thrust);
  sway_thrust = lateral_thrust(user_commands.right, user_commands.left, user_speed);

  /* Heave */
  if (user_commands.up) {
    heave_thrust = -MAX_USER_THRUST;
  }
  else if (user_commands.down) {
    heave_thrust = MAX_USER_THRUST - depth_ref;
  }
  else {
    heave_thrust = 0;
  }

  /* Calculate Thrust Values */
  if (user_commands.pid) {
    m1 = depth_ref + pid_output.depth_corr + heave_thrust - pid_output.pitch_corr;
    m2 = depth_ref + pid_output.depth_corr + heave_thrust + pid_output.pitch_corr;

    pitch_correction = pitch_stabilization(m1, m2, eul_angles[PITCH_DATA]);

    m3 = surge_thrust + pid_output.yaw_corr - pitch_correction;
    m4 = surge_thrust - pid_output.yaw_corr - pitch_correction;
    m5 = sway_thrust - pid_output.roll_corr;
    m6 = sway_thrust + pid_output.roll_corr;
  }
  /* PID Off */
  else {
    m1 = heave_thrust;
    m2 = heave_thrust;
    m3 = surge_thrust;
    m4 = surge_thrust;
    m5 = sway_thrust;
    m6 = sway_thrust;
  }

  /* Verifies Thrust Values are Within Range */
  m1 = verify_thrust(m1);
  m2 = verify_thrust(m2);
  m3 = verify_thrust(m3);
  m4 = verify_thrust(m4);
  m5 = verify_thrust(m5);
  m6 = verify_thrust(m6);

  /* Calculate PWM Values Based On Thrust */
  motor_values.m1_pwm = verify_pwm(thrust_to_pwm(m1, FWD));
  motor_values.m2_pwm = verify_pwm(thrust_to_pwm(m2, REV));
  motor_values.m3_pwm = verify_pwm(thrust_to_pwm(m3, FWD));
  motor_values.m4_pwm = verify_pwm(thrust_to_pwm(m4, REV));
  motor_values.m5_pwm = verify_pwm(thrust_to_pwm(m5, FWD));
  motor_values.m6_pwm = verify_pwm(thrust_to_pwm(m6, REV));
}

/* Send PWM Values to ESCs */
void set_motor_speed(bool power) {
  
  if (!power) {
    disable_motors();
  }

  escP1.writeMicroseconds(motor_values.m1_pwm);
  escP2.writeMicroseconds(motor_values.m2_pwm);
  escY1.writeMicroseconds(motor_values.m3_pwm);
  escY2.writeMicroseconds(motor_values.m4_pwm);
  escR1.writeMicroseconds(motor_values.m5_pwm);
  escR2.writeMicroseconds(motor_values.m6_pwm);
  
#ifdef DEBUG_PWM
  Serial.print(motor_values.m1_pwm);
  Serial.print("   ");
  Serial.print(motor_values.m2_pwm);
  Serial.print("   ");
  Serial.print(motor_values.m3_pwm);
  Serial.print("   ");
  Serial.print(motor_values.m4_pwm);
  Serial.print("   ");
  Serial.print(motor_values.m5_pwm);
  Serial.print("   ");
  Serial.print(motor_values.m6_pwm);
  Serial.println();
#endif
}

/* Send Zero Command to All Thrusters */
void disable_motors() {
  motor_values.m1_pwm = ZERO_THRUST;
  motor_values.m2_pwm = ZERO_THRUST;
  motor_values.m3_pwm = ZERO_THRUST;
  motor_values.m4_pwm = ZERO_THRUST;
  motor_values.m5_pwm = ZERO_THRUST;
  motor_values.m6_pwm = ZERO_THRUST;
}
