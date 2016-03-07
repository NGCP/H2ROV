#ifndef MOTORS_H
#define MOTORS_H

#include <Servo.h>
#include <math.h>
#include "PID.h"
#include "Command.h"

#define M1_PIN 8
#define M2_PIN 9
#define M3_PIN 10
#define M4_PIN 11
#define M5_PIN 12
#define M6_PIN 13

/* Linear Approximations of PWM vs. Thrust (Forward) */
#define FWD_SLOPE 61.1f
#define FWD_OFFSET 1571.466f

/* Linear Approximations of PWM vs. Thrust (Reverse) */
#define REV_SLOPE 82.3f
#define REV_OFFSET 1400.534f

/* Thrust Limits */
#define MIN_THRUST_ABS 0.15f
#define MAX_THRUST_ABS 3.8f
#define MAX_USER_THRUST 2.8f
#define ZERO_THRUST 1500
#define MIN_PWM 1100
#define MAX_PWM 1900

/* User Speed Limits */
#define MIN_SPEED 1
#define MAX_SPEED 7
#define NUM_SPEEDS 7

/* Physical Considerations */
#define BUOY 0.0f
#define MAX_PITCH 45

typedef struct Motors {
  int m1_pwm; /* Pitch 1 */
  int m2_pwm; /* Pitch 2 */
  int m3_pwm; /* Yaw 1   */
  int m4_pwm; /* Yaw 2   */
  int m5_pwm; /* Roll 1  */
  int m6_pwm; /* Roll 2  */
}Motors;

extern volatile Motors motor_values;
extern Servo escP1;
extern Servo escP2;
extern Servo escY1;
extern Servo escY2;
extern Servo escR1;
extern Servo escR2;

/* Convert Thrust Values to PWM */
int thrust_to_pwm(float thrust);

/* Initialize Motors and PWM Data */
void init_motors();

/* Verifies PWM is Within Range */
int verify_pwm(int pwm);

/* Verifies Thrust is Within Range */
float verify_thrust(float thrust);

/* Converts User Speed to Thrust */
float speed_to_thrust();

/* Calculates Compensation for Pitch */
float pitch_stabilization(int16_t pitch_angle);

/* Calculate Thrust Based on User Commands */
float lateral_thrust(int positive, int negative, int speed);

/* Convert Radians to Degrees */
float rad_to_deg(float rad);

/* Convert Degrees to Radians */
float deg_to_rad(float deg);

/* Calculates Pitch Motor Thrust for Vertical Stabilization */
float buoy_compensation();

/* Calculate Desired Thrust and PWM Values */
void motor_calculation(User_Commands user_commands);

/* Send PWM Values to ESCs */
void set_motor_speed();

#endif
