#include <Wire.h>
#include <Servo.h>
#include "Command.h"
#include "PID.h"
#include "IMU_ROV.h"

Command command;
User_Commands user_commands;

/* Initialization */
void setup() {
  init_pid();
  init_motors();
  IMU_setup();

  /* Calibrate IMU? */
}

/* Main Loop */
void loop() {
  command.parse_command();
  user_commands = command.get_user_commands();

  /* Calculate Setpoint */

  /* Calculate Error */

  /* PID Correction */
  pid_calculate();

  /* Motor Calculation */
  motor_calculation(user_commands);
}
