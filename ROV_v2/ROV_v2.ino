#include <Wire.h>
#include <Servo.h>
#include "Command.h"
#include "PID.h"
#include "IMU_ROV.h"
#include "Depth.h"

Command command;
User_Commands user_commands;

/* Initialization */
void setup() {
  init_motors();
  IMU_setup();
  _delay_ms(1000);
  init_pid();
  Serial.begin(115200);
}

/* Main Loop */
void loop() {
  /* Get User Commands */
  command.parse_command();
  user_commands = command.get_user_commands();

  /* Tune PID */
  if (user_commands.tune) {
    tune_all(user_commands);
    user_commands.tune = false;
  }

  /* PID Correction */
  pid_calculate(user_commands);

  /* Motor Calculation */
  motor_calculation(user_commands);
  
  /* Actuate Motors */
  set_motor_speed(user_commands.power);
}
