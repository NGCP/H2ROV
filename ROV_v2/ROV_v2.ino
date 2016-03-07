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
  Serial.begin(115200);

  /* Calibrate IMU? */
}

/* Main Loop */
void loop() {
  if (Serial1.available() >= 4) {
    command.parse_command();
    user_commands = command.get_user_commands();
    Serial.print("Forward: ");
    Serial.print(user_commands.forward);
    Serial.print("   ");
    Serial.print("Reverse: ");
    Serial.print(user_commands.backward);
    Serial.print("   ");
    Serial.print("Left: ");
    Serial.print(user_commands.left);
    Serial.print("   ");
    Serial.print("Right: ");
    Serial.print(user_commands.right);
    Serial.println();
  }

  /* Calculate Setpoint */
  

  /* Calculate Error */

  /* PID Correction */
  //pid_calculate();

  /* Motor Calculation */
  //motor_calculation(user_commands);
}
