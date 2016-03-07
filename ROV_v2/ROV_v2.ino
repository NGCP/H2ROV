#include <Wire.h>
#include <Servo.h>
#include "Command.h"
#include "PID.h"
#include "IMU_ROV.h"

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
  
  //user_commands = 0;
  
  //Serial.println(user_commands.user_speed);
  
//  Serial.print("Forward: ");
//  Serial.print(user_commands.forward);
//  Serial.print("   ");
//  Serial.print("Reverse: ");
//  Serial.print(user_commands.backward);
//  Serial.print("   ");
//  Serial.print("Left: ");
//  Serial.print(user_commands.left);
//  Serial.print("   ");
//  Serial.print("Right: ");
//  Serial.print(user_commands.right);
//  Serial.println();

  /* PID Correction */
  pid_calculate(user_commands);

  /* Motor Calculation */
  motor_calculation(user_commands);
  
  /* Actuate Motors */
  set_motor_speed();
}
