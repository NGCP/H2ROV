#include <Wire.h>
#include <Servo.h>
#include "Command.h"
#include "PID.h"
#include "IMU_ROV.h"

/* Uncomment to Print PID Values */
//#define DEBUG_PID

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
  //Serial1.print(1234567890);
  //Serial.println(user_commands.user_speed);
  
//  Serial.print("Ascend: ");
//  Serial.print(user_commands.up);
//  Serial.print("   ");
//  Serial.print("Submerge: ");
//  Serial.print(user_commands.down);
//  Serial.print("   ");
//  Serial.print("Yaw_L: ");
//  Serial.print(user_commands.yaw_left);
//  Serial.print("   ");
//  Serial.print("Yaw_R: ");
//  Serial.print(user_commands.yaw_right);
//  Serial.println();

  /* PID Correction */
  pid_calculate(user_commands);

  /* Motor Calculation */
  motor_calculation(user_commands);
  
  /* Actuate Motors */
  set_motor_speed();
}
