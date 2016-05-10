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
  
  Serial.print("Depth: ");
  Serial.print(user_commands.hold_depth);
  Serial.print("   PID: ");
  Serial.print(user_commands.pid);
  Serial.println();

  /* PID Correction */
  pid_calculate(user_commands);

  /* Motor Calculation */
  motor_calculation(user_commands);
  
  /* Actuate Motors */
  set_motor_speed(user_commands.power);
  
//  Serial.print((int)(imu_data[ROLL_DATA] / 16.0));
//  Serial.print("   ");
//  Serial.print((int)(imu_data[PITCH_DATA] / 16.0));
//  Serial.print("   ");
//  Serial.print((int)(imu_data[YAW_DATA] / 16.0));
//  Serial.println();
}
