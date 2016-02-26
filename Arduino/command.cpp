#include <string.h>
#include <Arduino.h>
#include "command.h"

// default constructor:
// starts up serial communication and sets members to 0
Command::Command() {
   Serial.begin(9600);

   command = 0;
   memset(&motor_command, 0, sizeof(Motor));
}

bool Command::parse_command() {
   command = Serial.read();
   command = command;
   if(!check_parity())
      return false;

   parse_light();
   parse_speed();
   parse_motor();

   return true;
}

// returns true if set parity is same as checked parity
// false is set parity is different (set as even, but received odd, etc)
bool Command::check_parity() {
   int count, i;
   bool parity = check_bit(PARITY_SHIFT);

   for(i = 1; i < COMMAND_SIZE; i++) {
      if(check_bit(i))
         count++;
   }
   
   // if count is odd and parity is odd, return true, vice versa
   if(count % 2)
      return parity;
   else
      return !parity;
}

void Command::parse_light() {
   if(check_bit(LIGHT_SHIFT)) {
      // turn_on_light()
   }
   else {
      // turn_off_light()
   }
}

void Command::parse_speed() {
   motor_command.speed = (command >> SPEED_SHIFT) & SPEED_MASK;
}

void Command::parse_motor() {
   motor_command.down = check_bit(DOWN);
   motor_command.up = check_bit(UP);
   motor_command.backward = check_bit(BACKWARD);
   motor_command.forward = check_bit(FORWARD);
   motor_command.right = check_bit(RIGHT);
   motor_command.left = check_bit(LEFT);
   motor_command.pitch_up = check_bit(PITCH_UP);
   motor_command.pitch_down = check_bit(PITCH_DOWN);
   motor_command.yaw_right = check_bit(YAW_RIGHT);
   motor_command.yaw_left = check_bit(YAW_LEFT);
}

Motor Command::get_motor_command() {
   return motor_command;
}

bool Command::check_bit(int bitshift) {
   return (command >> bitshift) & 1;
}
