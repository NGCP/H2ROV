#include "Command.h"

/* Default Constructor */
Command::Command() {
  Serial1.begin(9600);

  command = 0;
  memset(&user_commands, 0, sizeof(User_Commands));
}

/* Checks for High Bit */
bool Command::check_bit(int bitshift) {
  return (command >> bitshift) & 1;
}

/* Checks for Correct Parity (Error Detection) */
bool Command::check_parity() {
  int count, i;
  bool parity = check_bit(PARITY_SHIFT);

  for (i = 1; i < COMMAND_SIZE; i++) {
    if (check_bit(i)) {
      count++;
    }
  }
  
  /* Return True If Parity Matches */
  if (count % 2) {
    return parity;
  }
  else {
    return !parity;
  }
}

/* Checks Light Bit */
void Command::parse_light() {
  if (check_bit(LIGHT_SHIFT)) {
    // turn_on_light()
  }
  else {
    // turn_off_light()
  }
}

/* Checks Speed Bits */
void Command::parse_speed() {
  user_commands.speed = (command >> SPEED_SHIFT) & SPEED_MASK;
}

/* Parses User Motion Data */
void Command::parse_motor() {
  user_commands.down = check_bit(DOWN);
  user_commands.up = check_bit(UP);
  user_commands.backward = check_bit(BACKWARD);
  user_commands.forward = check_bit(FORWARD);
  user_commands.right = check_bit(RIGHT);
  user_commands.left = check_bit(LEFT);
  user_commands.pitch_up = check_bit(PITCH_UP);
  user_commands.pitch_down = check_bit(PITCH_DOWN);
  user_commands.yaw_right = check_bit(YAW_RIGHT);
  user_commands.yaw_left = check_bit(YAW_LEFT);
}

/* Parses Command Vector */
bool Command::parse_command() {
  command = Serial1.read();

  if (!check_parity()) {
    return false;
  }

  parse_light();
  parse_speed();
  parse_motor();

  return true;
}

/* Public Method to Get User Command Struct */
User_Commands Command::get_user_commands() {
  return user_commands;
}