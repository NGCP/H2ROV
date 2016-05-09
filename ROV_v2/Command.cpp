#include "Command.h"

/* Default Constructor */
Command::Command() {
  Serial1.begin(115200);

  command = 0;
  memset(&user_commands, 0, sizeof(User_Commands));
  user_commands.power = false;
  user_commands.hold_depth = false;
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

void Command::parse_power() {
  user_commands.power = check_bit(POWER_SHIFT);
}

void Command::parse_depth() {
  user_commands.hold_depth = check_bit(DEPTH_SHIFT);
}

/* Checks Speed Bits */
void Command::parse_speed() {
  user_commands.user_speed = (command >> SPEED_SHIFT) & SPEED_MASK;
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
  byte data[4];
  
  /* Initialize Data to Zero */
  for (int i = 0; i < 4; i++) {
    data[i] = 0;
  }
  
  if (Serial1.available() >= 4) {
    Serial1.readBytes(data, 4);
    command = 0;
    command |= (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
  }

  if (!check_parity()) {
    return false;
  }

  parse_light();
  parse_power();
  parse_speed();
  parse_motor();

  return true;
}

/* Public Method to Get User Command Struct */
User_Commands Command::get_user_commands() {
  return user_commands;
}
