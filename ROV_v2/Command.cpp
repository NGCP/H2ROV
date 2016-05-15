#include "Command.h"

/* Default Constructor */
Command::Command() {
  Serial1.begin(115200);

  command = 0;
  memset(&user_commands, 0, sizeof(User_Commands));
  user_commands.power = false;
  user_commands.pid = true;
  user_commands.hold_depth = false;
  user_commands.tune = false;
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

void Command::parse_pid() {
  user_commands.pid = check_bit(PID_SHIFT);
}

void Command::parse_tune() {
  user_commands.tune = check_bit(TUNE_SHIFT);
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
  unsigned long temp1 = 0, temp2 = 0, temp3 = 0, temp4 = 0;
  
  /* Initialize Data to Zero */
  for (int i = 0; i < 4; i++) {
    data[i] = 0;
  }
  
  if (Serial1.available() >= 4) {
    Serial1.readBytes(data, 4);
    command = 0;
    temp1 = data[0];
    temp2 = data[1];
    temp3 = data[2];
    temp4 = data[3];
    command = (temp1 << 24) | (temp2 << 16) | (temp3 << 8) | (temp4);
    //Serial.println(command, BIN);
  }

  if (!check_parity()) {
    return false;
  }
  //Serial.println(command, BIN);
  parse_light();
  parse_power();
  parse_pid();
  parse_depth();
  parse_tune();
  parse_speed();
  parse_motor();
  
  //Serial.println(command, BIN);

  return true;
}

/* Public Method to Get User Command Struct */
User_Commands Command::get_user_commands() {
  return user_commands;
}
