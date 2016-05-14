#ifndef COMMAND_H
#define COMMAND_H

#include <string.h>
#include <Arduino.h>

#define PARITY_SHIFT 0
#define MOTOR_SHIFT 1
#define SPEED_SHIFT 11
#define LIGHT_SHIFT 14
#define POWER_SHIFT 15
#define DEPTH_SHIFT 16
#define PID_SHIFT 17
#define TUNE_SHIFT 18
#define COMMAND_SIZE 32

/* Motor Command Constants */
#define MOTOR_SIZE 10
#define DOWN 1
#define UP 2 
#define BACKWARD 3
#define FORWARD 4
#define RIGHT 5
#define LEFT 6
#define PITCH_UP 7
#define PITCH_DOWN 8
#define YAW_RIGHT 9
#define YAW_LEFT 10
#define SPEED_MASK 7

/* Parsed User Command Data */
typedef struct User_Commands {
  bool down;
  bool up;
  bool backward;
  bool forward;
  bool right;
  bool left;
  bool pitch_up;
  bool pitch_down;
  bool yaw_right;
  bool yaw_left;
  bool power;
  bool hold_depth;
  bool pid;
  bool tune;
  int user_speed;
} User_Commands;

/* User Command Class */
class Command {
  public:
    Command();
    bool parse_command();
    User_Commands get_user_commands();

  private:
    User_Commands user_commands;
    unsigned long command;
    bool check_parity();
    bool check_bit(int bitshift);
    void parse_light();
    void parse_power();
    void parse_tune();
    void parse_depth();
    void parse_speed();
    void parse_motor();
};

#endif
