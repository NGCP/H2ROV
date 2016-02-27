#define PARITY_SHIFT 0
#define MOTOR_SHIFT 1
#define SPEED_SHIFT 11
#define LIGHT_SHIFT 14
#define COMMAND_SIZE 32

// motor command constants
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

typedef struct Motor {
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
   int speed;
} Motor;

class Command {
   public:
      Command();
      bool parse_command();
      Motor get_motor_command();

   private:
      int command;
      Motor motor_command;
      bool check_parity();
      bool check_bit(int bitshift);
      void parse_light();
      void parse_speed();
      void parse_motor();
};
