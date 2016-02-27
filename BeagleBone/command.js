var b = require('bonescript');


// constants used for bit masking
const parity_shift = 0;
const motor_shift = 1;
const speed_shift = 11;
const light_shift = 14;
const command_size = 32;

// motor command constants
const motor_size = 10;
const down = 1;
const up = 2;
const backward = 3;
const forward = 4;
const right = 5;
const left = 6;
const pitch_up = 7;
const pitch_down = 8;
const yaw_right = 9;
const yaw_left = 10;

var command = 0;

var port = '/dev/tty02';
var options = { 
   baudrate: 9600
};


function set_light(setting) {
   switch(setting) {
      case 'on':
         set_bit(light_shift);
         break;
      case 'off':
         clear_bit(light_shift);
         break;
      default:
         throw new Error("Invalid light setting: use 'on' or 'off'");
         break;
   }
}

function set_speed(speed) {
   // error checking
   if(speed < 0 || speed > 7)
      throw new Error('Speed setting out of range: 0-7');
   
   command |= speed << speed_shift;
}

// sets the motor command bits
// Takes in variable number of arguments
function set_motor() {
   if(arguments.length > motor_size) {
      throw new Error('Too many motor commands');
   }
   for(var i = 0; i < arguments.length; i++) {
      switch(arguments[i]) {
         case "down":
            set_bit(down);
            break;
         case "up":
            set_bit(up);
            break;
         case "backward":
            set_bit(backward);
            break;
         case "forward":
            set_bit(forward);
            break;
         case "right":
            set_bit(right);
            break;
         case "left":
            set_bit(left);
            break;
         case "pitch up":
            set_bit(pitch_up);
            break;
         case "pitch down":
            set_bit(pitch_down);
            break;
         case "yaw right":
            set_bit(yaw_right);
            break;
         case "yaw left":
            set_bit(yaw_left);
            break;
         default:
            throw new Error('Invalid motor command: ' + arguments[i]);
      }
   }

   // sanity checks
   if(check_bit(up) && check_bit(down))
      throw new Error('Cannot set both up and down');
   if(check_bit(backward) && check_bit(forward))
      throw new Error('Cannot set both backward and forward');
   if(check_bit(right) && check_bit(left))
      throw new Error('Cannot set both left and right');
   if(check_bit(pitch_up) && check_bit(pitch_down))
      throw new Error('Cannot set both pitch up and down');
   if(check_bit(yaw_right) && check_bit(yaw_left))
      throw new Error('Cannot set both yaw left and right');
}

function clear_motor() {
   for(var i = 0; i < motor_size; i++) {
      command &= ~(1 << (i + motor_shift));
   }
}

function send_command() { 
   set_parity();
   b.serialWrite(port, command);
   clear_motor();
}

function clear_command() {
   command &= 0;
}

// bit masking helper functions
function set_bit(bitshift) {
   command |= 1 << bitshift;
}
function clear_bit(bitshift) {
   command &= ~(1 << bitshift);
}
function check_bit(bitshift) {
   return (command >> bitshift) & 1;
}

function set_parity() {
   var command_copy = command;
   var count = 0;

   for(var i = 1; i < command_size; i++) {
      if((command_copy >> i) & 0x01)
         count++;
   }
   
   if(count % 2)
      set_bit(parity_shift);
   else
      clear_bit(parity_shift);
}

// for debugging 
function print_command() {
   console.log(command.toString(2));
}

// export public functions
//module.exports.serial_open = serial_open;
module.exports.set_light = set_light;
module.exports.set_speed = set_speed;
module.exports.set_motor = set_motor;
module.exports.clear_motor = clear_motor;
module.exports.send_command = send_command;
module.exports.clear_command = clear_command;

// for testing purposes, delete this
module.exports.set_parity = set_parity;
module.exports.print_command = print_command;
