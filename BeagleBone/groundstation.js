/* Ground Station Server for User Commands
 *
 * Created by: Joe Mahoney, Tyler Mau, Angela Yoeurng, Matt Rounds
 * Last Updated: April 4, 2016
*/

/* Load Modules */
var http = require('http');
var fs = require('fs');
var path = require('path');
var b = require('bonescript');

/* Constants for Bit Masking */
const parity_shift = 0;
const motor_shift = 1;
const speed_size = 3;
const speed_shift = 11;
const light_shift = 14;
const command_size = 32;

/* Motor Command Constants */
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

/* Command Bit Vector (as Integer) */
var command = 0;
var old_command = 0;

/* Define Serial Port and Options */
var port = '/dev/ttyO2';
var options = {
   baudrate: 115200
};

/* Initialize Server on Port 8888 */
var server = http.createServer(function (req, res) {
   /* Request HTML Files */
   var file = '/var/lib/cloud9/autorun' + ((req.url == '/') ? '/index.html' : req.url);
   var fileExtension = path.extname(file);
   var contentType = 'text/html';
    
   /* Allows Custom CSS */
   if (fileExtension == '.css') {
      contentType = 'text/css';
   }
    
   /* Error Checking */
   fs.exists(file, function(exists) {
      if (exists) {
         fs.readFile(file, function(error, content) {
            if (!error) {
               /* Page Found (Write Content) */
               res.writeHead(200, {'content-type':contentType});
               res.end(content);
            }
         })
      }
      else{
         /* Page Not Found */
         res.writeHead(404);
         res.end('Page not found');
      }
   })
}).listen(8888);

/* Load Socket IO Module */
var io = require('socket.io').listen(server);

/* On Established Connection */
io.on('connection', function (socket) {
   socket.on('updateKeyVector', handleMotorChange);
   socket.on('handleSlider', handleMotorSpeed);
   socket.on('changeLEDs', handleLEDs);
});

/* Indicate Server Running */
server.listen(console.log("Server Running..."));

/* Open Serial Port */
b.serialOpen(port, options, onSerial);

/* Handle Serial Activity */
function onSerial(x) {
   /* Error Check */
   if (x.err) {
      console.log('Error Opening Port: ' + JSON.stringify(x));
   }
   
   /* Check Serial Opening */
   if (x.event == 'open') {
      console.log('Serial Opened Successfully!');
   }
   
   /* Serial Data */
   if (x.event == 'data') {
      console.log(String(x.data));
   }
}

/* ---------------- Command Handling Functions ---------------- */

/* Handles Motor Commands */
function handleMotorChange(data) {
   var newData = JSON.parse(data);
   //console.log(bin(newData.state));
   clear_motor();

   command |= newData.state;
   //console.log('1: ' + command);
   //console.log(bin(command));

   /* Send Newly Updated Command */
   if (old_command != command) {
      send_command();
   }
   old_command = command;
} 

/* Handles Lights Setting */
function handleLEDs(data) {
   var toggle = JSON.parse(data);
    
   if (toggle.state == '0') {
      set_light('off');
      console.log("Lights: OFF");
   }
   else if (toggle.state == '1') {
      set_light('on');
      console.log("Lights: ON");
   }
   
   /* Send Newly Updated Command */
   send_command()
}

/* Handles Motor Speed Setting */
function handleMotorSpeed(data) {
   var newData = JSON.parse(data);
    
   set_speed(newData.state);
   console.log("Speed: " + newData.state);
   
   /* Send Newly Updated Command */
   send_command();
}

/* --------------------- Helper Functions --------------------- */

/* Sets Light Bit */
function set_light(setting) {
   switch (setting) {
      case 'on':
         set_bit(light_shift);
         break;
      case 'off':
         clear_bit(light_shift);
         break;
      default:
         throw new Error("Invalid light setting: use 'on' or 'off'");
   }
}

/* Sets Speed Bits */
function set_speed(speed) {
   /* Error Checking */
   if (speed < 0 || speed > 7) {
      throw new Error('Speed setting out of range: 0-7');
   }
   
   clear_speed();
   command |= speed << speed_shift;
}

/* Clears Speed Bits */
function clear_speed() {
   for (var i = 0; i < speed_size; i++) {
      command &= ~(1 << (i + speed_shift));
   }
}

/* Clears Motor Directions */
function clear_motor() {
   for (var i = 0; i < motor_size; i++) {
      command &= ~(1 << (i + motor_shift));
   }
}

/* Sends Command Over Serial */
function send_command() { 
   var command1 = 0, command2 = 0, command3 = 0, command4 = 0;
   set_parity();

   /* Splits Command Vector into Four Separate Bytes */
   command1 |= (command >> 24) & 0xFF;
   command2 |= (command >> 16) & 0xFF;
   command3 |= (command >> 8) & 0xFF;
   command4 |= (command) & 0xFF;

   /* Sends Command Bytes Over Serial */
   b.serialWrite(port, [command1]);
   b.serialWrite(port, [command2]);
   b.serialWrite(port, [command3]);
   b.serialWrite(port, [command4]);
}

/* Clears Command Vector */
function clear_command() {
   command &= 0;
}

/* Sets Given Bit in Command Vector */
function set_bit(bitshift) {
   command |= 1 << bitshift;
}

/* Clears Given Bit in Command Vector */
function clear_bit(bitshift) {
   command &= ~(1 << bitshift);
}

/* Checks if Given Bit in Command Vector is Set */
function check_bit(bitshift) {
   return (command >> bitshift) & 1;
}

/* Sets Parity Bit */
function set_parity() {
   var command_copy = command;
   var count = 0;

   for (var i = 1; i < command_size; i++) {
      if ((command_copy >> i) & 0x01) {
         count++;
      }
   }
   
   if (count % 2) {
      set_bit(parity_shift);
   }
   else {
      clear_bit(parity_shift);
   }
}

/* Converts Decimal to Binary */
function bin(dec) {
   return (dec >>> 0).toString(2);
}

/* Prints Command in Binary for Debugging */
function print_command() {
   console.log(command.toString(2));
}