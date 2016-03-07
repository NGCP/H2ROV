//Loading modules
var http = require('http');
var fs = require('fs');
var path = require('path');
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


var port = '/dev/ttyO2';
var options = {
    baudrate: 9600
};

// Create a variable called key, which refers to P9_14
//var led = "P9_14";

// Initialize the led as an OUTPUT
//b.pinMode(led, b.OUTPUT);

// Initialize the server on port 8888
var server = http.createServer(function (req, res) {
    // requesting files
    var file = '.'+((req.url=='/')?'/index.html':req.url);
    var fileExtension = path.extname(file);
    var contentType = 'text/html';
    // Uncoment if you want to add css to your web page
    /*
    if(fileExtension == '.css'){
        contentType = 'text/css';
    }*/
    fs.exists(file, function(exists){
        if(exists){
            fs.readFile(file, function(error, content){
                if(!error){
                    // Page found, write content
                    res.writeHead(200,{'content-type':contentType});
                    res.end(content);
                }
            })
        }
        else{
            // Page not found
            res.writeHead(404);
            res.end('Page not found');
        }
    })
}).listen(8888);

// Loading socket io module
var io = require('socket.io').listen(server);

// When communication is established
io.on('connection', function (socket) {
    socket.on('changeMotorDirection', handleMotorChange);
    socket.on('handleRadioButtons', handleMotorSpeed);
    socket.on('changeLEDs', handleLEDs);
});


// Change led state when a button is pressed
function handleMotorChange(data) {
    var newData = JSON.parse(data);
    //console.log("MOTOR = " + newData.state);
    
    // turns the motor forward (1), backward (2),  
	  if (newData.state == '1') {
        //b.serialWrite(port, [0x01]);
        //cmd.set_motor('forward');
        set_motor('forward');
        console.log("MOTOR = FORWARD");
    }
    else if (newData.state == '2') {
        //b.serialWrite(port, [0x02]);
        //cmd.set_motor('backward');
        set_motor('backward');
        console.log("MOTOR = BACKWARD");
    }
    else if (newData.state == '0') {
        clear_motor();
        console.log("MOTOR = IDLE");
    }
    
    if (newData.state == '3') {
        set_motor('yaw left');  
        console.log("MOTOR = YAW LEFT");
    }
    else if (newData.state == '4') {
        set_motor('yaw right');
        console.log("MOTOR = YAW RIGHT");
    }
    
    if (newData.state == '5') { //ascend        
        set_motor('up');
        console.log("MOTOR = ASCEND/UP");
    }
    else if (newData.state == '6') { //submerge 
        set_motor('down');
        console.log("MOTOR = SUBMERGE/DOWN");
    } 
    
    if (newData.state == '7') {       
        set_motor('pitch up');
        console.log("MOTOR = PITCH UP");
    }
    else if (newData.state == '8') { 
        set_motor('pitch down');
        console.log("MOTOR = PITCH DOWN");
    }
    
    if (newData.state == '9') {       
        set_motor('left');
        console.log("MOTOR = STRAFE LEFT/LEFT");
    }
    else if (newData.state == '10') { 
        set_motor('right');
        console.log("MOTOR = STRAFE RIGHT/RIGHT");
    }
    
    
    send_command();  
} 

function handleLEDs(data) {
    var toggle = JSON.parse(data);
    
    if (toggle.state == '0') {
        console.log("LED = " + 0);
        set_light('on');
    }
    else if (toggle.state == '1') {
        console.log("LED = " + 1);
        set_light('off');
    }
    
    send_command()
    
}


function handleMotorSpeed(data) {
    var newData = JSON.parse(data);
    console.log("Speed = " + newData.state);
    
    //update bit vector and send to arduino of serial 1 
    set_speed(newData.state);
    send_command();
}


// Displaying a console message for user feedback
server.listen(console.log("Server Running ..."));


b.serialOpen(port, options, onSerial);

function onSerial(x) {
    if (x.err) {
        console.log('***ERROR*** ' + JSON.stringify(x));
    }
    if (x.event == 'open') {
       console.log('***OPENED***');
    }
    if (x.event == 'data') {
        console.log(String(x.data));
    }
}

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
   b.serialWrite(port, [command]);
   clear_motor();
   console.log("SENT COMMAND BITCHES " + command);
   //b.serialWrite(port, 1);
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