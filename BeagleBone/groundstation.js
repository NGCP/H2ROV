//Loading modules
var http = require('http');
var fs = require('fs');
var path = require('path');
var b = require('bonescript');
var cmd = require('./command')


var port = '/dev/ttyO2';
var options = {
    baudrate: 9600
};

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

// Create a variable called key, which refers to P9_14
var led = "P9_14";

// Initialize the led as an OUTPUT
b.pinMode(led, b.OUTPUT);

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
    console.log("MOTOR = " + newData.state);
    
    // turns the motor forward (1), backward (2),  
	if (newData.state == '1') {
        //b.serialWrite(port, [0x01]);
        cmd.set_motor('forward');
    }
    else if (newData.state == '2') {
        //b.serialWrite(port, [0x02]);
        cmd.set_motor('backward');
    }
    
    if (newData.state == '3') {
        cmd.set_motor('right');   
    }
    else if (newData.state == '4') {
        cmd.set_motor('left');
    }
    
    if (newData.state == '5') {
        cmd.set_motor('up');
    }
    else if (newData.state == '6') {
        cmd.set_motor('down');
    }
    
    
    cmd.send_command();  
} 

function handleLEDs(data) {
    var toggle = JSON.parse(data);
    
    if (toggle.state == '0') {
        console.log("LED = " + 0);
        cmd.set_light('on');
    }
    else if (toggle.state == '1') {
        console.log("LED = " + 1);
        cmd.set_light('off');
    }
    
    cmd.send_command()
    
}


function handleMotorSpeed(data) {
    var newData = JSON.parse(data);
    console.log("Speed = " + newData.state);
    
    //update bit vector and send to arduino of serial 1 
    cmd.set_speed(newData.state);
    cmd.send_command();
}


// Displaying a console message for user feedback
server.listen(console.log("Server Running ..."));
