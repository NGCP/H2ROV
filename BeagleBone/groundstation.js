//Loading modules
var http = require('http');
var fs = require('fs');
var path = require('path');
var b = require('bonescript');

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
    socket.on('changeState', handleChangeState);
    socket.on('handleRadioButtons', handleMotorSpeed);
});

// Change led state when a button is pressed
function handleChangeState(data) {
    var newData = JSON.parse(data);
    console.log("MOTOR = " + newData.state);
    
    // turns the motor forward (1) reverse (2) 
	if (newData.state == '1') {
        b.serialWrite(port, [0x01]);
    }
    else if (newData.state == '2') {
        b.serialWrite(port, [0x02]);
    }
    
    //led on/off 
    if (newData.state == '3') {
        console.log("LED = " + 1);
        b.digitalWrite(led, 1);
    }
    else if (newData.state == '4') {
        console.log("LED = " + 0);
        b.digitalWrite(led, 0);
    }
}

function handleMotorSpeed(data) {
    var newData = JSON.parse(data);
    console.log("Speed = " + newData.state);
    
    
}

// Displaying a console message for user feedback
server.listen(console.log("Server Running ..."));
