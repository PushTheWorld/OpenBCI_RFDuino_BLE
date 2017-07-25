var SerialPort = require('serialport');
var port = new SerialPort('/dev/tty.usbserial-DQ008436', {
  baudrate: 115200
});

// Switches the port into "flowing mode"
port.on('data', function (data) {
  console.log('Data: ', data.toString());
});

let count = 0;
setInterval(() => {
  port.write('pushtheworldajkeller', function(err) {
    if (err) {
      return console.log('Error on write: ', err.message);
    }
    if (count % 20 == 0) console.log('message written');
  });
}, 100);

// Open errors will be emitted as an error event
port.on('error', function(err) {
  console.log('Error: ', err.message);
})
