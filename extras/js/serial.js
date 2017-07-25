var SerialPort = require('serialport');
var port = new SerialPort('/dev/tty.usbserial-DQ008436', {
  baudrate: 115200,
  autoOpen: false
});

// Switches the port into "flowing mode"
port.on('data', (data) => {
  console.log(data.toString('utf8'));
});

let count = 0;
let portOpen = false;
setInterval(() => {
  if (portOpen) {
    port.write(`pushtheworldajkeller`, function(err) {
      if (err) {
        return console.log('Error on write: ', err.message);
      }
      if (count % 20 == 0) console.log(count, 'messages written');
      count++;
    });
  }
}, 12);

// If you write before the port is opened the write will be queued
// Since there is no callback any write errors will be emitted on an error event

// Quit on any error
port.on('error', (err) => {
  console.log(err.message);
  process.exit(1);
});

port.open((err) => {
  if (err) {
    return console.log('Error opening port: ', err.message);
  }
  portOpen = true;

  console.log("port open");
  port.write('pushtheworldajkeller');

});
