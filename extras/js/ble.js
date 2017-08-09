const noble = require('noble');

const rfduinoServiceUuid = '2220';
const rfduinoSubscribeCharacteristicUuid = '2221';
const writeOneCharacteristicUuid = '2222';
const writeTwoCharacteristicUuid = '2223';

noble.on('stateChange', function(state) {
  if (state === 'poweredOn') {
    //
    // Once the BLE radio has been powered on, it is possible
    // to begin scanning for services. Pass an empty array to
    // scan for all services (uses more time and power).
    //
    console.log('scanning...');
    noble.startScanning([], false);
  }
  else {
    noble.stopScanning();
  }
})

let rfduinoService = null;
let rfduinoSubscribeCharacteristic = null;
let writeOneCharacteristic = null;
let writeTwoCharacteristic = null;

let connected = false;

const processBytes = (data) => {
  console.log('data', data);
};

noble.on('discover', function(peripheral) {
  // we found a peripheral, stop scanning
  noble.stopScanning();

  //
  // The advertisment data contains a name, power level (if available),
  // certain advertised service uuids, as well as manufacturer data,
  // which could be formatted as an iBeacon.
  //
  console.log('found peripheral:', peripheral.advertisement);
  //
  // Once the peripheral has been discovered, then connect to it.
  //
  peripheral.connect(function(err) {
    //
    // Once the peripheral has been connected, then discover the
    // services and characteristics of interest.
    //
    peripheral.discoverServices([rfduinoServiceUuid], function(err, services) {
      services.forEach(function(service) {
        //
        // This must be the service we were looking for.
        //
        console.log('found service:', service.uuid);

        //
        // So, discover its characteristics.
        //
        service.discoverCharacteristics([], function(err, characteristics) {

          characteristics.forEach(function(characteristic) {
            //
            // Loop through each characteristic and match them to the
            // UUIDs that we know about.
            //
            console.log('found characteristic:', characteristic.uuid);

            if (rfduinoSubscribeCharacteristicUuid == characteristic.uuid) {
              rfduinoSubscribeCharacteristic = characteristic;
            }
            else if (writeOneCharacteristicUuid == characteristic.uuid) {
              writeOneCharacteristic = characteristic;
            }
            else if (writeTwoCharacteristicUuid == characteristic.uuid) {
              writeTwoCharacteristic = characteristic;
            }
          })

          //
          // Check to see if we found all of our characteristics.
          //
          if (rfduinoSubscribeCharacteristic &&
              writeOneCharacteristic &&
              writeTwoCharacteristic) {
            //
            // We did, so bake a pizza!
            //
            rfduinoSubscribeCharacteristic.on('read', (data) => {
              // TODO: handle all the data, both streaming and not
              console.log(`${data[0]} head: ${data[1]} tail: ${data[2]}`);
              // processBytes(data);
            });
            rfduinoSubscribeCharacteristic.notify(true);

            connected = true;

            var crust = new Buffer("b");
            writeTwoCharacteristic.write(crust, false, function(err) {
              if (err) console.error(err);
              else console.log('wrote writeTwoCharacteristic');
            });
            writeOneCharacteristic.write(crust, false, function(err) {
              if (err) console.error(err);
              else console.log('wrote writeOneCharacteristic');
            });

          }
          else {
            console.log('missing characteristics');
          }
        })
      })
    })
  })
})
