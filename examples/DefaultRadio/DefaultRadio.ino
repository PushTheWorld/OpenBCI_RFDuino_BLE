/*
* Sets up RFduino Device for OpenBCI_32bit using RFduinoGZLL library
*
* This test behaves as a serial pass thru between two RFduinos,
* To Program, user must have RFduino core files installed in Arduino 1.5.8 or later
* Use the RFRST, RFRX, RFTX, and GND on the board to connect to USB<>Serial device
* Your USB<>Serial device must connect RFRST with DTR through 0.1uF capacitor (sorry)
*
* Written by Push The World LLC 2016 inspired by Joel Murphy, Leif Percifield
*  and Conor Russomanno. You should have recieved a copy of the license when
*  you downloaded from github. Free to use and share. This code presented for
*  use as-is.
*/
// #define DEBUG 1
#include <RFduinoBLE.h>
#include "OpenBCI_RFDuino_BLE.h"
uint8_t buffer[BYTES_PER_BLE_PACKET];
uint8_t bufferPos = 0;

void serialEvent(void){
  // Get one char and process it
  // Store it to serial buffer
  // tinyBuffer[tinyBufHead++] = Serial.read();
  // if (tinyBufHead >= BYTES_PER_TINY_BUF) tinyBufHead = 0;
  //
  // radioBLE.lastTimeSerialRead = micros();

  radioBLE.bufferBLEAddChar(radioBLE.bufferBLE + radioBLE.head, Serial.read());
  // radioBLE.bufferBLEAddChar(radioBLE.bufferBLE, Serial.read());
  // radioBLE.lastTimeSerialRead = micros();
}

void setup() {
  // Declare the secreteKey
  //  set the first time the board powers up OR after a flash of the non-
  //  volatile memory space with a call to `flashNonVolatileMemory`.
  radioBLE.begin(123456);

  RFduinoBLE.advertisementData = "OBCI";
  // Serial.println("Waiting for connection...");
  RFduinoBLE.begin();

}

void loop() {

  // if ((radioBLE.spBuffer.state == radioBLE.STREAM_STATE_READY || radioBLE.bufferBLEHeadReadyToMove()) && radioBLE.bufferStreamTimeout()) {
  //   radioBLE.bufferSerialReset(OPENBCI_NUMBER_SERIAL_BUFFERS);
  // }
//
//   if (Serial.available()) {
//     // char newChar = Serial.read();
//
// //    Serial.println(tinyBuffer[tinyBufTail]);
//     // Serial.print("h: "); Serial.print(tinyBufHead); Serial.print(" t: "); Serial.println(tinyBufTail);
//     // if (radioBLE.bufferStreamTimeout() && radioBLE.spBuffer.state == radioBLE.STREAM_STATE_READY && radioBLE.bufferSerialHasData()) {
//     //   radioBLE.bufferSerialReset(OPENBCI_NUMBER_SERIAL_BUFFERS);
//     // }
//     // radioBLE.bufferSerialAddChar(newChar);
//     radioBLE.bufferBLEAddChar(radioBLE.bufferBLE, Serial.read());
//
//     if (radioBLE.bufferBLE->state == radioBLE.STREAM_STATE_READY) {
//       // RFduinoBLE.send((const char *)radioBLE.bufferBLE->data, BYTES_PER_BLE_PACKET);
//       while (! RFduinoBLE.send((const char *)radioBLE.bufferBLE->data, BYTES_PER_BLE_PACKET))
//         ;
//       radioBLE.bufferBLEReset(radioBLE.bufferBLE);
//     }
//
//     // radioBLE.bufferBLEAddChar(radioBLE.bufferBLE, Serial.read());
//     // radioBLE.lastTimeSerialRead = micros();
//
//     // if (tinyBufTail >= BYTES_PER_TINY_BUF) tinyBufTail = 0;
//
//   }

  // First we must ask if an emergency stop flag has been triggered, as a Device
  //  we must frequently ask this question as we are the only one that can
  //  initiaite a communication between back to the Driver.
  // if (radioBLE.bufferSerial.overflowed) {
  //   // Clear the buffer holding all serial data.
  //   radioBLE.bufferSerialReset(OPENBCI_NUMBER_SERIAL_BUFFERS);
  //
  //   // Reset the stream buffer
  //   radioBLE.bufferBLEReset();
  //
  //   // Send reset message to the board
  //   radioBLE.resetPic32();
  //
  //   // Send emergency message to the host
  //   // radioBLE.singleCharMsg[0] = (char)ORPM_DEVICE_SERIAL_OVERFLOW;
  //
  //   radioBLE.bufferSerial.overflowed = false;
  //   // if (RFduinoGZLL.sendToHost(radio.singleCharMsg,1)) {
  //   //   radio.bufferSerial.overflowed = false;
  //   // }
  // } else {
  //   // if (radioBLE.bufferBLE->state == radioBLE.STREAM_STATE_READY) {
  //     // while (! RFduinoBLE.send((const char *)radioBLE.bufferBLE->data, BYTES_PER_BLE_PACKET));  // all tx buffers in use (can't send - try again later)
  //     // radioBLE.bufferBLE->state = radioBLE.STREAM_STATE_INIT;
  //   // }
    if (radioBLE.bufferBLEHeadReadyToMove()) { // Is there a stream packet waiting to get sent to the Host?
      radioBLE.bufferBLEHeadMove();
      // Serial.println("moved head");
    }

    if (radioBLE.bufferBLETailReadyToSend()) { // Is there a stream packet waiting to get sent to the Host?
      // radioBLE.bufferSerialReset(OPENBCI_NUMBER_SERIAL_BUFFERS);
      radioBLE.bufferBLETailSend();
    }
  //
  //   // if (radioBLE.bufferSerialHasData()) { // Is there data from the Pic waiting to get sent to connected device?
  //   //   if (radioBLE.bufferStreamTimeout() && (radioBLE.bufferBLE + radioBLE.head)->bytesIn > 7  && radioBLE.bufferSerialHasData()) {
  //   //     radioBLE.bufferSerialReset(OPENBCI_NUMBER_SERIAL_BUFFERS);
  //   //     return;
  //   //   }
  //   //   // Has 3ms passed since the last time the serial port was read. Only the
  //   //   //  first packet get's sent from here
  //   //   if (radioBLE.bufferSerialTimeout() && radioBLE.bufferSerial.numberOfPacketsSent == 0 ) {
  //   //     // In order to do checksumming we must only send one packet at a time
  //   //     //  this stands as the first time we are going to send a packet!
  //   //     radioBLE.sendPacketToConnectedDevice();
  //   //   }
  //   // }
  // }

  // if (radioBLE.bufferBLE->state == radioBLE.STREAM_STATE_READY) {
  //   radioBLE.bufferBLE->state = radioBLE.STREAM_STATE_INIT;
  //   RFduinoBLE.send((const char *)radioBLE.bufferBLE->data, BYTES_PER_BLE_PACKET);  // all tx buffers in use (can't send - try again later)
  // }

  if (bufferPos > 0) {
    uint8_t tempBytesToSend = bufferPos;
    // if (tempBytesToSend == 1) {
    //   switch (buffer[0]) {
    //     case OPENBCI_HOST_CMD_BAUD_DEFAULT: // 0x05
    //       radioBLE.beginSerial(OPENBCI_BAUD_RATE_DEFAULT);
    //       bufferPos = 0;
    //       return;
    //    case OPENBCI_HOST_CMD_BAUD_BLE: // 0x0B
    //       radioBLE.beginSerial(OPENBCI_BAUD_RATE_BLE);
    //       bufferPos = 0;
    //       return;
    //   }
    // }
    for (int i = 0; i < tempBytesToSend; i++) {
      Serial.write(buffer[i]);
    }
    bufferPos = 0;
  }
}

/**
 * Fired when a device connects
 * @type {Boolean}
 */
void RFduinoBLE_onConnect() {
  radioBLE.connectedDevice = true;
  // Serial.println("Connected");
  // first send is not possible until the iPhone completes service/characteristic discovery
}

/**
 * Triggered when a BLE device disconnects
 * @type {Boolean}
 */
void RFduinoBLE_onDisconnect() {
  radioBLE.connectedDevice = false;
  // Serial.println("Disconnected");
  // first send is not possible until the iPhone completes service/characteristic discovery
}

/**
* @description A packet with 1 byte is a private radio message, a packet with
*                  more than 1 byte is a standard packet with a checksum. and
*                  a packet with no length is a NULL packet that indicates a
*                  successful message transmission
* @param device {device_t} - The host in this case
* @param data {char *} - The packet of data sent in the packet
* @param len {int} - The length of the `data` packet
*/
void RFduinoBLE_onReceive(char *data, int len) {
  for (int i = 0; i < len; i++) {
    if (bufferPos >= BYTES_PER_BLE_PACKET) {
      return;
    }
    buffer[bufferPos++] = data[i];
  }
}
