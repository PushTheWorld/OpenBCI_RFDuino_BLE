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
#define DEBUG 1
#include <RFduinoBLE.h>
#include "OpenBCI_Radios.h"

void serialEvent(void){
  // Get one char and process it
  // Mark the last serial as now;
  radioBLE.bufferStreamAddChar(blePackets + head, Serial.read());
  radioBLE.lastTimeSerialRead = micros();
}

void setup() {
  // Declare the radio mode and channel number. Note this channel is only
  //  set the first time the board powers up OR after a flash of the non-
  //  volatile memory space with a call to `flashNonVolatileMemory`.
  // MAKE SURE THIS CHANNEL NUMBER MATCHES THE HOST!
  radioBLE.begin();
}

void loop() {

  // First we must ask if an emergency stop flag has been triggered, as a Device
  //  we must frequently ask this question as we are the only one that can
  //  initiaite a communication between back to the Driver.
  if (radio.bufferSerial.overflowed) {
    // Clear the buffer holding all serial data.
    radio.bufferSerialReset(OPENBCI_NUMBER_SERIAL_BUFFERS);

    // Reset the stream buffer
    radio.bufferStreamReset();

    // Send reset message to the board
    radio.resetPic32();

    // Reset the last time we contacted the host to now
    radio.pollRefresh();

    // Send emergency message to the host
    radio.singleCharMsg[0] = (char)ORPM_DEVICE_SERIAL_OVERFLOW;

    if (RFduinoGZLL.sendToHost(radio.singleCharMsg,1)) {
      radio.bufferSerial.overflowed = false;
    }
  } else {
    if (Serial.available()) { // Is there new serial data available?
      char newChar = Serial.read();
      // Mark the last serial as now;
      radio.lastTimeSerialRead = micros();
      // Store it to serial buffer
      radio.bufferSerialAddChar(newChar);
      // Get one char and process it
      radio.bufferStreamAddChar((radio.streamPacketBuffer + radio.streamPacketBufferHead), newChar);
      // Reset the poll timer to prevent contacting the host mid read
      radio.pollRefresh();
    }

    if ((radioBLE.blePackets + radioBLE.head)->state == radioBLE.BLE_PACKET_STATE_READY) { // Is there a stream packet waiting to get sent to the Host?
      // Load the packet into the BLESendPacketBuffer
      if (radioBLE.bufferStreamTimeout()) {
        // We are sure this is a streaming packet.
        radioBLE.head++;
        if (radioBLE.head > (NUM_INPUT_BUFFERS - 1)) {
          radioBLE.head = 0;
        }
      }
    }

    if ((radioBLE.blePackets + radioBLE.tail)->state == radio.STREAM_STATE_READY) { // Is there a stream packet waiting to get sent to the Host?
      if (radioBLE.head != radioBLE.tail) {
        while (! RFduinoBLE.send((const char *)(radioBLE.blePackets + radioBLE.tail)->data, BYTES_PER_BLE_PACKET))
          ;  // all tx buffers in use (can't send - try again later)
        // Try to add the tail to the TX buffer
        radioBLE.tail++;
        if (radioBLE.tail > (NUM_INPUT_BUFFERS - 1)) {
          radioBLE.tail = 0;
        }
      }
    }

    if (radio.bufferSerialHasData()) { // Is there data from the Pic waiting to get sent to Host
      // Has 3ms passed since the last time the serial port was read. Only the
      //  first packet get's sent from here
      if (radio.bufferSerialTimeout() && radio.bufferSerial.numberOfPacketsSent == 0 ) {
        // In order to do checksumming we must only send one packet at a time
        //  this stands as the first time we are going to send a packet!
        radio.sendPacketToHost();
      }
    }

    radio.bufferRadioFlushBuffers();

    if (millis() > (radio.timeOfLastPoll + radio.pollTime)) {  // Has more than the poll time passed?
      // Refresh the poll timer
      radio.pollRefresh();
      // Poll the host
      radio.sendPollMessageToHost();
    }
  }
}

/**
 * Fired when a device connects
 * @type {Boolean}
 */
void RFduinoBLE_onConnect() {
  connectedDevice = true;
#ifdef DEBUG
  Serial.println("Connected");
#endif
  // first send is not possible until the iPhone completes service/characteristic discovery
}

/**
 * Triggered when a BLE device disconnects
 * @type {Boolean}
 */
void RFduinoBLE_onDisconnect() {
  connectedDevice = false;
#ifdef DEBUG
  Serial.println("Disconnected");
#endif
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
  for (uint8_t i = 0; i < len; i++) {
    Serial.write((uint8_t)data[i]);
  }
}
