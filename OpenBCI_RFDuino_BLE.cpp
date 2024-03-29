/***************************************************
This is a library for the OpenBCI 32bit RFduinoBLE

OpenBCI invests time and resources providing this open source code,
please support OpenBCI and open-source hardware by purchasing
products from OpenBCI or donating on our downloads page!

Written by AJ Keller of Push The World LLC

MIT license
****************************************************/

#include "OpenBCI_RFDuino_BLE.h"

// CONSTRUCTOR
OpenBCI_RFDuino_BLE_Class::OpenBCI_RFDuino_BLE_Class() {
  // Set defaults
  secreteKey = 12345; // Random number
  debugMode = false; // Set true if doing dongle-dongle sim
  lastTimeSerialRead = 0;
  connectedDevice = false;
}

/**
* @description The function that the radio will call in setup()
* @author AJ Keller (@aj-ptw)
*/
void OpenBCI_RFDuino_BLE_Class::begin() {
  // configure radio
  configure(secreteKey);
}

/**
* @description The function that the radio will call in setup()
* @param: mode {unint8_t} - The mode the radio shall operate in
* @param: _secreteKey {uint32_t} - The secreteKey the RFduinoBLE will
*           use to communicate with the driver.
* @author AJ Keller (@aj-ptw)
*/
void OpenBCI_RFDuino_BLE_Class::begin(uint32_t _secreteKey) {
  // configure radio
  configure(_secreteKey);
}

/**
* @description Puts into debug mode and then call other function
* @author AJ Keller (@aj-ptw)
*/
void OpenBCI_RFDuino_BLE_Class::beginDebug() {
  beginDebug(secreteKey);
}

/**
* @description Puts into debug mode and then call other function
* @param: _secreteKey {uint32_t} - The _secreteKey the RFduinoBLE will use.
* @author AJ Keller (@aj-ptw)
*/
void OpenBCI_RFDuino_BLE_Class::beginDebug(uint32_t _secreteKey) {
  debugMode = true;
  begin(_secreteKey);
}

/**
* @description Private function to initialize the OpenBCI_RFDuino_BLE_Class object
* @param: secreteKey {uint32_t} - The channelNumber the RFduinoGZLL will
*           use to communicate with the other RFduinoGZLL
* @author AJ Keller (@aj-ptw)
*/
void OpenBCI_RFDuino_BLE_Class::configure(uint32_t _secreteKey) {
  // Check to see if we need to set the secreteKey number
  //  this is only the case on the first run of the program
  if (needToSetSecreteKey()) {
    setSecreteKey(_secreteKey);
  }
  secreteKey = getSecreteKey();

  head = 0;
  tail = 0;

  bufferBLEReset();
  bufferSerialReset(OPENBCI_NUMBER_SERIAL_BUFFERS);
  bufferStreamReset();
  configureDevice(); // setup for Device
}

/**
* @description Private function to initialize the radio in Device mode
* @author AJ Keller (@aj-ptw)
*/
void OpenBCI_RFDuino_BLE_Class::configureDevice(void) {
  // override_uart_limit = true;
  // Configure pins
  if (debugMode) { // Dongle debug mode
    // BEGIN: To run host as device
    pinMode(OPENBCI_PIN_HOST_RESET,INPUT);
    pinMode(OPENBCI_PIN_HOST_LED,OUTPUT);
    digitalWrite(OPENBCI_PIN_HOST_LED,HIGH);
    // END: To run host as device
  } else {
    // BEGIN: To run host normally
    pinMode(OPENBCI_PIN_DEVICE_PCG, INPUT); //feel the state of the PIC with this pin
    // END: To run host normally
  }
  beginSerial();
}

/**
 * Used to start the serial port with 115200 baud
 */
void OpenBCI_RFDuino_BLE_Class::beginSerial(void) {
  beginSerial(OPENBCI_BAUD_RATE_BLE);
}

/**
 * Used to start the serial port with 115200 baud
 * @param baudRate {uint32_t} - The baud rate to set to.
 */
void OpenBCI_RFDuino_BLE_Class::beginSerial(uint32_t baudRate) {
  if (Serial) Serial.end();

  if (debugMode) {
    Serial.begin(baudRate);
  } else {
    // Start the serial connection. On the device we must specify which pins are
    //    rx and tx, where:
    //      rx = GPIO3
    //      tx = GPIO2
    Serial.begin(baudRate, 3, 2);
  }
}


/**
* @description Gets the channel number from non-volatile flash memory
* @returns {uint32_t} - The channel number from non-volatile memory
* @author AJ Keller (@aj-ptw)
*/
uint32_t OpenBCI_RFDuino_BLE_Class::getSecreteKey(void) {
  return *ADDRESS_OF_PAGE(RFDUINOGZLL_FLASH_MEM_ADDR);
}

/**
* @description Reads from memory to see if the channel number needs to be set
* @return {boolean} True if the channel number needs to be set
* @author AJ Keller (@aj-ptw)
*/
boolean OpenBCI_RFDuino_BLE_Class::needToSetSecreteKey(void) {
  return getSecreteKey() == 0xFFFFFFFF;
}

/**
* @description Store a channel number to memory. Allows for the channel to be
*      maintained even after power cycle.
* @param channelNumber {uint32_t} - The new channel to set to. Must be less
*      than 25.
* @return {boolean} - If the channel was successfully flashed to memory. False
*      when the channel number is out of bounds.
* @author AJ Keller (@aj-ptw)
*/
boolean OpenBCI_RFDuino_BLE_Class::setSecreteKey(uint32_t _secreteKey) {
  // Secrete Key
  uint32_t *p = ADDRESS_OF_PAGE(RFDUINOGZLL_FLASH_MEM_ADDR);
  int rc;
  if (flashNonVolatileMemory()) {
    rc = flashWrite(p, _secreteKey);
    if (rc == 0) {
      return true;
    } else if (rc == 1) {
      return false;
    } else if (rc == 2) {
      return false;
    }
  }
  return false;
}

/**
* @description Used to reset the non-volatile memory back to it's factory state so
*  the parameters in `begin()` will be accepted.
* @return {boolean} - `true` if the memory was successfully reset, `false` if not...
* @author AJ Keller (@aj-ptw)
*/
boolean OpenBCI_RFDuino_BLE_Class::flashNonVolatileMemory(void) {

  uint32_t *p = ADDRESS_OF_PAGE(RFDUINOGZLL_FLASH_MEM_ADDR);

  int rc = flashPageErase(PAGE_FROM_ADDRESS(p)); // erases 1k of flash
  if (rc == 1) {
    return false;
  } else if (rc == 2) {
    return false;
  }
  return true;
}

/**
* @description Called from Devices to send a packet to Host. Uses global
*  variables to send the correct packet.
* @returns {boolean} - The packet number sent.
* @author AJ Keller (@aj-ptw)
*/
boolean OpenBCI_RFDuino_BLE_Class::sendPacketToConnectedDevice(void) {

  // Reset the stream buffers
  bufferBLEReset(bufferBLE + head);

  int packetNumber = bufferSerial.numberOfPacketsToSend - bufferSerial.numberOfPacketsSent - 1;

  // Make the byteId
  // char byteId = byteIdMake(false,packetNumber,(bufferSerial.packetBuffer + bufferSerial.numberOfPacketsSent)->data + 1, (bufferSerial.packetBuffer + bufferSerial.numberOfPacketsSent)->positionWrite - 1);
//
  // Add the byteId to the packet
  // (bufferSerial.packetBuffer + bufferSerial.numberOfPacketsSent)->data[0] = byteId;

  while (! RFduinoBLE.send((const char *)(radioBLE.bufferBLE + radioBLE.tail)->data, BYTES_PER_BLE_PACKET))
    ;  // all tx buffers in use (can't send - try again later)

  bufferSerial.numberOfPacketsSent++;

  return false;
}


/**
* @description Test to see if a char follows the stream tail byte format
* @author AJ Keller (@aj-ptw)
*/
boolean OpenBCI_RFDuino_BLE_Class::isATailByte(uint8_t newChar) {
  return (newChar >> 4) == 0xC;
}

/**
* @description Sends a soft reset command to the Pic 32 incase of an emergency.
* @author AJ Keller (@aj-ptw)
*/
void OpenBCI_RFDuino_BLE_Class::resetPic32(void) {
  Serial.write('v');
}

/**
* @description Private function to clean (clear/reset) a Buffer.
* @author AJ Keller (@aj-ptw)
*/
void OpenBCI_RFDuino_BLE_Class::bufferCleanBuffer(Buffer *buffer, int numberOfPacketsToClean) {
  bufferCleanPacketBuffer(buffer->packetBuffer,numberOfPacketsToClean);
  buffer->numberOfPacketsToSend = 0;
  buffer->numberOfPacketsSent = 0;
  buffer->overflowed = false;
}

/**
* @description Private function to clean a PacketBuffer.
* @author AJ Keller (@aj-ptw)
*/
void OpenBCI_RFDuino_BLE_Class::bufferCleanPacketBuffer(PacketBuffer *packetBuffer, int numberOfPackets) {
  for(int i = 0; i < numberOfPackets; i++) {
    packetBuffer[i].positionRead = 0;
    packetBuffer[i].positionWrite = 1;
  }
}

/********************************************/
/********************************************/
/********    COMMON MEHTOD CODE    **********/
/********************************************/
/********************************************/

/**
* @description Writes a buffer to the serial port of a given length
* @param buffer [char *] The buffer you want to write out
* @param length [int] How many bytes to you want to write out?
* @author AJ Keller (@aj-ptw)
*/
void OpenBCI_RFDuino_BLE_Class::writeBufferToSerial(char *buffer, int length) {
  // Make sure we don't seg fault
  if (buffer == NULL) return;

  // Loop through all bytes in buffer
  for (int i = 0; i < length; i++) {
    Serial.write(buffer[i]);
  }
}

/**
* @description Resets the stream packet buffer to default settings
* @author AJ Keller (@aj-ptw)
*/
void OpenBCI_RFDuino_BLE_Class::bufferBLEReset(void) {
  for (int i = 0; i < NUM_BLE_PACKETS; i++) {
    bufferBLEReset(bufferBLE + i);
  }
  head = 0;
  tail = 0;
}

/**
* @description Resets the stream packet buffer to default settings
* @param `buf` {StreamPacketBuffer *} - Pointer to a stream packet buffer to reset
* @author AJ Keller (@aj-ptw)
*/
void OpenBCI_RFDuino_BLE_Class::bufferBLEReset(BLEPacket *blePacket) {
  blePacket->bytesIn = 0;
  blePacket->state = STREAM_STATE_INIT;
}

/**
 * Used to safely move the head of the ble packet buffer.
 */
void OpenBCI_RFDuino_BLE_Class::bufferBLEHeadMove(void) {
  head++;
  if (head > (NUM_BLE_PACKETS - 1)) {
    head = 0;
  }
}

/**
 * Used to determine if the head is ready to be moved
 * @return  {boolean} `true` if 3 packets have been loaded into blePacket and
 *                    enough time has passed.
 */
boolean OpenBCI_RFDuino_BLE_Class::bufferBLEHeadReadyToMove(void) {
  // TODO: I'm commenting out the wait for bufferStreamTimeout with blePackets
  //  because the state will only be ready after three stream packets have been
  //  recieved which is for sure a streaming packet condition.
  // return (bufferBLE + head)->state == STREAM_STATE_READY && bufferStreamTimeout();
  return (bufferBLE + head)->state == STREAM_STATE_READY;// && bufferStreamTimeout();
}

/**
 * Should reset current tail buffer and move tail with wrap around protection
 */
void OpenBCI_RFDuino_BLE_Class::bufferBLETailMove(void) {
  bufferBLEReset(bufferBLE + tail);
  tail++;
  if (tail > (NUM_BLE_PACKETS - 1)) {
    tail = 0;
  }
}

/**
 * Used to send the tail of the ble packet buffer to the connected device
 */
void OpenBCI_RFDuino_BLE_Class::bufferBLETailSend(void) {
  if (!connectedDevice) return;

  int lastTail = tail;

  // This will simply add to the tx buffer
  RFduinoBLE.send((const char *)(bufferBLE + lastTail)->data, BYTES_PER_BLE_PACKET);
  // This will wait for the TX buffers to clear
  // while (! RFduinoBLE.send((const char *)(bufferBLE + tail)->data, BYTES_PER_BLE_PACKET))
    // ;  // all tx buffers in use (can't send - try again later)
  bufferBLETailMove();

}

boolean OpenBCI_RFDuino_BLE_Class::bufferBLETailReadyToSend(void) {
  return head != tail && (bufferBLE+tail)->state == STREAM_STATE_READY;
}

/**
* @description Private function to clear the given buffer of length
* @author AJ Keller (@aj-ptw)
*/
void OpenBCI_RFDuino_BLE_Class::bufferCleanChar(char *buffer, int bufferLength) {
  for (int i = 0; i < bufferLength; i++) {
    buffer[i] = 0;
  }
}

/**
* @description Used to add a char data array to the the radio buffer. Always
*      skips the fist
* @param data {char *} - An array from RFduinoGZLL_onReceive
* @param len {int} - Length of array from RFduinoGZLL_onReceive
* @param clearBuffer {boolean} - If true then will reset the flags on the radio
*      buffer.
* @return {boolean} - True if the data was added to the buffer, false if the
*      buffer was overflowed.
* @author AJ Keller (@aj-ptw)
*/
boolean OpenBCI_RFDuino_BLE_Class::bufferRadioAddData(BufferRadio *buf, char *data, int len, boolean lastPacket) {
  if (lastPacket) {
    buf->gotAllPackets = true;
  }
  // Serial.print("Pos write "); Serial.println(currentRadioBuffer->positionWrite);
  for (int i = 0; i < len; i++) {
    if (buf->positionWrite < OPENBCI_BUFFER_LENGTH_MULTI) { // Check for to prevent overflow
      buf->data[buf->positionWrite] = data[i];
      buf->positionWrite++;
    } else { // We overflowed, need to return false.
      return false;
    }
  }
  return true;
}

/**
* @description Used to fill the buffer with all zeros. Should be used as
*      frequently as possible. This is very useful if you need to ensure that
*      no bad data is sent over the serial port.
* @param `buf` {BufferRadio *} - The buffer to clean.
* @author AJ Keller (@aj-ptw)
*/
void OpenBCI_RFDuino_BLE_Class::bufferRadioClean(BufferRadio *buf) {
  bufferCleanChar(buf->data,OPENBCI_BUFFER_LENGTH_MULTI);
}

/**
* @description Called when all the packets have been recieved to flush the
*       contents of the radio buffer to the serial port.
* @param `buf` {BufferRadio *} - The buffer to flush.
* @author AJ Keller (@aj-ptw)
*/
void OpenBCI_RFDuino_BLE_Class::bufferRadioFlush(BufferRadio *buf) {
  // Lock this buffer down!
  buf->flushing = true;
  if (debugMode) {
    for (int j = 0; j < buf->positionWrite; j++) {
      Serial.print(buf->data[j]);
    }
    Serial.println();
  } else {
    for (int j = 0; j < buf->positionWrite; j++) {
      Serial.write(buf->data[j]);
    }
  }
  buf->flushing = false;
}

/**
* @description Used to flush any radio buffer that is ready to be flushed to
*  the serial port.
* @author AJ Keller (@aj-ptw)
*/
void OpenBCI_RFDuino_BLE_Class::bufferRadioFlushBuffers(void) {
  for (int i = 0; i < OPENBCI_NUMBER_RADIO_BUFFERS; i++) {
    bufferRadioProcessSingle(bufferRadio + i);
  }
}

/**
* @description Used to determine if there is data in the radio buffer. Most
*  likely this data needs to be cleared.
* @param `buf` {BufferRadio *} - The buffer to examine.
* @returns {boolean} - `true` if the radio buffer has data, `false` if not...
* @author AJ Keller (@aj-ptw)
*/
boolean OpenBCI_RFDuino_BLE_Class::bufferRadioHasData(BufferRadio *buf) {
  return buf->positionWrite > 0;
}

byte OpenBCI_RFDuino_BLE_Class::bufferRadioProcessPacket(char *data, int len) {
  // The packetNumber is embedded in the first byte, the byteId
  int packetNumber = byteIdGetPacketNumber(data[0]);
  // Last packet
  if (packetNumber == 0) {
    // Current buffer has no data
    if (bufferRadioReadyForNewPage(currentRadioBuffer)) {
      // Take it! Mark Last
      bufferRadioAddData(currentRadioBuffer,data+1,len-1,true);
      // Return that this last packet was added
      return OPENBCI_PROCESS_RADIO_PASS_LAST_SINGLE;

      // Current buffer has data
    } else {
      // Current buffer has all packets or is flushing
      if (currentRadioBuffer->gotAllPackets || currentRadioBuffer->flushing) {
        // Can swtich to other buffer
        if (bufferRadioSwitchToOtherBuffer()) {
          // Take it! Mark Last
          bufferRadioAddData(currentRadioBuffer,data+1,len-1,true);
          // Return that this last packet was added
          return OPENBCI_PROCESS_RADIO_PASS_SWITCH_LAST;

          // Cannot switch to other buffer
        } else {
          // Serial.println("Last packet / Current buffer has data / Current buffer has all packets / Cannot switch to other buffer");
          // Reject it!
          return OPENBCI_PROCESS_RADIO_FAIL_SWITCH_LAST;
        }
        // Current buffer does not have all packets
      } else {
        // Previous packet number == packetNumber + 1
        if (currentRadioBuffer->previousPacketNumber - packetNumber == 1) {
          // Serial.println("Last packet / Current buffer has data / Current buffer does not have all packets / Previous packet number == packetNumber + 1");
          // Take it! Mark last.
          bufferRadioAddData(currentRadioBuffer,data+1,len-1,true);
          // Return that this last packet was added
          return OPENBCI_PROCESS_RADIO_PASS_LAST_MULTI;

          // Missed a packet
        } else {
          // Reject it!
          return OPENBCI_PROCESS_RADIO_FAIL_MISSED_LAST;
        }
      }
    }
    // Not last packet
  } else {
    // Current buffer has no data
    if (bufferRadioReadyForNewPage(currentRadioBuffer)) {
      // Serial.println("Not last packet / Current buffer has no data");
      // Take it, not last
      bufferRadioAddData(currentRadioBuffer,data+1,len-1,false);

      // Update the previous packet number
      currentRadioBuffer->previousPacketNumber = packetNumber;

      // Return that a packet that was not last was added
      return OPENBCI_PROCESS_RADIO_PASS_NOT_LAST_FIRST;

      // Current buffer has data
    } else {
      // Current buffer has all packets
      if (currentRadioBuffer->gotAllPackets) {
        // Can switch to other buffer
        if (bufferRadioSwitchToOtherBuffer()) {
          // Take it! Not last
          bufferRadioAddData(currentRadioBuffer,data+1,len-1,false);

          // Update the previous packet number
          currentRadioBuffer->previousPacketNumber = packetNumber;

          // Return that a packet that was not last was added
          return OPENBCI_PROCESS_RADIO_PASS_SWITCH_NOT_LAST;

          // Cannot switch to other buffer
        } else {
          // Reject it!
          return OPENBCI_PROCESS_RADIO_FAIL_SWITCH_NOT_LAST;
        }
        // Current buffer does not have all packets
      } else {
        // Previous packet number == packetNumber + 1
        if (currentRadioBuffer->previousPacketNumber - packetNumber == 1) {
          // Take it! Not last.
          bufferRadioAddData(currentRadioBuffer,data+1,len-1,false);

          // Update the previous packet number
          currentRadioBuffer->previousPacketNumber = packetNumber;

          // Return that a packet that was not last was added
          return OPENBCI_PROCESS_RADIO_PASS_NOT_LAST_MIDDLE;

          // Missed a packet
        } else {
          // Reject it! Reset current buffer
          return OPENBCI_PROCESS_RADIO_FAIL_MISSED_NOT_LAST;
        }
      }
    }
  }
}

/**
* @description Should only flush a buffer if it has data in it and has gotten all
*  of it's packets. This function will be called every loop so it's important to
*  make sure we don't flush a buffer unless it's really ready!
* @author AJ Keller (@aj-ptw)
*/
void OpenBCI_RFDuino_BLE_Class::bufferRadioProcessSingle(BufferRadio *buf) {
  if (bufferRadioHasData(buf) && buf->gotAllPackets) {
    // Flush radio buffer to the driver
    bufferRadioFlush(buf);
    // Reset the radio buffer flags
    bufferRadioReset(buf);
  }
}

/**
* @description Used to determing if the buffer radio `buf` is in a locked state.
* @param `buf` {BufferRadio *} - The buffer to examine.
* @returns {boolen} - `true` if there is no lock on `buf`
* @author AJ Keller (@aj-ptw)
*/
boolean OpenBCI_RFDuino_BLE_Class::bufferRadioReadyForNewPage(BufferRadio *buf) {
  return !buf->flushing && !bufferRadioHasData(buf);
}

/**
* @description Used to reset the flags and positions of the radio buffer.
* @param `buf` {BufferRadio *} - The buffer to reset.
* @author AJ Keller (@aj-ptw)
*/
void OpenBCI_RFDuino_BLE_Class::bufferRadioReset(BufferRadio *buf) {
  buf->flushing = false;
  buf->gotAllPackets = false;
  buf->positionWrite = 0;
  buf->previousPacketNumber = 0;
}

/**
* @description Used to safely swap the global buffers!
* @returns {boolean} - `true` if the current radio buffer has been swapped,
*  `false` if the swap was not able to occur.
* @author AJ Keller (@aj-ptw)
*/
boolean OpenBCI_RFDuino_BLE_Class::bufferRadioSwitchToOtherBuffer(void) {
  if (OPENBCI_NUMBER_RADIO_BUFFERS == 2) {
    // current radio buffer is set to the first one
    if (currentRadioBuffer == bufferRadio) {
      if (bufferRadioReadyForNewPage(bufferRadio + 1)) {
        currentRadioBuffer++;
        return true;
      }
      // current radio buffer is set to the second one
    } else {
      if (bufferRadioReadyForNewPage(bufferRadio)) {
        currentRadioBuffer--;
        return true;
      }
    }
  }
  return false;
}

/**
* @description Stores a char to the serial buffer. Used by both the Device and
*  the Host.
* @param newChar {char} - The new char to store to the serial buffer.
* @return {boolean} - `true` if the new char was added to the serial buffer,
*  `false` on serial buffer overflow.
* @author AJ Keller (@aj-ptw)
*/
boolean OpenBCI_RFDuino_BLE_Class::bufferSerialAddChar(char newChar) {
  // Is the serial buffer overflowed?

  if (bufferSerial.overflowed) {
    // End the subroutine
    // Serial.println("OVR");
    return false;
  } else {
    // Is the current buffer's write position less than max size of 20?
    if (currentPacketBufferSerial->positionWrite < BYTES_PER_BLE_PACKET) {
      // Serial.printf("- packs 2 send: %d | pw: %d\n", bufferSerial.numberOfPacketsToSend, currentPacketBufferSerial->positionWrite);
      // Store the char into the serial buffer at the write position
      currentPacketBufferSerial->data[currentPacketBufferSerial->positionWrite] = newChar;
      // Increment the write position
      currentPacketBufferSerial->positionWrite++;
      // Set the number of packets to 1 initally, it will only grow
      if (bufferSerial.numberOfPacketsToSend == 0) {
        bufferSerial.numberOfPacketsToSend = 1;
      }
      // end successful subroutine read
      return true;

    } else {
      // Are we out of serial buffers?
      if (bufferSerial.numberOfPacketsToSend >= OPENBCI_NUMBER_SERIAL_BUFFERS - 1) {
        // Set the overflowed flag equal to true
        bufferSerial.overflowed = true;
        // Serial.println("OVR");
        // End the subroutine with a failure
        return false;

      } else {
        // Increment the current buffer pointer to the next one
        currentPacketBufferSerial++;
        // Increment the number of packets to send
        bufferSerial.numberOfPacketsToSend++;
        // Serial.printf("-- packs 2 send: %d | pw: %d\n", bufferSerial.numberOfPacketsToSend, currentPacketBufferSerial->positionWrite);

        // Store the char into the serial buffer at the write position
        currentPacketBufferSerial->data[currentPacketBufferSerial->positionWrite] = newChar;
        // Increment the write position
        currentPacketBufferSerial->positionWrite++;
        // End the subroutine with success
        return true;
      }
    }
  }
}

/**
* @description If there are packets to be sent in the serial buffer.
* @return {boolean} - `true` if there are packets waiting to be sent from the
*  serial buffer, `false` if not...
* @author AJ Keller (@aj-ptw)
*/
boolean OpenBCI_RFDuino_BLE_Class::bufferSerialHasData(void) {
  return bufferSerial.numberOfPacketsSent < bufferSerial.numberOfPacketsToSend;
}

/**
* @description Function to clean (clear/reset) the bufferSerial.
* @param - `n` - {uint8_t} - The number of packets you want to
*      clean, for example, on init, we would clean all packets, but on cleaning
*      from the RFduinoGZLL_onReceive() we would only clean the number of
*      packets actually used.
* @author AJ Keller (@aj-ptw)
*/
void OpenBCI_RFDuino_BLE_Class::bufferSerialReset(uint8_t n) {
  bufferCleanBuffer(&bufferSerial, n);
  currentPacketBufferSerial = bufferSerial.packetBuffer;
  // previousPacketNumber = 0;
}

/**
* @description Based off the last time the serial port was read from, Determines
*  if enough time has passed to qualify this data as a full serial page.
* @returns {boolean} - `true` if enough time has passed, `false` if not.
* @author AJ Keller (@aj-ptw)
*/
boolean OpenBCI_RFDuino_BLE_Class::bufferSerialTimeout(void) {
  return micros() > (lastTimeSerialRead + OPENBCI_TIMEOUT_PACKET_NRML_uS);
}

/**
* @description Process a char from the serial port on the Device. Enters the char
*  into the stream state machine.
* @param `buf` {BLEPacket *} - The ble buffer to add the char to.
* @param `newChar` {char} - A new char to process
* @author AJ Keller (@aj-ptw)
*/
void OpenBCI_RFDuino_BLE_Class::bufferBLEAddChar(BLEPacket *blePacket, char newChar) {
  // Process the new char
  // Serial.printf("%d 0x", blePacket->bytesIn);
  // if (newChar < 15) Serial.print("0");
  // Serial.print(newChar, HEX); Serial.println();

  switch (blePacket->state) {
    case STREAM_STATE_TAIL:
      // Is the current char equal to 0xCX where X is 0-F?
      if (isATailByte(newChar)) {
        // Set the type byte
        blePacket->data[0] = newChar;
        // Change the state to ready
        blePacket->state = STREAM_STATE_READY;
        // Serial.print(33); Serial.print(" state: "); Serial.print("READY-");
        // Serial.println((streamPacketBuffer + streamPacketBufferHead)->state);
      } else {
        // Reset the state machine
        blePacket->state = STREAM_STATE_INIT;
        // Set bytes in to 0
        blePacket->bytesIn = 0;
        // Test to see if this byte is a head byte, maybe if it's not a
        //  tail byte then that's because a byte was dropped on the way
        //  over from the Pic.
        if (newChar == OPENBCI_STREAM_PACKET_HEAD) {
          // Move the state
          blePacket->state = STREAM_STATE_STORING;
          // Set to 1
          blePacket->bytesIn = 1;
        }
      }
      break;
    case STREAM_STATE_STORING:
      // Serial.println(" store");
      // Serial.print("storing: 0x");
      // if (newChar < 15) Serial.print("0");
      // Serial.print(newChar, HEX);
      blePacket->data[blePacket->bytesIn++] = newChar;
      if (blePacket->bytesIn >= BYTES_PER_BLE_PACKET) {
        blePacket->state = STREAM_STATE_TAIL;
      }
      break;
    // We have called the function before we were able to send the stream
    //  packet which means this is not a stream packet, it's part of a
    //  bigger message
    case STREAM_STATE_READY:
      // Serial.println(" ready");
      blePacket->state = STREAM_STATE_INIT;
      blePacket->bytesIn = 0;
      // break; INTENTIONALLY COMMENTED BREAK OUT FOR TEST OF NEW CHAR AS STREAM
    case STREAM_STATE_INIT:
      // Serial.println(" init");
      if (newChar == OPENBCI_STREAM_PACKET_HEAD) {
        // Serial.println(" head");
        // Move the state
        blePacket->state = STREAM_STATE_STORING;
        // Set to 1
        blePacket->bytesIn = 1;
      }
      break;
    default:
      // // Reset the state
      blePacket->state = STREAM_STATE_INIT;
      // Set to 0
      blePacket->bytesIn = 0;
      break;

  }
}

/**
* @description Resets the stream packet buffer to default settings
* @param `buf` {StreamPacketBuffer *} - Pointer to a stream packet buffer to reset
* @author AJ Keller (@aj-ptw)
*/
void OpenBCI_RFDuino_BLE_Class::bufferStreamReset(void) {
  spBuffer.bytesIn = 0;
  spBuffer.state = STREAM_STATE_INIT;
}

/**
* @description Based off the last time the serial port was read from, determines
*  if enough time has passed to qualify this data as a stream packet.
* @returns {boolean} - `true` if enough time has passed, `false` if not.
* @author AJ Keller (@aj-ptw)
*/
boolean OpenBCI_RFDuino_BLE_Class::bufferStreamTimeout(void) {
  return micros() > (lastTimeSerialRead + OPENBCI_TIMEOUT_PACKET_STREAM_uS_9600);
}

/**
* @description Creates a byteId for sending data over the RFduinoGZLL
* @param isStreamPacket [boolean] Set true if this is a streaming packet
* @param packetNumber {uint8_t} What number packet are you trying to send?
* @param data [char *] The data you want to send. NOTE: Do not send the address
*           of the entire buffer, send this method address of buffer + 1
* @param length [int] The length of the data buffer
* @returns [char] The newly formed byteId where a byteId is defined as
*           Bit 7 - Streaming byte packet
*           Bits[6:3] - Packet count
*           Bits[2:0] - The check sum
* @author AJ Keller (@aj-ptw)
*/
char OpenBCI_RFDuino_BLE_Class::byteIdMake(boolean isStreamPacket, uint8_t packetNumber, char *data, uint8_t length) {
  // Set output initially equal to 0
  char output = 0x00;

  // Set first bit if this is a streaming packet
  if  (isStreamPacket) output = output | 0x80;

  // Set packet count bits Bits[6:3] NOTE: 0xFF is error
  // convert int to char then shift then or
  output = output | ((packetNumber & 0x0F) << 3);

  return output;
}

/**
* @description Determines if this byteId is a stream byte
* @param byteId [char] a byteId (see ::byteIdMake for description of bits)
* @returns [int] the check sum
* @author AJ Keller (@aj-ptw)
*/
boolean OpenBCI_RFDuino_BLE_Class::byteIdGetIsStream(uint8_t byteId) {
  return byteId > 0x7F;
}

/**
* @description Strips and gets the packet number from a byteId
* @param byteId [char] a byteId (see ::byteIdMake for description of bits)
* @returns [int] the packetNumber
* @author AJ Keller (@aj-ptw)
*/
int OpenBCI_RFDuino_BLE_Class::byteIdGetPacketNumber(uint8_t byteId) {
  return (int)((byteId & 0x78) >> 3);
}

/**
* @description Strips and gets the packet number from a byteId
* @param byteId [char] a byteId (see ::byteIdMake for description of bits)
* @returns [byte] the packet type
* @author AJ Keller (@aj-ptw)
*/
byte OpenBCI_RFDuino_BLE_Class::byteIdGetStreamPacketType(uint8_t byteId) {
  return (byte)((byteId & 0x78) >> 3);
}

/**
* @description Strips and gets the packet number from a byteId
* @returns [byte] the packet type
* @author AJ Keller (@aj-ptw)
*/
byte OpenBCI_RFDuino_BLE_Class::byteIdMakeStreamPacketType(uint8_t typeByte) {
  return typeByte & 0x0F;
}

/**
* @description Takes a byteId and converts to a Stop Byte for a streaming packet
* @param `byteId` - [byte] - A byteId with packet type in bits 6-3
* @return - [byte] - A stop byte with 1100 as the MSBs with packet type in the
*          four LSBs
* @example byteId == 0b10111000 returns 0b11000111
* @author AJ Keller (@aj-ptw)
*/
byte OpenBCI_RFDuino_BLE_Class::outputGetStopByteFromByteId(char byteId) {
  return byteIdGetStreamPacketType(byteId) | 0xC0;
}

OpenBCI_RFDuino_BLE_Class radioBLE;

/**
* @description Used to determine if there are packets in the serial buffer to be sent.
* @returns {boolean} - True if there are packets in the buffer and enough time
*  has passed
* @author AJ Keller (@aj-ptw)
*/
boolean OpenBCI_RFDuino_BLE_Class::packetToSend(void) {
  return packetsInSerialBuffer() && serialWriteTimeOut();
}

/**
* @description Used to determine if there are packets in the serial buffer to be sent.
* @returns {boolean} - True if there are packets in the buffer
* @author AJ Keller (@aj-ptw)
*/
boolean OpenBCI_RFDuino_BLE_Class::packetsInSerialBuffer(void) {
  return bufferSerial.numberOfPacketsSent < bufferSerial.numberOfPacketsToSend;
}

/**
* @description Used to see if enough time has passed since the last serial read. Useful to
*  if a serial transmission from the PC/Driver has concluded
* @returns {boolean} - `true` if enough time has passed
* @author AJ Keller (@aj-ptw)
*/
boolean OpenBCI_RFDuino_BLE_Class::serialWriteTimeOut(void) {
  return micros() > (lastTimeSerialRead + OPENBCI_TIMEOUT_PACKET_NRML_uS);
}
