#include <RFduinoBLE.h>
#include "OpenBCI_RFDuino_BLE.h"
#include "PTW-Arduino-Assert.h"

int ledPin = 2;

void setup() {
  // Get's serial up and running
  pinMode(ledPin,OUTPUT);
  Serial.begin(115200);
  test.setSerial(Serial);
  test.failVerbosity = true;
}

void loop() {
  // Start tests by just sending a command
  if (Serial.available()) {
    Serial.read();
    go();
  }
}

void go() {
  // Start the test
  test.begin();
  digitalWrite(ledPin, HIGH);

  testBufferBLE();
  testProcessChar();
  testBufferBLEAddChar();
  testProcessRadioChar();
  testByteIdMakeStreamPacketType();

  digitalWrite(ledPin, LOW);
  test.end();
}

void testBufferBLE() {
  testBufferBLEHeadMove();
  testBufferBLEHeadReadyToMove();
  testBufferBLEReset();
  testBufferBLETailSend();
  testBufferBLETailReadyToSend();
}

void testBufferBLEHeadMove() {
  test.describe("bufferBLEHeadMove");

  radioBLE.bufferBLEReset();
  radioBLE.bufferStreamReset();

  test.assertFalse(radioBLE.bufferBLEHeadReadyToMove(), "should not be ready to move", __LINE__);

  writeABLEPacketToAddChar(0xC0);
  radioBLE.lastTimeSerialRead = micros() - OPENBCI_TIMEOUT_PACKET_STREAM_uS_9600 * 2;
  test.assertTrue(radioBLE.bufferBLEHeadReadyToMove(), "should be ready to move", __LINE__);

}

void testBufferBLEHeadReadyToMove() {
  test.describe("bufferBLEHeadReadyToMove");
  radioBLE.bufferBLEReset();
  radioBLE.bufferStreamReset();

  for (int i = 0; i < NUM_BLE_PACKETS; i++) {
    test.assertEqual(radioBLE.head, i);
    radioBLE.bufferBLEHeadMove();
  }
  test.assertEqual(radioBLE.head, 0, "head should wrap around", __LINE__);

}

void testBufferBLEReset() {
  test.describe("bufferBLEReset");
  radioBLE.bufferBLEReset();
  radioBLE.bufferStreamReset();

  writeABLEPacketToAddChar(0xC0);

  radioBLE.bufferBLEReset(radioBLE.bufferBLE);

  test.assertEqual(radioBLE.bufferBLE->bytesIn, (int)0, "should reset bytesIn to 0", __LINE__);
  test.assertEqual(radioBLE.bufferBLE->state, radioBLE.STREAM_STATE_INIT, "should set state to init", __LINE__);
}

void testBufferBLETailMove() {
  test.describe("bufferBLETailMove");
  radioBLE.bufferStreamReset();
  radioBLE.bufferBLEReset();

  test.it("should move tail if connected device is true");
  for (int i = 0; i < NUM_BLE_PACKETS; i++) {
    (radioBLE.bufferBLE + radioBLE.tail)->state = radioBLE.STREAM_STATE_READY;
    test.assertEqual(radioBLE.tail, i, "should move tail", __LINE__);
    radioBLE.bufferBLETailMove();
    if (i > 0) {
      test.assertEqual((radioBLE.bufferBLE + i - 1)->state, radioBLE.STREAM_STATE_INIT, "should have reset last tail", __LINE__);
    }
  }
  test.assertEqual((radioBLE.bufferBLE + NUM_BLE_PACKETS)->state, radioBLE.STREAM_STATE_INIT, "should have reset the last tail", __LINE__);
  test.assertEqual(radioBLE.tail, 0, "tail should wrap around", __LINE__);

}

void testBufferBLETailSend() {
  test.describe("bufferBLETailSend");
  radioBLE.bufferBLEReset();
  radioBLE.bufferStreamReset();

  test.it("should not move tail if no connected device");
  radioBLE.bufferBLETailSend();
  test.assertEqual(radioBLE.tail, (int)0, "should not have moved the tail");
}

void testBufferBLETailReadyToSend() {
  test.describe("bufferBLETailReadyToSend");
  radioBLE.bufferBLEReset();
  radioBLE.bufferStreamReset();

  test.it("should not be ready to send tail if head equals tail");
  test.assertFalse(radioBLE.bufferBLETailReadyToSend(), "should not be ready to send tail ble packet", __LINE__);

  test.it("should still not be ready to send tail because packet not ready");
  radioBLE.bufferBLEHeadMove();
  test.assertFalse(radioBLE.bufferBLETailReadyToSend(), "should not be ready to send tail ble packet", __LINE__);

  radioBLE.bufferBLEReset();

  test.it("should be ready to send once packet loaded into tail ble packet and head is moved");
  writeABLEPacketToAddChar(0xC0);
  radioBLE.lastTimeSerialRead = micros() - OPENBCI_TIMEOUT_PACKET_STREAM_uS_9600 * 2;
  writeABLEPacketToAddChar(0xC0);
  radioBLE.lastTimeSerialRead = micros() - OPENBCI_TIMEOUT_PACKET_STREAM_uS_9600 * 2;
  writeABLEPacketToAddChar(0xC0);
  radioBLE.bufferBLEHeadMove();
  test.assertTrue(radioBLE.bufferBLETailReadyToSend(), "should be ready to send tail ble packet", __LINE__);
}


void testBufferBLEAddChar() {
  test.describe("bufferBLEAddChar");

  testBufferBLEAddChar_STREAM_STATE_INIT();
  testBufferBLEAddChar_STREAM_STATE_READY();
  testBufferBLEAddChar_STREAM_STATE_STORING();
  testBufferBLEAddChar_STREAM_STATE_TAIL();
}

void testBufferBLEAddChar_STREAM_STATE_INIT() {
  test.detail("STREAM_STATE_INIT");
  char newChar = (char)0x00;

  test.it("should recognize the start byte and change state to storing");
  radioBLE.bufferBLEReset(radioBLE.bufferBLE);
  newChar = (char)OPENBCI_STREAM_PACKET_HEAD;
  radioBLE.bufferBLEAddChar(radioBLE.bufferBLE, newChar);
  test.assertEqualHex(radioBLE.bufferBLE->state, radioBLE.STREAM_STATE_STORING, "should enter state storing", __LINE__);
  test.assertEqual(radioBLE.bufferBLE->bytesIn, (uint8_t)1, "should have stored no bytes to ble packet", __LINE__);

  test.it("should do nothing if not the start byte");
  radioBLE.bufferBLEReset(radioBLE.bufferBLE);
  newChar = (char)0xF9;
  radioBLE.bufferBLEAddChar(radioBLE.bufferBLE, newChar);
  test.assertEqualHex(radioBLE.bufferBLE->state, radioBLE.STREAM_STATE_INIT, "should stay in init state", __LINE__);
  test.assertNotEqual(radioBLE.bufferBLE->data[0], (int)newChar, "should not have stored the new char", __LINE__);
  test.assertEqual(radioBLE.bufferBLE->bytesIn, (uint8_t)0, "should not have read any bytes in", __LINE__);
}

void testBufferBLEAddChar_STREAM_STATE_READY() {
  test.detail("STREAM_STATE_READY");

  char newChar = (char)0xC0;
  uint8_t expected_sampleNumber = 70;
  uint8_t sampleNumber = expected_sampleNumber;

  test.it("should set the ble packet to ready state");
  newChar = (char)OPENBCI_STREAM_PACKET_TAIL;
  radioBLE.bufferBLEReset(radioBLE.bufferBLE);
  writeABLEPacketToAddChar(newChar);
  test.assertEqualHex(radioBLE.bufferBLE->state, radioBLE.STREAM_STATE_READY, "radioBLE should be in the ready state", __LINE__);
}

void testBufferBLEAddChar_STREAM_STATE_STORING() {
  test.detail("STREAM_STATE_STORING");

  char newChar = (char)0xC0;
  uint8_t initialBytesIn = 5;
  uint8_t sampleNumber = 30;
  uint8_t expected_sampleNumber = sampleNumber;
  test.it("should store the new char to the buffer in position of bytesIn and increment byteIn by 1");
  uint8_t expected_bytesIn = 5;
  radioBLE.bufferBLEReset(radioBLE.bufferBLE);
  radioBLE.bufferBLEAddChar(radioBLE.bufferBLE, OPENBCI_STREAM_PACKET_HEAD); // make the first one a stream one so 0x41
  radioBLE.bufferBLEAddChar(radioBLE.bufferBLE, sampleNumber); // Sample number what have you
  bufferBLEAdd3Byte(1);
  test.assertEqualHex(radioBLE.bufferBLE->state, radioBLE.STREAM_STATE_STORING, "should remain in state storing", __LINE__);
  test.assertEqualHex(radioBLE.bufferBLE->data[1], sampleNumber, "should have stored the sampleNumber to 2nd bytesIn position", __LINE__);
  test.assertEqualHex(radioBLE.bufferBLE->data[expected_bytesIn-1], 1, "should have stored 1 in last bytesIn position", __LINE__);
  test.assertEqualHex(radioBLE.bufferBLE->bytesIn, expected_bytesIn, "should have incremented bytes in to 5",__LINE__);
  bufferBLEAdd3Byte(2);
  bufferBLEAdd3Byte(3);
  bufferBLEAdd3Byte(4);
  bufferBLEAdd3Byte(5);
  test.assertEqualHex(radioBLE.bufferBLE->state, radioBLE.STREAM_STATE_STORING, "should remain in state storing", __LINE__);
  bufferBLEAdd3Byte(6);
  test.assertEqualHex(radioBLE.bufferBLE->state, radioBLE.STREAM_STATE_TAIL, "should be in tail state", __LINE__);
}

void testBufferBLEAddChar_STREAM_STATE_TAIL() {
  test.detail("STREAM_STATE_TAIL");

  char newChar = 'A';

  test.it("should set the typeByte and set state to ready");
  newChar = (char)OPENBCI_STREAM_PACKET_TAIL;
  radioBLE.bufferBLEReset(radioBLE.bufferBLE);
  radioBLE.bufferBLE->bytesIn = BYTES_PER_BLE_PACKET;
  radioBLE.bufferBLE->state = radioBLE.STREAM_STATE_TAIL;
  radioBLE.bufferBLEAddChar(radioBLE.bufferBLE, newChar);
  test.assertEqualHex(radioBLE.bufferBLE->data[0], newChar, "should set the type byte to the stop byte", __LINE__);
  test.assertEqualHex(radioBLE.bufferBLE->state, radioBLE.STREAM_STATE_READY, "should be in the ready state", __LINE__);

  test.it("should set state to init if byte is not stop byte or head byte");
  newChar = (char)0x00;
  radioBLE.bufferBLEReset(radioBLE.bufferBLE);
  radioBLE.bufferBLE->bytesIn = BYTES_PER_BLE_PACKET;
  radioBLE.bufferBLE->state = radioBLE.STREAM_STATE_TAIL;
  radioBLE.bufferBLEAddChar(radioBLE.bufferBLE, newChar);
  test.assertEqualHex(radioBLE.bufferBLE->state, radioBLE.STREAM_STATE_INIT, "should be in the ready state", __LINE__);
  test.assertEqualHex(radioBLE.bufferBLE->bytesIn, 0, "should set bytesIn back to 0", __LINE__);

  test.it("should set the state to storing if the byte is a head byte");
  radioBLE.bufferBLEReset(radioBLE.bufferBLE);
  radioBLE.bufferBLE->bytesIn = BYTES_PER_BLE_PACKET;
  radioBLE.bufferBLE->state = radioBLE.STREAM_STATE_TAIL;
  newChar = (char)OPENBCI_STREAM_PACKET_HEAD;
  radioBLE.bufferBLEAddChar(radioBLE.bufferBLE, newChar);
  test.assertEqualHex(radioBLE.bufferBLE->state, radioBLE.STREAM_STATE_STORING, "should enter state storing", __LINE__);
  test.assertEqualHex(radioBLE.bufferBLE->bytesIn, 1, "should have read one byte in",__LINE__);
}

void testProcessChar() {
  testIsATailByte();
  testProcessCharSingleChar();
  testProcessCharStreamPacket();
  testProcessCharStreamPackets();
  testProcessCharStreamReal();
  testProcessCharNotStreamPacket();
  testProcessCharOverflow();
}

void testIsATailByte() {
  test.describe("isATailByte");

  test.assertBoolean(radioBLE.isATailByte(0xC0),true,"Stream packet type 0",__LINE__);
  test.assertBoolean(radioBLE.isATailByte(0xC1),true,"Stream packet type 1",__LINE__);
  test.assertBoolean(radioBLE.isATailByte(0xC8),true,"Stream packet type 8",__LINE__);
  test.assertBoolean(radioBLE.isATailByte(0xCA),true,"Stream packet type 10",__LINE__);
  test.assertBoolean(radioBLE.isATailByte(0xCF),true,"Stream packet type 15",__LINE__);
  test.assertBoolean(radioBLE.isATailByte(0xB0),false,"Not a stream packet type",__LINE__);

  // Remember to clean up after yourself
  testProcessChar_CleanUp();
}

void testProcessCharSingleChar() {
  test.describe("processCharForSingleChar");

  // Clear the buffers
  radioBLE.bufferSerialReset(OPENBCI_NUMBER_SERIAL_BUFFERS);
  radioBLE.bufferBLEReset(radioBLE.bufferBLE);

  // try to add a char
  char input = OPENBCI_STREAM_PACKET_TAIL;
  // Store it to serial buffer
  radioBLE.bufferSerialAddChar(input);
  // Get one char and process it
  radioBLE.bufferBLEAddChar(radioBLE.bufferBLE,input);

  // Verify serial buffer
  test.assertEqual(radioBLE.bufferSerial.packetBuffer->data[radioBLE.bufferSerial.packetBuffer->positionWrite - 1], (char)input,"Char stored to serial buffer",__LINE__);
  test.assertEqual(radioBLE.bufferSerial.numberOfPacketsToSend, (int)1, "Serial buffer has 1 packet to send", __LINE__);
  test.assertEqual(radioBLE.bufferSerial.numberOfPacketsSent, (int)0, "Serial buffer not sent any packets", __LINE__);
  // Verify stream packet buffer
  test.assertEqual(radioBLE.bufferBLE->data[0], (char)input, "Char stored to stream packet buffer", __LINE__);

  // Remember to clean up after yourself
  testProcessChar_CleanUp();

}

void testProcessCharStreamPacket() {
  test.describe("processCharForStreamPacket");
  test.it("should recognze a stream packet and wait 88us before allowing the stream packet to be sent with stop byte of 0xC0");

  // Clear the buffers
  radioBLE.bufferSerialReset(OPENBCI_NUMBER_SERIAL_BUFFERS);
  radioBLE.bufferStreamReset();
  radioBLE.bufferBLEReset();

  // Write a stream packet with end byte 0xC0
  writeABLEPacketToAddChar(0xC0);

  // Right away we want to see if enough time has passed, this should be false
  //  because we just processed a char, after this test is complete, we should
  //  be far passed 90uS
  test.assertBoolean(radioBLE.bufferStreamTimeout(),false,"waiting...",__LINE__);

  // Do we have a stream packet waiting to launch?
  test.assertEqualHex(radioBLE.bufferBLE->state, radioBLE.STREAM_STATE_READY, "state ready with 0xC0",__LINE__);

  // This should return true this time
  test.assertBoolean(radioBLE.bufferStreamTimeout(),true,"able to send",__LINE__);

  ///////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////
  // Try a stream packet with another stop byte /////////////////////////////
  ///////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////
  test.it("should recognze a stream packet and wait 88us before allowing the stream packet to be sent with stop byte of 0xC5");
  // Clear the buffers
  radioBLE.bufferSerialReset(OPENBCI_NUMBER_SERIAL_BUFFERS);
  radioBLE.bufferStreamReset();

  // Write a stream packet with end byte not 0xC0
  writeABLEPacketToAddChar(0xC5);

  // Right away we want to see if enough time has passed, this should be false
  //  because we just processed a char, after this test is complete, we should
  //  be far passed 90uS
  test.assertBoolean(radioBLE.bufferStreamTimeout(),false,"Type 5 waiting",__LINE__);

  // Do we have a stream packet waiting to launch?
  test.assertEqualHex(radioBLE.bufferBLE->state, radioBLE.STREAM_STATE_READY, "state ready with 0xC5",__LINE__);

  // This should return true this time
  test.assertBoolean(radioBLE.bufferStreamTimeout(),true,"Type 5 ready",__LINE__);

  // Remember to clean up after yourself
  testProcessChar_CleanUp();

}

void getMeAStreamPacket(uint8_t *output) {
  output[0] = 0x41;
  output[1] = 0;
  output[2] = 0; output[3] = 0; output[4] = 0;
  output[5] = 0; output[6] = 0; output[7] = 1;
  output[8] = 0; output[9] = 0; output[10] = 2;
  output[11] = 0; output[12] = 0; output[13] = 3;
  output[14] = 0; output[15] = 0; output[16] = 4;
  output[17] = 0; output[18] = 0; output[19] = 5;
  output[20] = 0; output[21] = 0; output[22] = 6;
  output[23] = 0; output[24] = 0; output[25] = 7;
  output[26] = 0; output[27] = 0;
  output[28] = 0; output[29] = 1;
  output[30] = 0; output[31] = 2;
  output[32] = 0xC0;
}

void getMeABLEPacket(uint8_t *output) {
  output[0] = 0x41;
  output[1] = 0;
  output[2] = 0; output[3] = 0; output[4] = 1;
  output[5] = 0; output[6] = 0; output[7] = 2;
  output[8] = 0; output[9] = 1; output[10] = 1;
  output[11] = 0; output[12] = 1; output[13] = 2;
  output[14] = 0; output[15] = 2; output[16] = 1;
  output[17] = 0; output[18] = 2; output[19] = 2;
  output[20] = 0xC0;
}

void testProcessCharStreamReal() {
  test.detail("Process Real Stream");
  radioBLE.bufferBLEReset(radioBLE.bufferBLE);
  radioBLE.bufferSerialReset(OPENBCI_NUMBER_SERIAL_BUFFERS);

  boolean run = true;
  uint8_t maxSize = BYTES_PER_BLE_PACKET+1;
  uint8_t blePacket[BYTES_PER_BLE_PACKET+1];
  blePacket[0] = OPENBCI_STREAM_PACKET_HEAD;
  blePacket[20] = OPENBCI_STREAM_BYTE_STOP;
  getMeABLEPacket(blePacket);
  uint8_t packetPosition = 0;
  uint8_t numPacket = 0;
  uint8_t maxPackets = 5;
  uint8_t packetsSent = 0;
  unsigned long now = millis();
  unsigned long testTimeLimit = 10000;
  while (run) {
    if (millis() > testTimeLimit+now) {
      run = false;
      test.fail("timeout - not able to produce a ble packet", __LINE__);
    } else {
      if (packetPosition <= BYTES_PER_BLE_PACKET) {
        radioBLE.bufferSerialAddChar(blePacket[packetPosition]);
        radioBLE.bufferBLEAddChar(radioBLE.bufferBLE + radioBLE.head, blePacket[packetPosition]);
        radioBLE.lastTimeSerialRead = micros();
        delayMicroseconds(OPENBCI_TIMEOUT_PACKET_STREAM_uS_9600); // OPENBCI_TIMEOUT_PACKET_STREAM_uS
      }

      if (radioBLE.bufferBLEHeadReadyToMove()) { // Is there a stream packet waiting to get sent to the Host?
        radioBLE.bufferBLEHeadMove();
        Serial.println("head move");
      }

      if (radioBLE.bufferBLETailReadyToSend()) { // Is there a stream packet waiting to get sent to the Host?
        packetsSent++;
        if (packetsSent >= maxPackets) {
          run = false;
          test.pass("should have sent all packets", __LINE__);
        }
        // Serial.print("0x");
        // for (int i = 0; i < 20; i++) {
        //   if ((radioBLE.bufferBLE + radioBLE.tail)->data[i] < 10) {
        //     Serial.print("0");
        //   }
        //   Serial.print((radioBLE.bufferBLE + radioBLE.tail)->data[i], HEX);
        // }
        // Serial.println();
        // Serial.write((radioBLE.bufferBLE + radioBLE.tail)->data, 20);Serial.println();
        radioBLE.bufferBLETailMove();
        // Serial.println("tail moved");

      }

      packetPosition++;
      if (packetPosition > 21 && packetPosition < 25) {
        delayMicroseconds(100);
      } else if (packetPosition > 30) {
        packetPosition = 0;
        blePacket[1] += 3;
      }
    }
  }
}

// Send stream packets, one after the other
void testProcessCharStreamPackets() {
  test.describe("processCharForStreamPackets");

  // Clear the buffers
  radioBLE.bufferSerialReset(OPENBCI_NUMBER_SERIAL_BUFFERS);
  radioBLE.bufferStreamReset();
  radioBLE.bufferBLEReset();

  int numberOfTrials = 9;
  // char testMessage[] = "Sent 0";
  // unsigned long t1, t2;

  for (int i = 0; i < numberOfTrials; i++) {
    // Write a stream packet with end byte 0xC0
    writeABLEPacketToAddChar(0xC0);
    radioBLE.lastTimeSerialRead = micros();
    // Stream packet should be waiting
    test.assertEqualHex(radioBLE.bufferBLE->state,radioBLE.STREAM_STATE_READY,"state ready",__LINE__);
    // Wait
    while(!radioBLE.bufferStreamTimeout()) {};
    // configure the test message
    // testMessage[5] = (i + 1) + '0';
    // Send the stream packet
    if (radioBLE.bufferBLE->state == radioBLE.STREAM_STATE_READY) {
      RFduinoBLE.send((const char *)radioBLE.bufferBLE->data, BYTES_PER_BLE_PACKET);
    }

    radioBLE.bufferBLEReset(radioBLE.bufferBLE);

  }
}

// Test conditions that result in a stream packet not being launched
void testProcessCharNotStreamPacket() {
  test.describe("processCharForNotStreamPacket");

  // Clear the buffers
  radioBLE.bufferSerialReset(OPENBCI_NUMBER_SERIAL_BUFFERS);
  radioBLE.bufferStreamReset();
  radioBLE.bufferBLEReset();

  // Write a stream packet
  writeABLEPacketToAddChar(0xC0);
  // Fake Serial read
  char newChar = (char)0xFF;
  // Save current time as the last serial read
  radioBLE.lastTimeSerialRead = micros();
  // Quick! Write another char
  radioBLE.bufferBLEAddChar(radioBLE.bufferBLE, newChar);
  test.assertEqualHex(radioBLE.spBuffer.state,radioBLE.STREAM_STATE_INIT,"state init",__LINE__);
  test.assertEqual(radioBLE.spBuffer.bytesIn, (int)0, "0 bytes in",__LINE__);
  test.assertEqualHex(radioBLE.bufferBLE->state,radioBLE.STREAM_STATE_INIT,"state init",__LINE__);
  test.assertEqual(radioBLE.bufferBLE->bytesIn, (int)0, "0 bytes in",__LINE__);

  // Clear the buffers
  radioBLE.bufferSerialReset(OPENBCI_NUMBER_SERIAL_BUFFERS);
  radioBLE.bufferStreamReset();
  radioBLE.bufferBLEReset();

  // Write a stream packet with a bad end byte
  writeABLEPacketToAddChar(0xB5);
  test.assertEqualHex(radioBLE.spBuffer.state,radioBLE.STREAM_STATE_INIT,"bad end byte state init",__LINE__);
  test.assertEqualHex(radioBLE.bufferBLE->state,radioBLE.STREAM_STATE_INIT,"bad end byte state init",__LINE__);

  // Remember to clean up after yourself
  testProcessChar_CleanUp();
}

// Put the system in an overflow condition
void testProcessCharOverflow() {
  test.describe("testProcessCharOverflow");

  // Clear the buffers
  radioBLE.bufferSerialReset(OPENBCI_NUMBER_SERIAL_BUFFERS);
  radioBLE.bufferStreamReset();
  radioBLE.bufferBLEReset();
  // Write the max number of bytes in buffers
  int maxBytes = (OPENBCI_NUMBER_SERIAL_BUFFERS - 1) * (BYTES_PER_BLE_PACKET - 1);
  // Write max bytes but stop 1 before
  for (int i = 0; i < maxBytes; i++) {
    // Serial.printf("i: %d ", i);
    radioBLE.bufferSerialAddChar(0x00);
  }

  // Verify that the emergency stop flag has NOT been deployed
  test.assertFalse(radioBLE.bufferSerial.overflowed, "Overflow emergency not hit", __LINE__);
  // Verify that there are 15 buffers filled
  test.assertEqual(radioBLE.bufferSerial.numberOfPacketsToSend, (int)(OPENBCI_NUMBER_SERIAL_BUFFERS - 1), "15 buffers", __LINE__);
  // Verify the write position
  test.assertEqual(radioBLE.currentPacketBufferSerial->positionWrite, BYTES_PER_BLE_PACKET, "20 bytes in buffer", __LINE__);

  // Write one more byte to overflow the buffer
  // Serial.printf("maxBytes: %d\n", maxBytes);
  radioBLE.bufferSerialAddChar(0x00);
  // Verify that the emergency stop flag has been deployed
  test.assertTrue(radioBLE.bufferSerial.overflowed, "Overflow emergency", __LINE__);

  // Remember to clean up after yourself
  testProcessChar_CleanUp();
}

void testProcessChar_CleanUp() {
  // Clear the buffers
  radioBLE.bufferSerialReset(OPENBCI_NUMBER_SERIAL_BUFFERS);
  radioBLE.bufferStreamReset();
  radioBLE.bufferBLEReset();
}

/********************************************/
/********************************************/
/******    Process Radio Char Tests    ******/
/********************************************/
/********************************************/
void testProcessRadioChar() {
  testPacketToSend();
}

// This is used to determine if there is in fact a packet waiting to be sent
void testPacketToSend() {
  test.describe("testPacketToSend");
  // Clear the buffers
  radioBLE.bufferSerialReset(OPENBCI_NUMBER_SERIAL_BUFFERS);
  radioBLE.bufferStreamReset();
  radioBLE.bufferBLEReset();
  // Set the buffers up to think there is a packet to be sent
  //  by triggering a serial read
  char input = 'A';
  // Set last serial read to now
  radioBLE.lastTimeSerialRead = micros();
  // Process that char!
  radioBLE.bufferSerialAddChar(input);
  // Less than 3ms has passed, veryify we can't send a packet
  test.assertBoolean(radioBLE.packetToSend(),false,"Can't send packet yet",__LINE__);
  // Wait for 3 ms
  delayMicroseconds(3000);
  // Re ask if there is something to send
  test.assertBoolean(radioBLE.packetToSend(),true,"Enough time passed",__LINE__);

}

void testByteIdMakeStreamPacketType() {
  test.describe("byteIdMakeStreamPacketType");

  test.assertEqual(radioBLE.byteIdMakeStreamPacketType(0xC5),5,"Can get type 5",__LINE__);

  testProcessChar_CleanUp();
}

void writeABLEPacketToAddChar(char endByte) {
  // Quickly write a bunch of bytes into the buffers
  radioBLE.bufferBLEAddChar(radioBLE.bufferBLE, 0x41); // make the first one a stream one so 0x41
  radioBLE.bufferBLEAddChar(radioBLE.bufferBLE, 0x00); // Sample number what have you

  for (byte i = 0; i < 6; i++) {
    bufferBLEAdd3Byte(i);
  }
  // 5 bytes - channel 1.1
  // 8 bytes - channel 1.2
  // 11 bytes - channel 2.1
  // 14 bytes - channel 2.2
  // 17 bytes - channel 3.1
  // 20 bytes - channel 3.2

  radioBLE.bufferBLEAddChar(radioBLE.bufferBLE, endByte);

  radioBLE.lastTimeSerialRead = micros();
}

// void writeAStreamPacketToAddChar(char endByte) {
//   // Quickly write a bunch of bytes into the buffers
//   radioBLE.bufferBLEAddChar(radioBLE.bufferBLE, 0x41); // make the first one a stream one so 0x41
//   radioBLE.bufferBLEAddChar(radioBLE.bufferBLE, 0x00); // Sample number what have you
//
//   for (byte i = 0; i < 8; i++) {
//     bufferStreamAdd3Byte(i);
//   }
//   // 5 bytes - channel 1
//   // 8 bytes - channel 2
//   // 11 bytes - channel 3
//   // 14 bytes - channel 4
//   // 17 bytes - channel 5
//   // 20 bytes - channel 6
//   // 23 bytes - channel 7
//   // 26 bytes - channel 8
//
//   for (byte i = 0; i < 3; i++) {
//       bufferStreamAdd2Byte(i);
//   }
//   // 28 bytes - Aux 1
//   // 30 bytes - Aux 2
//   // 32 bytes - Aux 3
//
//   radioBLE.bufferBLEAddChar(radioBLE.bufferBLE, endByte); // This locks in the final stream packet
//   radioBLE.lastTimeSerialRead = micros();
// }

void bufferBLEAdd3Byte(byte n) {
  radioBLE.bufferBLEAddChar(radioBLE.bufferBLE, (char)0x00);
  radioBLE.bufferBLEAddChar(radioBLE.bufferBLE, (char)0x00);
  radioBLE.bufferBLEAddChar(radioBLE.bufferBLE, (char)n);
}

void bufferStreamAdd3Byte(byte n) {
  radioBLE.bufferBLEAddChar(radioBLE.bufferBLE, (char)0x00);
  radioBLE.bufferBLEAddChar(radioBLE.bufferBLE, (char)0x00);
  radioBLE.bufferBLEAddChar(radioBLE.bufferBLE, (char)n);
}

void bufferStreamAdd2Byte(byte n) {
  radioBLE.bufferBLEAddChar(radioBLE.bufferBLE, (char)0x00);
  radioBLE.bufferBLEAddChar(radioBLE.bufferBLE, (char)n);
}
