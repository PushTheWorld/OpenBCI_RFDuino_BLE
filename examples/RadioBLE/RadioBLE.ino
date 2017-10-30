/*
 Copyright (c) 2013 OpenSourceRF.com.  All right reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 See the GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*
This sketch demonstrates how to do error free bulk data
transfer over Bluetooth Low Energy 4.

The data rate should be approximately:
  - 32 kbit/sec at 1.5ft (4000 bytes per second)
  - 24 kbit/sec at 40ft (3000 bytes per second)

This sketch sends a fixed number of 20 byte packets to
an iPhone application.  Each packet is different, so
that the iPhone application can verify if any data or
packets were dropped.
*/
#define NUM_BYTES_INPUT_BUFFER 50
#define NUM_INPUT_BUFFERS 50
#define BYTES_PER_BLE_PACKET 20
#define TIME_TO_SEND_ONE_BYTE_US_115200 80
#define TIME_TO_SEND_ONE_BYTE_US_57600 150
#define TIME_TO_SEND_ONE_BYTE_US_9600 940
#include <RFduinoBLE.h>

typedef struct {
  uint8_t data[BYTES_PER_BLE_PACKET];
  int     positionWrite;
} RawBuffer;

// variables used in packet generation
int start;

RawBuffer rawBuffer[NUM_INPUT_BUFFERS];

// uint8_t inputBuffer[NUM_BYTES_INPUT_BUFFER];
uint8_t outputBuffer[BYTES_PER_BLE_PACKET];
uint8_t outputBufferPosition = 0;
uint8_t sampleCounter = 0;

unsigned long timeOfLastHeadShift_uS = micros();
// Realistically need to send 20 bytes every 12ms or 12000us
unsigned long interPacketInterval_uS = 12000;
unsigned long lastTimeSerialRead = 0;
unsigned long timeToSendOneByte = TIME_TO_SEND_ONE_BYTE_US_115200 + 100;

volatile boolean connectedDevice = false;
volatile boolean newChar = false;

// volatile uint8_t inputBufferHead = 0;
// volatile uint8_t inputBufferTail = 0;
volatile uint8_t head = 0;
volatile uint8_t tail = 0;

// void rawBufferClean(RawBuffer *buf) {
//   for(uint8_t i = 0; i < BYTES_PER_BLE_PACKET; i++) {
//     buf->data[i] = 0;
//   }
//   buf->positionWrite = 0;
// }
//
// void rawBufferCleanAll() {
//   for (uint8_t i = 0; i < NUM_INPUT_BUFFERS; i++) {
//     rawBufferClean(rawBuffer + i);
//   }
// }


void setup() {
  // Serial.begin(9600);

  // Without this line the RFDuino will fail to boot with serial baud 115200
  override_uart_limit = true;
  Serial.begin(115200);
  timeToSendOneByte = TIME_TO_SEND_ONE_BYTE_US_115200 + 3;
  // Serial.begin(57600);
  // timeToSendOneByte = TIME_TO_SEND_ONE_BYTE_US_57600 + 3;


  RFduinoBLE.advertisementData = "OBCI";
  Serial.println("Waiting for connection...");
  RFduinoBLE.begin();
}

void serialEvent(void){
  sampleCounter++;
  if (sampleCounter > 69 && sampleCounter < 90) {
    if ((rawBuffer+head)->positionWrite >= BYTES_PER_BLE_PACKET) {
      // need to move pointer of
      head++;
      if (head >= NUM_INPUT_BUFFERS) {
        head = 0;
      }
      (rawBuffer+head)->positionWrite = 0;
    }
    (rawBuffer+head)->data[(rawBuffer+head)->positionWrite++] = Serial.read();
    lastTimeSerialRead = micros();
  }
  if (sampleCounter > 99) {
   sampleCounter = 0;
  }
}

// Used to send the recieved data from iPhone to the board
void RFduinoBLE_onReceive(char *data, int len) {
  uint8_t myByte = data[0];
  Serial.write(myByte);
}

void RFduinoBLE_onConnect() {
  connectedDevice = true;
  Serial.println("Connected");
  // first send is not possible until the iPhone completes service/characteristic discovery
}

void RFduinoBLE_onDisconnect() {
  connectedDevice = false;
  Serial.println("Disconnected");
  // first send is not possible until the iPhone completes service/characteristic discovery
}
uint8_t counter=0;

void loop() {

  if (connectedDevice && (micros() > (lastTimeSerialRead + timeToSendOneByte))) {
    // boolean packetReadyToSend = false;
    if (head != tail) {
      // packetReadyToSend = true;
      (rawBuffer + tail)->data[0] = counter++;
      (rawBuffer + tail)->data[1] = head;
      (rawBuffer + tail)->data[2] = tail;
      // memcpy(, outputBuffer, BYTES_PER_BLE_PACKET);
      while (! RFduinoBLE.send((const char *)(rawBuffer + tail)->data, BYTES_PER_BLE_PACKET))
        ;  // all tx buffers in use (can't send - try again later)
      (rawBuffer + tail)->positionWrite = 0;
      tail++;
      if (tail >= NUM_INPUT_BUFFERS) {
        tail = 0;
      }
      // outputBuffer[outputBufferPosition++] = inputBuffer[inputBufferTail++];
      // if (outputBufferPosition >= BYTES_PER_BLE_PACKET) {
        // Serial.println("outputBufferPosition break");
        // break; // out of while loop
      // }
    }
    // if (packetReadyToSend) {
    //   // send is queued (the ble stack delays send to the start of the next tx window)
    //   // Serial.print("send queued: ");
    //   // RFduinoBLE.send((const char *)outputBuffer, BYTES_PER_BLE_PACKET);
    //   // if (bytesToSend > BYTES_PER_BLE_PACKET) bytesToSend = BYTES_PER_BLE_PACKET;
    //   while (! RFduinoBLE.send((const char *)outputBuffer, BYTES_PER_BLE_PACKET))
    //     ;  // all tx buffers in use (can't send - try again later)
    //   // Serial.println("packet was sent");
    //   outputBufferPosition = 0;
    //   packetReadyToSend = false;
    // }
  }
  // if (micros() > timeOfLastHeadShift_uS + interPacketInterval_uS) {
  //   timeOfLastHeadShift_uS = micros();
  //   inputBufferHead += 19;
  //   if (inputBufferHead >= NUM_BYTES_INPUT_BUFFER) {
  //     inputBufferHead -= NUM_BYTES_INPUT_BUFFER;
  //   }
  // }
}
