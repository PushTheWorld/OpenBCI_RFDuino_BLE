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
#define NUM_BYTES_BLE_PACKET 20
#include <RFduinoBLE.h>

// send 500 20 byte buffers = 10000 bytes
int packets = 500;

// flag used to start sending
int flag = false;

// variables used in packet generation
int ch;
int packet;
volatile boolean newChar = false;
uint8_t inputBuffer[NUM_BYTES_INPUT_BUFFER];
volatile uint8_t inputBufferHead = 0;
volatile uint8_t inputBufferTail = 0;

uint8_t outputBuffer[NUM_BYTES_BLE_PACKET];
uint8_t outputBufferPosition = 0;

int start;

void setup() {
  // Serial.begin(9600);

  // Without this line the RFDuino will fail to boot with serial baud 115200
  override_uart_limit = true;
  Serial.begin(115200);


  RFduinoBLE.advertisementData = "OBCI";
  Serial.println("Waiting for connection...");
  RFduinoBLE.begin();
}

void serialEvent(void){
  if (inputBufferHead >= NUM_BYTES_INPUT_BUFFER) {
    inputBufferHead = 0;
  }
  inputBuffer[inputBufferHead++] = Serial.read();
}

// Used to send the recieved data from iPhone to the board
void RFduinoBLE_onReceive(char *data, int len) {
  uint8_t myByte = data[0];
  Serial.write(myByte);
}

void RFduinoBLE_onConnect() {
  packet = 0;
  ch = 'A';
  start = 0;
  flag = true;
  Serial.println("Sending");
  // first send is not possible until the iPhone completes service/characteristic discovery
}

void loop() {
  boolean packetReadyToSend = false;
  while (inputBufferHead != inputBufferTail) {
    packetReadyToSend = true;
    if (inputBufferTail >= NUM_BYTES_INPUT_BUFFER) {
      inputBufferTail = 0;
    }
    outputBuffer[outputBufferPosition++] = inputBuffer[inputBufferTail++];
    if (outputBufferPosition >= NUM_BYTES_BLE_PACKET) {
      break; // out of while loop
    }
  }
  if (packetReadyToSend) {
    // send is queued (the ble stack delays send to the start of the next tx window)
    while (! RFduinoBLE.send(outputBuffer, NUM_BYTES_BLE_PACKET))
      ;  // all tx buffers in use (can't send - try again later)

    outputBufferPosition = 0;
    packetReadyToSend = false;
  }
}
