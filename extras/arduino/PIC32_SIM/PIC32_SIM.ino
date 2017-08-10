/*
  DigitalReadSerial
 Reads a digital input on pin 2, prints the result to the serial monitor

 This example code is in the public domain.
 */

#define SAMPLE_RATE_HZ 250
#define INTERPACKET_SEND_INTERVAL_MS 1000/SAMPLE_RATE_HZ
unsigned long lastTimePacketSent = 0;
boolean streaming = false;

uint8_t counter = 0;
// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
}

// the loop routine runs over and over again forever:
void loop() {
  if (streaming && millis() > (lastTimePacketSent + INTERPACKET_SEND_INTERVAL_MS)) {
    serialWriteAStreamPacket(counter++);
    lastTimePacketSent = millis();
  }
  if (Serial.available()) {
    char newChar = Serial.read();
    switch (newChar) {
      case 'b':
        streaming = true;
        break;
      case 's':
        streaming = false;
        break;
      default:
        break;
    }
  }
}

void serialWriteAStreamPacket(uint8_t sampleNumber) {
  Serial.write(0x41);
  Serial.write(sampleNumber);
  Serial.write(0); Serial.write(0); Serial.write(0);
  Serial.write(0); Serial.write(0); Serial.write(1);
  Serial.write(0); Serial.write(0); Serial.write(2);
  Serial.write(0); Serial.write(0); Serial.write(3);
  Serial.write(0); Serial.write(0); Serial.write(4);
  Serial.write(0); Serial.write(0); Serial.write(5);
  Serial.write(0); Serial.write(0); Serial.write(6);
  Serial.write(0); Serial.write(0); Serial.write(7);
  Serial.write(0); Serial.write(0);
  Serial.write(0); Serial.write(1);
  Serial.write(0); Serial.write(2);
  Serial.write(0xC0);
  // 4101000001000001000001000001000001000001000001000001000100010001C0
}
