/*
  DigitalReadSerial
 Reads a digital input on pin 2, prints the result to the serial monitor

 This example code is in the public domain.
 */

#define SAMPLE_RATE_HZ 42
#define INTERPACKET_SEND_INTERVAL_MS 24
unsigned long lastTimePacketSent = 0;
boolean streaming = false;

uint8_t counter = 0;
// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial0.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  if (streaming && millis() > (lastTimePacketSent + INTERPACKET_SEND_INTERVAL_MS)) {
    serialWriteABLEPacket(counter);
    counter += 3;
    lastTimePacketSent = millis();
  }
  if (Serial0.available()) {
    char newChar = Serial0.read();
    switch (newChar) {
      case 'b':
        counter = 0;
        streaming = true;
        break;
      case 's':
        streaming = false;
        break;
      case 'v':
        softReset();
        break;
      default:
        break;
    }
  }
}

void serialWriteABLEPacket(uint8_t sampleNumber) {
  Serial0.write((uint8_t)0x41);
  Serial0.write(sampleNumber);
  Serial0.write((uint8_t)0x00); Serial0.write((uint8_t)0x00); Serial0.write((uint8_t)0x01);
  Serial0.write((uint8_t)0x00); Serial0.write((uint8_t)0x00); Serial0.write((uint8_t)0x02);
  Serial0.write((uint8_t)0x00); Serial0.write((uint8_t)0x00); Serial0.write((uint8_t)0x03);
  Serial0.write((uint8_t)0x00); Serial0.write((uint8_t)0x00); Serial0.write((uint8_t)0x04);
  Serial0.write((uint8_t)0x00); Serial0.write((uint8_t)0x00); Serial0.write((uint8_t)0x05);
  Serial0.write((uint8_t)0x00); Serial0.write((uint8_t)0x00); Serial0.write((uint8_t)0x06);
  Serial0.write((uint8_t)0xC0);
}

void softReset() {
  Serial0.println("OpenBCI V3 8-16 channel");
  Serial0.println("On Board ADS1299 Device ID: 0x3E");
  Serial0.println("LIS3DH Device ID: 0xA0");
  Serial0.println("Firmware: v3.0.0");
}
