/*
  DigitalReadSerial
 Reads a digital input on pin 2, prints the result to the serial monitor

 This example code is in the public domain.
 */

unsigned long lastTimePacketSent = 0;
uint8_t *output = "pushtheworldajkellerpushtheworld!";

uint8_t counter = 0;
// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);

  output[0] = 0x41;
  output[2] = 0; output[3] = 0; output[4] = 0;
  output[5] = 0; output[6] = 0; output[7] = 0;
  output[32] = 0xC0;
}

// the loop routine runs over and over again forever:
void loop() {
  if (millis() > (lastTimePacketSent + 4)) {
    output[1] = counter++;
    Serial.write((const char *)output, 33);
   lastTimePacketSent = millis();
  }
}
