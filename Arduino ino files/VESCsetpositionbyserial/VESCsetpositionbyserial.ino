#include <VescUart.h>

/** Initiate VescUart class */
VescUart UART;

float position = 73.4; 
void setup() {
  Serial.begin(115200);
  /** Setup UART port (Serial1 on Atmega32u4) */
  Serial1.begin(115200);
  
  while (!Serial1) {;}

  /** Define which ports to use as UART */
  UART.setSerialPort(&Serial1);
}

void loop() {

  if (Serial.available())
  {
    position = Serial.readStringUntil('\n').toFloat();
  }

  UART.setPosition(position);
  Serial.println(position);
  
}
