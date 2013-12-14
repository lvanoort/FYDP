#include <Arduino.h>
#include "registers.h"
#include <Servo.h>


void setup()  
{
  Serial.begin(9600);
  while (!Serial) {
  }

  Serial.println("Initalization complete");
}

void loop()
{
  if (Serial.available() > 0) {
    unsigned char incomingByte = Serial.read();
    processByte(incomingByte);
  }
}
