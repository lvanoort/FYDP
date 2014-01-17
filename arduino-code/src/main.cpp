#include <Arduino.h>
#include "registers.h"
#include <Servo.h>

#define LEFT_MOTOR_1 13
#define LEFT_MOTOR_2 14

#define DEBUG_MODE 0

Servo l_servo1; Servo l_servo2; 

unsigned long last_control;

unsigned long last_check = 0;

void check_reg()
{
  if (millis() - 1000 > last_check) {
      Serial.print("Reg 40 is ");
      Serial.print(integerRegisters[LEFT_MOTOR_CMD], DEC);
      Serial.println();
      last_check = millis();
  }
}

void setup()  
{
  Serial.begin(115200);
  while (!Serial) {
  }

  //initialize control timers
  last_control = last_message = last_check = millis();

  l_servo1.attach(LEFT_MOTOR_1);
  l_servo2.attach(LEFT_MOTOR_2);

  delay(2000);
  Serial.println("Initalization complete");
}

void loop()
{

  if (Serial.available() > 0) {
    unsigned char incomingByte = Serial.read();
    processByte(incomingByte);    
  }
  
#if DEBUG_MODE
  check_reg();
#endif
  
  //Security check: 
  //Ensure messages are being recieved from laptop
  //If no messages have been recieved over the last
  //second, halt all actuators
  if (millis() - 1000 > last_message) {
#if DEBUG_MODE
    static int no_message_count;
    no_message_count = (no_message_count + 1) % 50;
    if(no_message_count == 0) {
      Serial.println("No messages recieved for 1s");
    }
#endif
    
    l_servo1.write(90);
    l_servo2.write(90);
    delay(20);
    return;
  }
  
  //Write motor command
  if (millis() - 20 > last_control) { //20ms update
    l_servo1.write(90+integerRegisters[LEFT_MOTOR_CMD]);
    l_servo2.write(90+integerRegisters[LEFT_MOTOR_CMD]);
    last_control = millis();
  }
}
