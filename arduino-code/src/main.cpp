#include <Arduino.h>
#include "registers.h"
#include "encoder.h"
#include "controller.h"
#include <Servo.h>

//Talons have a 0.9ms to 2.2ms pulse width range
#define TALON_MIN_PW 900 
#define TALON_MAX_PW 2200

#define LEFT_MOTOR_1 13
#define LEFT_MOTOR_2 14

#define RIGHT_MOTOR_1 15
#define RIGHT_MOTOR_2 16

#define DEBUG_MODE 0

#define DEBUG_ENCODER 0

#define FEEDBACK //activate feedback, else feedforward 
#define KP_LEFT 0.2
#define KP_RIGHT 0.2
#define SPEED_SCALING_FACTOR 0.512 //ticks per millisecond to metres per second

Servo l_servo1; Servo l_servo2; 
Servo r_servo1; Servo r_servo2; 

unsigned long last_control;
unsigned long last_sensor;

unsigned long last_check = 0;

unsigned long last_read_l = 0;
unsigned long last_read_r = 0;
double current_speed_l;
double current_speed_r;


void check_reg()
{
  if (millis() - 1000 > last_check) {
      Serial.print("Reg 40 is ");
      Serial.print(integerRegisters[LEFT_MOTOR_CMD], DEC);
      Serial.println();
      last_check = millis();
  }
}

void check_count()
{
  if (millis() - 1000 > last_check) {
      Serial.print(get_count_r(), DEC);
      Serial.println();
      last_check = millis();
  }
}

void callback()
{
  Serial.print(".");
}

void setup()  
{
  Serial.begin(115200);
  //while (!Serial) {
  //}

  //initialize control timers
  last_sensor = last_control = last_message = last_check = millis();

  l_servo1.attach(LEFT_MOTOR_1,TALON_MIN_PW,TALON_MAX_PW);
  l_servo2.attach(LEFT_MOTOR_2,TALON_MIN_PW,TALON_MAX_PW);
  r_servo1.attach(RIGHT_MOTOR_1,TALON_MIN_PW,TALON_MAX_PW);
  r_servo2.attach(RIGHT_MOTOR_2,TALON_MIN_PW,TALON_MAX_PW);
  setup_encoders();

  delay(2000);
  Serial.println("Initalization complete v0.2");
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
  
#if DEBUG_ENCODER
  check_count();
#endif

  // Send sensor data
  if (millis() - 20 > last_sensor) { //20ms update
    //TODO send bytes out
    
	 //probably excessive storing of times and such, but assuming time is constant
	 //while sending serial data makes me uncomfortable
	 int r_count = get_count_r();
    unsigned long current_time = millis();
	 current_speed_r = ((double)r_count / (current_time-last_read_r))*SPEED_SCALING_FACTOR;
	 last_read_r = current_time;

	 Serial.print("RC");
    Serial.print(r_count, DEC);
	 Serial.print("RO");
    Serial.print(integerRegisters[RIGHT_MOTOR_CMD], DEC);
	 

	 int l_count = get_count_l();
	 current_time = millis();
    current_speed_l = ((double)l_count / (current_time-last_read_l))*SPEED_SCALING_FACTOR;
    last_read_l = current_time;

	 Serial.print("LC");
    Serial.print(l_count, DEC);
	 Serial.print("LO");
	 Serial.print(integerRegisters[LEFT_MOTOR_CMD], DEC);
    Serial.println();
    last_sensor = millis();
  }
  
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
    r_servo1.write(90);
    r_servo2.write(90);
    delay(20);
    return;
  }

  //Write motor command
  if (millis() - 20 > last_control) { //20ms update
   #ifndef FEEDBACK 
	 l_servo1.write(90+integerRegisters[LEFT_MOTOR_CMD]);
    l_servo2.write(90+integerRegisters[LEFT_MOTOR_CMD]);
    r_servo1.write(90+integerRegisters[RIGHT_MOTOR_CMD]);
    r_servo2.write(90+integerRegisters[RIGHT_MOTOR_CMD]);
    last_control = millis();
	#else
	 int cmdL = proportional(LEFT_MOTOR_CMD, current_speed_l, KP_LEFT);
	 int cmdR = proportional(RIGHT_MOTOR_CMD, current_speed_r, KP_RIGHT);
	 l_servo1.write(90+cmdL);
    l_servo2.write(90+cmdL);
    r_servo1.write(90+cmdR);
    r_servo2.write(90+cmdR);

	#endif
  }
}
