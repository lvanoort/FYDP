#ifndef REGISTERS_H
#define REGISTERS_H

extern int integerRegisters[128];
extern float floatRegisters[128];
extern unsigned long last_message;

#define LEFT_MOTOR_CMD 40
#define RIGHT_MOTOR_CMD 41

//TODO: inline?
void processByte(unsigned char incomingByte);

#endif
