#ifndef REGISTERS_H
#define REGISTERS_H

volatile extern int integerRegisters[128];
volatile extern float floatRegisters[128];

//TODO: inline?
void processByte(unsigned char incomingByte);

#endif
