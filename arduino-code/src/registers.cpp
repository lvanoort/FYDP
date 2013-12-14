#include "registers.h"

volatile int integerRegisters[128];
volatile float floatRegisters[128];

#define UART_RX_BUFFER_SIZE 8
unsigned char uartRxBuffer[UART_RX_BUFFER_SIZE];
unsigned int uartRxNextByte;  //Next to send
unsigned int uartRxFreeByte;  //Next free
volatile char uartRxLock;

inline void processBuffer()
{
  if(uartRxBuffer[0] != 0xFF || uartRxBuffer[1] != 0xFF)
  {
    uartRxBuffer[0] = uartRxBuffer[1];
    uartRxBuffer[1] = uartRxBuffer[2];
    uartRxBuffer[2] = uartRxBuffer[3];
    uartRxBuffer[3] = uartRxBuffer[4];
    uartRxBuffer[4] = uartRxBuffer[5];
    uartRxBuffer[5] = uartRxBuffer[6];
    uartRxBuffer[6] = uartRxBuffer[7];
    uartRxFreeByte = 7;
    return;
  }

  int address = (uartRxBuffer[2] << 8) | uartRxBuffer[3];
  int data = (uartRxBuffer[4] <<8) | uartRxBuffer[5];
  int checksum = data + address;
  int compare  = (uartRxBuffer[6] << 8) | uartRxBuffer[7];

  if (checksum != compare)
  {
    uartRxBuffer[0] = uartRxBuffer[1];
    uartRxBuffer[1] = uartRxBuffer[2];
    uartRxBuffer[2] = uartRxBuffer[3];
    uartRxBuffer[3] = uartRxBuffer[4];
    uartRxBuffer[4] = uartRxBuffer[5];
    uartRxBuffer[5] = uartRxBuffer[6];
    uartRxBuffer[6] = uartRxBuffer[7];
    uartRxFreeByte = 7;
    return;
  }

  //to be lazy, just mask bits and such
  //TODO: add the check to write to int or float registers
  unsigned char add = uartRxBuffer[3] & 0b01111111;
  unsigned char dat = uartRxBuffer[5];
  uartRxFreeByte = 0;
  integerRegisters[add] = dat;
}

void processByte(unsigned char incomingByte)
{
  uartRxBuffer[uartRxFreeByte] = incomingByte;
  uartRxFreeByte++;
  if(uartRxFreeByte == 8)
    processBuffer();
}
