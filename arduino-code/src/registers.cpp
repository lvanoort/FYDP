#include "registers.h"
#include <Arduino.h>

int integerRegisters[128];
float floatRegisters[128];
unsigned long last_message;

#define UART_RX_BUFFER_SIZE 10
unsigned char uartRxBuffer[UART_RX_BUFFER_SIZE];
unsigned int uartRxNextByte;  //Next to send
unsigned int uartRxFreeByte;  //Next free
char uartRxLock;

//prints buffer for debugging
inline void printBuffer()
{
    Serial.print(uartRxBuffer[0],DEC);
    Serial.println();
    Serial.print(uartRxBuffer[1],DEC);
    Serial.println();
    Serial.print(uartRxBuffer[2],DEC);
    Serial.println();
    Serial.print(uartRxBuffer[3],DEC);
    Serial.println();
    Serial.print(uartRxBuffer[4],DEC);
    Serial.println();
    Serial.print(uartRxBuffer[5],DEC);
    Serial.println();
    Serial.print(uartRxBuffer[6],DEC);
    Serial.println();
    Serial.print(uartRxBuffer[7],DEC);
    Serial.println();
    Serial.print(uartRxBuffer[8],DEC);
    Serial.println();
    Serial.print(uartRxBuffer[9],DEC);
    Serial.println();
}

//shifts buffer to get rid of oldest byte
inline void shiftBuffer()
{
    uartRxBuffer[0] = uartRxBuffer[1];
    uartRxBuffer[1] = uartRxBuffer[2];
    uartRxBuffer[2] = uartRxBuffer[3];
    uartRxBuffer[3] = uartRxBuffer[4];
    uartRxBuffer[4] = uartRxBuffer[5];
    uartRxBuffer[5] = uartRxBuffer[6];
    uartRxBuffer[6] = uartRxBuffer[7];
    uartRxBuffer[7] = uartRxBuffer[8];
    uartRxBuffer[8] = uartRxBuffer[9];
    uartRxFreeByte = 9;
}

inline void processBuffer()
{
  if(uartRxBuffer[0] != 0xFF || uartRxBuffer[1] != 0xFF)
  {
    shiftBuffer();
    return;
  }

  int address = (uartRxBuffer[2] << 8) | uartRxBuffer[3];
  int data[2];
  data[0] = (uartRxBuffer[4] <<8) | uartRxBuffer[5];
  data[1] = (uartRxBuffer[6] <<8) | uartRxBuffer[7];
  int checksum = data[0] + data[1] + address;
  int compare  = (uartRxBuffer[8] << 8) | uartRxBuffer[9];

  if (checksum != compare)
  {
    printBuffer();
    Serial.println("Bad checksum");
    shiftBuffer();
    return;
  }

  //to be lazy, just mask bits and such
  //TODO: add the check to write to int or float registers
  unsigned char add = uartRxBuffer[3] & 0b01111111;
  uartRxFreeByte = 0;
  integerRegisters[add] = data[0];
  integerRegisters[add+1] = data[1];

  /*Serial.print("Message get add: ");
  Serial.print(add,DEC);
  Serial.print(" data1: ");
  Serial.print(data[0],DEC);
  Serial.print(" data2: ");
  Serial.print(data[1],DEC);*/
  
  last_message = millis();
}

void processByte(unsigned char incomingByte)
{
  uartRxBuffer[uartRxFreeByte] = incomingByte;
  uartRxFreeByte++;
  if(uartRxFreeByte == 10)
    processBuffer();
}
