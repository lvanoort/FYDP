#include "encoder.h"
#include <Arduino.h>

bool line_a;
bool line_b;

int count = 0;

int get_count()
{
  int temp = count;
  count = 0;
  return temp;
}

void chan_a()
{
  /* If pinA and pinB are both high or both low, it is spinning
   * forward. If they're different, it's going backward.
   *
   * For more information on speeding up this process, see
   * [Reference/PortManipulation], specifically the PIND register.
   */
  line_a = !line_a;

  if( !(line_b ^ line_a) )
  {
    count--;
  }
  else
  {
    count++;
  }

}

void chan_b()
{
  line_b = !line_b;

  if( (line_b ^ line_a) )
  {
    count--;
  }
  else
  {
    count++;
  }
}

void setup_encoders()
{

  pinMode (2,INPUT);
  pinMode (3,INPUT);
  int n = digitalRead(2);
  line_a = (n != LOW);
  n = digitalRead(3);
  line_b = (n != LOW);

  attachInterrupt(0,chan_a, CHANGE);
  attachInterrupt(1,chan_b, CHANGE);
}
