#include "encoder.h"
#include <Arduino.h>

bool rline_a;
bool rline_b;

volatile int rcount = 0;

int get_count_r()
{
  int temp = rcount;
  rcount = 0;
  return temp;
}

void rchan_a()
{
  /* If pinA and pinB are both high or both low, it is spinning
   * forward. If they're different, it's going backward.
   *
   * For more information on speeding up this process, see
   * [Reference/PortManipulation], specifically the PIND register.
   */
  rline_a = !rline_a;

  if( !(rline_b ^ rline_a) )
  {
    rcount++;
  }
  else
  {
    rcount--;
  }

}

void rchan_b()
{
  rline_b = !rline_b;

  if( (rline_b ^ rline_a) )
  {
    rcount++;
  }
  else
  {
    rcount--;
  }
}

// Other encoders
bool lline_a;
bool lline_b;

volatile int lcount = 0;

int get_count_l()
{
  int temp = lcount;
  lcount = 0;
  return temp;
}

void lchan_a()
{
  /* If pinA and pinB are both high or both low, it is spinning
   * forward. If they're different, it's going backward.
   *
   * For more information on speeding up this process, see
   * [Reference/PortManipulation], specifically the PIND register.
   */
  lline_a = !lline_a;

  if( !(lline_b ^ lline_a) )
  {
    lcount++;
  }
  else
  {
    lcount--;
  }

}

void lchan_b()
{
  lline_b = !lline_b;

  if( (lline_b ^ lline_a) )
  {
    lcount++;
  }
  else
  {
    lcount--;
  }
}









void setup_encoders()
{
  //x side
  pinMode (2,INPUT);
  pinMode (3,INPUT);
  int n = digitalRead(2);
  rline_a = (n != LOW);
  n = digitalRead(3);
  rline_b = (n != LOW);

  attachInterrupt(0,rchan_a, CHANGE);
  attachInterrupt(1,rchan_b, CHANGE);

  //y side
  pinMode (18,INPUT);
  pinMode (19,INPUT);
  n = digitalRead(18);
  lline_a = (n != LOW);
  n = digitalRead(19);
  lline_b = (n != LOW);

  attachInterrupt(5,lchan_a, CHANGE);
  attachInterrupt(4,lchan_b, CHANGE);

}
