#include "delay.h"


void Delay_Ms(volatile int n)
{
  volatile int i, j;
  for (i = 0; i < n; i++){
    for (j = 0; j < 3180; j++)
    ;
  }
}


