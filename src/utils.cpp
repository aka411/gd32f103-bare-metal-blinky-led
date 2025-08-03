#include "utils.h"

//software delay
void softDelay(const uint32_t time)
{
  volatile uint32_t count = 0;
  while(count < time)
  {
    count++;
  }
}
