#include "Arduino.h"
#include "NBDelay.h"


bool nb_delay(unsigned long *start_millis, unsigned long to_delay)
{   
  if(*start_millis == 0)
  {
    *start_millis = millis();
    return false;
  }

  if((unsigned long)(millis() - *start_millis) < to_delay)
  {
    return false;
  }

  *start_millis = 0;
  return true;
}
