#include "log.h"


void log_begin()
{
  uint32_t ms = my_millis();
  int32_t sec_ms = ms % 60000;
  ms /= 60000;
  uint8_t minutes = ms % 60;
  ms /= 60;
  uint16_t hours = ms;
  Serial0.print('[');
  Serial0.print(hours);
  Serial0.print(':');
  if (minutes < 10)
  {
    Serial0.print('0');
  }
  Serial0.print(minutes);  
  Serial0.print(':');  
  if (sec_ms < 10000)
  {
    Serial0.print('0');
  }
  Serial0.fixPrint(sec_ms, 3);
  Serial0.print("] ");
}
