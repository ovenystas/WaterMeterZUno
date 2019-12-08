#include <Arduino.h>
#include "crc.h"

#ifdef __cplusplus
extern "C"
{
#endif


uint8_t crc_calc(const uint8_t *data, size_t len)
{
  uint8_t crc = 0;

  if (len)
  {
    uint8_t eor;
    do
    {
      unsigned int i = 8;
      crc ^= *data++;
      do
      {
        /* This might produce branchless code */
        eor = crc & 0x80 ? 0xdf : 0;
        crc <<= 1;
        crc ^= eor;
      } while (--i);
    } while (--len);
  }

  return crc;
}

#ifdef __cplusplus
}
#endif
