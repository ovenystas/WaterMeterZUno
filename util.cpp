#include "util.h"


// Workaround for broken mills() function in Z-Uno 2.1.5
// Function my_millis() in util.h replaces millis()
uint32_t g_my_millis = 0;

uint32_t my_millis()
{
  return g_my_millis;
}
