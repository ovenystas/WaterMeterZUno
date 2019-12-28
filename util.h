#pragma once

#include <Arduino.h>


#define MS_PER_HOUR 3600000UL
#define S_PER_HOUR 3600U

#define DIV_ROUND_CLOSEST(n, d) \
  ((((n) < 0) ^ ((d) < 0)) ? (((n) - (d) / 2) / (d)) : (((n) + (d) / 2) / (d)))


// Workaround for broken mills() function in Z-Uno 2.1.5
// Function my_millis() in util.h replaces millis()
extern uint32_t g_my_millis;
uint32_t my_millis();
