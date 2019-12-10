#pragma once

#include <Arduino.h>

union Value_u
{
  int16_t s16;
  uint16_t u16;
  int32_t s32;
  uint32_t u32;
};

typedef struct Sensor_s
{
  uint8_t zuno_channel;
  uint16_t measure_interval_s;
  uint16_t report_interval_s;
  uint16_t report_interval_min_s;
  union Value_u report_threshold;

  union Value_u value;
  union Value_u value_reported;
  uint32_t time_measured_s;
  uint32_t time_reported_s;
} Sensor_t;

bool Sensor_isTimeToMeasure(Sensor_t* sensor_p);
void Sensor_setMeasured(Sensor_t* sensor_p);
bool Sensor_isReportThresholdReached(Sensor_t* sensor_p);
void Sensor_sendReport(Sensor_t* sensor_p);
bool Sensor_isReportIntervalReached(Sensor_t* sensor_p);
bool Sensor_isReportIntervalMinReached(Sensor_t* sensor_p);
void Sensor_setValueS(Sensor_t* sensor_p, int32_t value);
void Sensor_setValueU(Sensor_t* sensor_p, uint32_t value);
