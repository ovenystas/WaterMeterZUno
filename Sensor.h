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
  uint32_t measure_interval_ms;
  uint32_t report_interval_ms;
  uint32_t report_interval_min_ms;
  union Value_u report_threshold;

  union Value_u value;
  union Value_u value_reported;
  uint32_t time_measured_ms;
  uint32_t time_reported_ms;
} Sensor_t;

bool Sensor_isTimeToMeasure(Sensor_t* sensor_p);
void Sensor_setMeasured(Sensor_t* sensor_p);
void Sensor_setMeasuredTime(Sensor_t* sensor_p, uint32_t time_ms);
uint32_t Sensor_getMeasuredTime(Sensor_t* sensor_p);
bool Sensor_isReportThresholdReached(Sensor_t* sensor_p);
void Sensor_sendReport(Sensor_t* sensor_p);
bool Sensor_isReportIntervalReached(Sensor_t* sensor_p);
bool Sensor_isReportIntervalMinReached(Sensor_t* sensor_p);
bool Sensor_isTimeToSendReport(Sensor_t* sensor_p);
void Sensor_setValueS(Sensor_t* sensor_p, int32_t value);
void Sensor_setValueU(Sensor_t* sensor_p, uint32_t value);
int16_t Sensor_getValueS16(Sensor_t* sensor_p);
uint16_t Sensor_getValueU16(Sensor_t* sensor_p);
int32_t Sensor_getValueS32(Sensor_t* sensor_p);
uint32_t Sensor_getValueU32(Sensor_t* sensor_p);
