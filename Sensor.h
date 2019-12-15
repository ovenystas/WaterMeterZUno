#pragma once

#include <Arduino.h>

union Value_U
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
  union Value_U report_threshold;

  union Value_U value;
  union Value_U value_reported;
  uint32_t time_measured_ms;
  uint32_t time_reported_ms;
} Sensor_T;

void Sensor_setMeasured(Sensor_T* sensor_p);
void Sensor_setMeasuredTime(Sensor_T* sensor_p, uint32_t time_ms);

uint32_t Sensor_getMeasuredTime(Sensor_T* sensor_p);

bool Sensor_isTimeToMeasure(Sensor_T* sensor_p);
bool Sensor_isReportThresholdReached(Sensor_T* sensor_p);
bool Sensor_isReportIntervalReached(Sensor_T* sensor_p);
bool Sensor_isReportIntervalMinReached(Sensor_T* sensor_p);
bool Sensor_isTimeToSendReport(Sensor_T* sensor_p);

void Sensor_sendReport(Sensor_T* sensor_p);

void Sensor_setValueS(Sensor_T* sensor_p, int32_t value);
void Sensor_setValueU(Sensor_T* sensor_p, uint32_t value);

int16_t Sensor_getValueS16(Sensor_T* sensor_p);
uint16_t Sensor_getValueU16(Sensor_T* sensor_p);
int32_t Sensor_getValueS32(Sensor_T* sensor_p);
uint32_t Sensor_getValueU32(Sensor_T* sensor_p);
