#include "Sensor.h"

bool Sensor_isTimeToMeasure(Sensor_t* sensor_p)
{
  if (sensor_p->measure_interval_s == 0)
  {
    return false;
  }
  if (sensor_p->time_measured_s == 0)
  {
    return true;
  }
  uint32_t time_since_measured = (millis() / 1000) - sensor_p->time_measured_s;
  return time_since_measured >= sensor_p->measure_interval_s;
}

void Sensor_setMeasured(Sensor_t* sensor_p)
{
  sensor_p->time_measured_s = millis() / 1000;
}

// TODO: Needs testing of all data types pos and neg
bool Sensor_isReportThresholdReached(Sensor_t* sensor_p)
{
  return abs(sensor_p->value.s32 - sensor_p->value_reported.s32) >= sensor_p->report_threshold.s32;
}

void Sensor_sendReport(Sensor_t* sensor_p)
{
  sensor_p->value_reported.s32 = sensor_p->value.s32;
  sensor_p->time_reported_s = millis() / 1000;
  zunoSendReport(sensor_p->zuno_channel);
}

bool Sensor_isReportIntervalReached(Sensor_t* sensor_p)
{
  if (sensor_p->report_interval_s == 0)
  {
    return false;
  }
  uint32_t time_since_reported = (millis() / 1000) - sensor_p->time_reported_s;
  return time_since_reported >= sensor_p->report_interval_s;
}

bool Sensor_isReportIntervalMinReached(Sensor_t* sensor_p)
{
  uint32_t time_since_reported = (millis() / 1000) - sensor_p->time_reported_s;
  return time_since_reported >= sensor_p->report_interval_min_s;
}

// TODO: Needs testing of all data types pos and neg
void Sensor_setValueS(Sensor_t* sensor_p, int32_t value)
{
  sensor_p->value.s32 = value;
}

// TODO: Needs testing of all data types pos and neg
void Sensor_setValueU(Sensor_t* sensor_p, uint32_t value)
{
  sensor_p->value.u32 = value;
}
