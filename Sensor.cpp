#include "Sensor.h"
#include "util.h"


bool Sensor_isTimeToMeasure(Sensor_T* sensor_p)
{
  if (sensor_p->measure_interval_ms == 0)
  {
    return false;
  }

  if (sensor_p->time_measured_ms == 0)
  {
    return true;
  }

  return (my_millis() - sensor_p->time_measured_ms) >= sensor_p->measure_interval_ms;
}


void Sensor_setMeasured(Sensor_T* sensor_p)
{
  sensor_p->time_measured_ms = my_millis();
}


void Sensor_setMeasuredTime(Sensor_T* sensor_p, uint32_t time_ms)
{
  sensor_p->time_measured_ms = time_ms;
}


uint32_t Sensor_getMeasuredTime(Sensor_T* sensor_p)
{
  return sensor_p->time_measured_ms;
}


// TODO: Needs testing of all data types pos and neg
bool Sensor_isReportThresholdReached(Sensor_T* sensor_p)
{
  return abs(sensor_p->value.s32 - sensor_p->value_reported.s32) >=
    sensor_p->report_threshold.s32;
}


void Sensor_sendReport(Sensor_T* sensor_p)
{
  sensor_p->value_reported.s32 = sensor_p->value.s32;
  sensor_p->time_reported_ms = my_millis();
  zunoSendReport(sensor_p->zuno_channel);
}


bool Sensor_isReportIntervalReached(Sensor_T* sensor_p)
{
  if (sensor_p->report_interval_ms == 0)
  {
    return false;
  }

  return (my_millis() - sensor_p->time_reported_ms) >=
    sensor_p->report_interval_ms;
}


bool Sensor_isReportIntervalMinReached(Sensor_T* sensor_p)
{
  return (my_millis() - sensor_p->time_reported_ms) >=
    sensor_p->report_interval_min_ms;
}


bool Sensor_isTimeToSendReport(Sensor_T* sensor_p)
{
  return Sensor_isReportIntervalMinReached(sensor_p) &&
    (Sensor_isReportThresholdReached(sensor_p) ||
     Sensor_isReportIntervalReached(sensor_p));
}

// TODO: Needs testing of all data types pos and neg
void Sensor_setValueS(Sensor_T* sensor_p, int32_t value)
{
  sensor_p->value.s32 = value;
}


// TODO: Needs testing of all data types pos and neg
void Sensor_setValueU(Sensor_T* sensor_p, uint32_t value)
{
  sensor_p->value.u32 = value;
}

int16_t Sensor_getValueS16(Sensor_T* sensor_p)
{
  return sensor_p->value.s16;
}

uint16_t Sensor_getValueU16(Sensor_T* sensor_p)
{
  return sensor_p->value.u16;
}

int32_t Sensor_getValueS32(Sensor_T* sensor_p)
{
  return sensor_p->value.s32;
}

uint32_t Sensor_getValueU32(Sensor_T* sensor_p)
{
  return sensor_p->value.u32;
}
