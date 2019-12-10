/*
 * This is a simple 3 channel water consumption and meter and temperature sensor
 * 
 * Ove Nystas 2019
 */
 
#include <EEPROM.h>
#include <ZUNO_DS18B20.h>
#include "crc.h"
#include "Sensor.h"

#define VERSION "0.01"

// channel numbers
#define ZUNO_CHANNEL_WATER_METER 1
#define ZUNO_CHANNEL_WATER_FLOW 2
#define ZUNO_CHANNEL_WATER_TEMPERATURE 3

#define PIN_SENSOR 17 // Is also INT0
#define PIN_DS18B20 11

#define EEPROM_ADDR 0x800
#define EEPROM_UPDATE_INTERVAL_MS  120000

#define TICK_VALUE 1 // in liters

#define METER_CHECK_INTERVAL_MS 100
#define TEMPERATURE_MEASURE_INTERVAL_S 30
#define TEMPERATURE_REPORT_INTERVAL_S 30
#define TEMPERATURE_REPORT_INTERVAL_MIN_S (SENSOR_MULTILEVEL_REPORT_INTERVAL_MS / 1000)

#define SENSOR_MULTILEVEL_REPORT_INTERVAL_MS 30000

// TODO: Make this a Z-Wave configurable parameter
#define TEMP_DIFF_REPORT 2 // In degree * 10

#define MS_PER_HOUR 3600000UL

#define DIV_ROUND_CLOSEST(n, d) ((((n) < 0) ^ ((d) < 0)) ? (((n) - (d)/2)/(d)) : (((n) + (d)/2)/(d)))


typedef struct MeterData_s
{
  uint32_t ticks;
  uint8_t  crc8;
} MeterData_t;

// Variables for water meter
MeterData_t meter_data = {0};
uint32_t new_pulse_ms = 0;
uint32_t last_update_ms = 0;
bool new_meter_data = false;
bool triggered = false;
uint32_t last_run_meter_ms = 0;

// Variables for water flow
uint16_t water_flow_lph = 0;
uint32_t last_pulse_ms = 0;
uint32_t last_water_flow_report_ms = 0;

// Variables for temperature sensor
OneWire ow(PIN_DS18B20);
DS18B20Sensor ds18b20(&ow);
byte addr1[8] = {0xFF};
bool temp_sensor_found = false;
struct Sensor_s sensor_temperature =
{
  ZUNO_CHANNEL_WATER_TEMPERATURE,
  TEMPERATURE_MEASURE_INTERVAL_S,
  TEMPERATURE_REPORT_INTERVAL_S,
  TEMPERATURE_REPORT_INTERVAL_MIN_S,
  { TEMP_DIFF_REPORT },
  { 0 } , { 0 },
  0, 0
};

// Configure Z-Uno
ZUNO_SETUP_DEBUG_MODE(DEBUG_ON);
ZUNO_SETUP_SLEEPING_MODE(ZUNO_SLEEPING_MODE_ALWAYS_AWAKE);
ZUNO_SETUP_ISR_INT0(int0_handler);

// Sensor multilevel type for water flow is not yet defined in Z-uno but defined in spec v9.
// Unit: Liter per hour (l/h)
#define ZUNO_SENSOR_MULTILEVEL_TYPE_WATER_FLOW 0x38
#define SENSOR_MULTILEVEL_SCALE_LITER_PER_HOUR 0x00

// Set up the Z-Uno channels
ZUNO_SETUP_CHANNELS(
  ZUNO_METER(
    ZUNO_METER_TYPE_WATER,
    METER_RESET_ENABLE,
    ZUNO_METER_WATER_SCALE_METERS3,
    METER_SIZE_FOUR_BYTES,
    METER_PRECISION_THREE_DECIMALS,
    getWaterMeter,
    resetWaterMeter
  ),
  ZUNO_SENSOR_MULTILEVEL(
    ZUNO_SENSOR_MULTILEVEL_TYPE_WATER_FLOW, 
    SENSOR_MULTILEVEL_SCALE_LITER_PER_HOUR, 
    SENSOR_MULTILEVEL_SIZE_TWO_BYTES, 
    SENSOR_MULTILEVEL_PRECISION_ZERO_DECIMALS,
    getWaterFlow
  ),
  ZUNO_SENSOR_MULTILEVEL(
	  ZUNO_SENSOR_MULTILEVEL_TYPE_WATER_TEMPERATURE, 
	  SENSOR_MULTILEVEL_SCALE_CELSIUS, 
	  SENSOR_MULTILEVEL_SIZE_TWO_BYTES, 
	  SENSOR_MULTILEVEL_PRECISION_ONE_DECIMAL,
	  getWaterTemperature
	)
);


void int0_handler() {
  new_pulse_ms = millis();
  triggered = true;
}


void setup()
{
  // Setup serial port. Wait for input in 10 seconds.
  Serial.begin(115200);
  delay(10000);

  meterSetup();
  tempSetup();

  printVersion();
  printNodeId();
  printTempSensorAddress();
}


void loop()
{
  if (triggered)
  {
    meterCheck();
  }
  meterEepromUpdate();
  
  if (temp_sensor_found)
  {
    if (Sensor_isTimeToMeasure(&sensor_temperature))
    {
      int16_t tempC10 = tempMeasure();
   
      Sensor_setValueS(&sensor_temperature, tempC10);
      Sensor_setMeasured(&sensor_temperature);

      if (tempC10 == BAD_TEMP)
      {
        Serial.println("ERROR: Invalid temperature");
      }
      else
      {
        Serial.print("TempC10: ");
        Serial.println(tempC10);
      }
    }
    
    if (Sensor_isReportIntervalMinReached(&sensor_temperature))
    {
      if (Sensor_isReportThresholdReached(&sensor_temperature) ||
          Sensor_isReportIntervalReached(&sensor_temperature))
      {
          Serial.println("Sending water temperature report");
          Sensor_sendReport(&sensor_temperature);
      }
    }
  }
}


void printVersion(void)
{
  Serial.println(__FILE__ " v" VERSION ", " __DATE__ ", " __TIME__);
}

void printNodeId(void)
{
  byte node_id = zunoNID();

  if (node_id == 0)
  {
    Serial.println("WARN: Not included in any Z-Wave network");
  }
  else
  {
    Serial.print("Z-Wave NodeId: ");
    Serial.println(node_id);
  }
}

void printTempSensorAddress(void)
{
  if (temp_sensor_found)
  {
    Serial.print("DS18B20 address: ");
    printHex(addr1, sizeof(addr1));
    Serial.println();
  }
  else
  {
    Serial.println("ERROR: DS18B20 sensor not found");
  }
}

void printMeterData(void)
{
  Serial.print("Meter data: ticks=");
  Serial.print(meter_data.ticks);
  Serial.print(", crc=");
  printHex(&meter_data.crc8, 1);
}

// Prints 8-bit data in hex with leading zeroes
void printHex(uint8_t* data_p, size_t length)
{
  for (size_t i = 0; i < length; i++)
  {
    if (data_p[i] < 0x10)
    {
      Serial.print('0');
    }
    Serial.print(data_p[i], HEX);
    Serial.print(' ');
  }
}


void meterSetup(void)
{
  // Dry contacts of meters connect to these pins
  pinMode(PIN_SENSOR, INPUT_PULLUP);

  // Get last meter values from EEPROM
  EEPROM.get(EEPROM_ADDR, &meter_data, sizeof(meter_data));

  printMeterData();

  // Check data  
  if (crc_calc((uint8_t*)&meter_data.ticks, sizeof(meter_data) - 1) != meter_data.crc8)
  {
    // Invalid data - reset all
    Serial.println("Bad eeprom crc8 - init meter data");
    meter_data.ticks = 0;
    updateMeterData();
  }
}

void tempSetup(void)
{
  temp_sensor_found = ds18b20.scanAloneSensor(addr1) == 1;
}


void meterCheck(void)
{
    triggered = false;

    // Calculate water flow in l/h
    if (last_pulse_ms > 0)
    {
      uint32_t dt_ms = new_pulse_ms - last_pulse_ms;
      water_flow_lph = (uint16_t)((TICK_VALUE * MS_PER_HOUR) / dt_ms);
      Serial.print("Water flow: ");
      Serial.print(water_flow_lph);
      Serial.println(" l/h");

      if ((millis() - last_water_flow_report_ms) >= SENSOR_MULTILEVEL_REPORT_INTERVAL_MS)
      {
        last_water_flow_report_ms = millis();
        zunoSendReport(ZUNO_CHANNEL_WATER_FLOW);
        Serial.println("Water flow report sent");
      }
    }
    last_pulse_ms = new_pulse_ms;

    // Add tick to water meter
    meter_data.ticks++;
    new_meter_data = true;
    Serial.print("Water meter: ");
    Serial.print(meter_data.ticks * TICK_VALUE);
    Serial.println(" l");
    Serial.println("Sending water meter report");
    zunoSendReport(ZUNO_CHANNEL_WATER_METER);
}


void meterEepromUpdate(void)
{
  // To save EEPROM from a lot of r/w operation 
  // write it once in EEPROM_UPDATE_INTERVAL_MS if data was updated
  if (new_meter_data && (millis() - last_update_ms) > EEPROM_UPDATE_INTERVAL_MS)
  {
    last_update_ms =  millis();

    updateMeterData();
    new_meter_data = false;
    Serial.println("EEPROM updated");
  }
}

int16_t tempMeasure(void)
{
  int16_t tempC10;
  int16_t tempC100 = ds18b20.getTempC100(addr1);
  if (tempC100 == BAD_TEMP)
  {
    tempC10 = BAD_TEMP;
  }
  else
  {
    tempC10 = DIV_ROUND_CLOSEST(tempC100, 10);
  }
  return tempC10;
}

void updateMeterData(void)
{ 
  meter_data.crc8 = crc_calc((uint8_t*)&meter_data.ticks, sizeof(meter_data) - 1);
  printMeterData();
  EEPROM.put(EEPROM_ADDR, &meter_data, sizeof(meter_data));
}

void resetWaterMeter(void)
{
  meter_data.ticks = 0;
  updateMeterData();
  Serial.println("Meter was reset");
}

dword getWaterMeter(void)
{
  return meter_data.ticks * TICK_VALUE;
}

word getWaterFlow(void)
{
  return water_flow_lph;
}

word getWaterTemperature(void)
{
  return sensor_temperature.value.s16;
}
