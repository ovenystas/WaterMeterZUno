/*
 * This is a simple 2 channel meter example
 * (c) Z-Wave.Me 2016
 * 
 * Changed to triple channel and changed crc algorithm
 * by Ove Nystas 2018-2019
 */
 
#include <EEPROM.h>
#include <ZUNO_DS18B20.h>
#include "crc.h"

#define VERSION "0.01"

// channel numbers
#define ZUNO_CHANNEL_WATER_METER 1
#define ZUNO_CHANNEL_WATER_FLOW 2
#define ZUNO_CHANNEL_WATER_TEMPERATURE 3

#define PIN_SENSOR 0
#define PIN_DS18B20 11

#define EEPROM_ADDR 0x800
#define EEPROM_UPDATE_INTERVAL_MS  120000

#define TICK_VALUE 1 // in liters

#define METER_CHECK_INTERVAL_MS 100
#define TEMP_CHECK_INTERVAL_MS 30000
#define SENSOR_MULTILEVEL_REPORT_INTERVAL 30000

// TODO: Make this a Z-Wave configurable parameter
#define TEMP_DIFF_REPORT 2 // In degree * 10

#define MS_PER_HOUR 3600000UL

#define DIV_ROUND_CLOSEST(n, d) ((((n) < 0) ^ ((d) < 0)) ? (((n) - (d)/2)/(d)) : (((n) + (d)/2)/(d)))


typedef struct meter_data
{
  uint32_t ticks;
  uint8_t  crc8;
} MeterData_t;


// Variables for water meter
MeterData_t meter_data = {0};
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
int water_temperature;
int temp_reported;
bool temp_sensor_found = false;
uint32_t last_run_temp_ms = 0;

// Configure Z-Uno
ZUNO_SETUP_DEBUG_MODE(DEBUG_ON);

ZUNO_SETUP_SLEEPING_MODE(ZUNO_SLEEPING_MODE_ALWAYS_AWAKE);

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


// For some cases use UART (Serial0/Serial1)
// It's a most comfortable way for debugging
// By default we use built-in USB CDC (Serial)
#define SERIAL Serial


void setup()
{
  // Setup serial port. Wait for input in 10 seconds.
  //byte tmp;
  SERIAL.begin(115200);
  //SERIAL.setTimeout(10000);
  //size_t bytesRead = SERIAL.readBytes(&tmp, 1);
  delay(10000);

  meterSetup();
  tempSetup();

//  if (bytesRead > 0)
//  {
    printVersion();
    printNodeId();
    printTempSensorAddress();
//  }
//  else
//  {
//    SERIAL.println("Timeout");
//  }
}


void loop()
{
  meterCheck();
  meterEepromUpdate();

  tempCheck();
}


void printVersion(void)
{
  SERIAL.println(__FILE__ " v" VERSION ", " __DATE__ ", " __TIME__);
}

void printNodeId(void)
{
  byte node_id = zunoNID();

  if (node_id == 0)
  {
    SERIAL.println("WARN: Not included in any Z-Wave network");
  }
  else
  {
    SERIAL.print("Z-Wave NodeId: ");
    SERIAL.println(node_id);
  }
}

void printTempSensorAddress(void)
{
  if (temp_sensor_found)
  {
    SERIAL.print("DS18B20 address: ");
    printHex(addr1, sizeof(addr1));
    SERIAL.println();
  }
  else
  {
    SERIAL.println("ERROR: DS18B20 sensor not found");
  }
}

void printMeterData(void)
{
  SERIAL.print("Meter data: ticks=");
  SERIAL.print(meter_data.ticks);
  SERIAL.print(", crc=");
  printHex(&meter_data.crc8, 1);
}

// Prints 8-bit data in hex with leading zeroes
void printHex(uint8_t* data_p, uint8_t length)
{
  for (uint8_t i = 0; i < length; i++)
  {
    if (data_p[i] < 0x10)
    {
      SERIAL.print('0');
    }
    SERIAL.print(data_p[i], HEX);
    SERIAL.print(' ');
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
  if (crc_calc((uint8_t*)&meter_data.ticks, sizeof(meter_data.ticks)) != meter_data.crc8)
  {
    // Invalid data - reset all
    SERIAL.println("Bad eeprom crc8 - init meter data");
    meter_data.ticks = 0;
    updateMeterData();
  }
}

void tempSetup(void)
{
  temp_sensor_found = ds18b20.scanAloneSensor(addr1) == 1;
}


// TODO: Change to interrupt for meter sensor pin
void meterCheck(void)
{
  if ((millis() - last_run_meter_ms) < METER_CHECK_INTERVAL_MS)
  {
    return;
  }

  last_run_meter_ms = millis();

  if (digitalRead(PIN_SENSOR) == LOW)
  {
    if (!triggered)
    {
      triggered = true;

      // Calculate water flow in l/h
      uint32_t new_pulse_ms = millis();
      if (last_pulse_ms > 0)
      {
        uint32_t dt_ms = new_pulse_ms - last_pulse_ms;
        water_flow_lph = (uint16_t)((TICK_VALUE * MS_PER_HOUR) / dt_ms);
        SERIAL.print("Water flow: ");
        SERIAL.print(water_flow_lph);
        SERIAL.println(" l/h");
        if ((millis() - last_water_flow_report_ms) >= SENSOR_MULTILEVEL_REPORT_INTERVAL)
        {
          last_water_flow_report_ms = millis();
          zunoSendReport(ZUNO_CHANNEL_WATER_FLOW);
          SERIAL.println("Water flow report sent");
        }
      }
      last_pulse_ms = new_pulse_ms;

      // Add tick to water meter
      meter_data.ticks++;
      new_meter_data = true;
      SERIAL.print("Water meter: ");
      SERIAL.print(meter_data.ticks * TICK_VALUE);
      SERIAL.println(" l");
      zunoSendReport(ZUNO_CHANNEL_WATER_METER);
      SERIAL.println("Water meter report sent");
    }
  }
  else
  {
    triggered = false;
  }
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
    SERIAL.println("EEPROM updated");
  }
}

void tempCheck(void)
{
  if (!temp_sensor_found || (millis() - last_run_temp_ms) < TEMP_CHECK_INTERVAL_MS)
  {
    return;
  }

  last_run_temp_ms = millis();

  int tempC100 = ds18b20.getTempC100(addr1);
  if (tempC100 != BAD_TEMP)
  {
    water_temperature = DIV_ROUND_CLOSEST(tempC100, 10);

    SERIAL.print("TempC100: ");
    SERIAL.print(tempC100);
    SERIAL.print(", Temp: ");
    SERIAL.println(water_temperature);

    if (abs(water_temperature - temp_reported) >= TEMP_DIFF_REPORT)
    {
      zunoSendReport(ZUNO_CHANNEL_WATER_TEMPERATURE);
      temp_reported = water_temperature;
      SERIAL.println("Water temperature report sent");
    }
  }
  else
  {
    SERIAL.println("ERROR: Invalid temperature");
  }
}

void updateMeterData(void)
{ 
  meter_data.crc8 = crc_calc((uint8_t*)&meter_data.ticks, sizeof(meter_data.ticks));
  printMeterData();
  EEPROM.put(EEPROM_ADDR, &meter_data, sizeof(meter_data));
}

void resetWaterMeter(void)
{
  meter_data.ticks = 0;
  updateMeterData();
  SERIAL.println("Meter was reset");
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
  return water_temperature;
}
