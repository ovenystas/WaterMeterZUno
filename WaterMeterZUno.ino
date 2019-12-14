/*
 * This is a 3 channel water consumption meter, water flow sensor and 
 * temperature sensor. It uses Z-Uno by Z-Wave.me.
 * 
 * Ove Nystas 2019
 */
 
// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------
#include <EEPROM.h>
#include <ZUNO_DS18B20.h>
#include "Crc.h"
#include "Sensor.h"


// -----------------------------------------------------------------------------
// Defines
// -----------------------------------------------------------------------------
#define VERSION "0.01"

// Z-Uno channels
#define ZUNO_CHANNEL_WATER_METER 1
#define ZUNO_CHANNEL_WATER_FLOW 2
#define ZUNO_CHANNEL_WATER_TEMPERATURE 3

// Pins
#define PIN_SENSOR 17 // Is also INT0
#define PIN_DS18B20 11
#define PIN_PULSE 18

#define EEPROM_ADDR 0x800
#define EEPROM_UPDATE_INTERVAL_ms  120000

#define TICK_VALUE 1 // How many liters each tick is

// Minimum time between reports for sensor multilevel
#define SENSOR_MULTILEVEL_REPORT_INTERVAL_MIN_ms 30000

#define METER_MEASURE_INTERVAL_ms 0 // Interrupt based
#define METER_REPORT_INTERVAL_ms 30000  // Default 900000
#define METER_REPORT_INTERVAL_MIN_ms SENSOR_MULTILEVEL_REPORT_INTERVAL_MIN_ms
#define METER_DIFF_REPORT_l 1 // In liters // TODO: Make this a Z-Wave configurable parameter

#define FLOW_MEASURE_INTERVAL_ms 0 // Interrupt based
#define FLOW_REPORT_INTERVAL_ms 30000  // Default 900
#define FLOW_REPORT_INTERVAL_MIN_ms SENSOR_MULTILEVEL_REPORT_INTERVAL_MIN_ms
#define FLOW_DIFF_REPORT_lph 1 // In l/h // TODO: Make this a Z-Wave configurable parameter

#define TEMPERATURE_MEASURE_INTERVAL_ms 30000
#define TEMPERATURE_REPORT_INTERVAL_ms 30000  // Default 900
#define TEMPERATURE_REPORT_INTERVAL_MIN_ms SENSOR_MULTILEVEL_REPORT_INTERVAL_MIN_ms
#define TEMPERATURE_DIFF_REPORT_dC 1 // In deci degrees C // TODO: Make this a Z-Wave configurable parameter

#define MS_PER_HOUR 3600000UL

#define DIV_ROUND_CLOSEST(n, d) \
  ((((n) < 0) ^ ((d) < 0)) ? (((n) - (d) / 2) / (d)) : (((n) + (d) / 2) / (d)))

typedef enum
{
  ERROR_NONE = 0,
  ERROR_INT_NOT_HANDLED_IN_TIME,
} Error_t;

// -----------------------------------------------------------------------------
// Struct definitions and typedefs
// -----------------------------------------------------------------------------
typedef struct MeterData_s
{
  uint32_t ticks;
  uint8_t  crc8;
} MeterData_t;


// -----------------------------------------------------------------------------
// Variables
// -----------------------------------------------------------------------------
// Error handling
Error_t g_error = ERROR_NONE;
uint16_t g_error_count = 0;


// Water meter
uint32_t g_new_pulse_ms = 0;
bool g_new_meter_data = false;
bool g_pulse_sensor_triggered = false;
Sensor_t sensor_meter =
{
  ZUNO_CHANNEL_WATER_METER,
  METER_MEASURE_INTERVAL_ms,
  METER_REPORT_INTERVAL_ms,
  METER_REPORT_INTERVAL_MIN_ms,
  { METER_DIFF_REPORT_l },
  { 0 } , { 0 },
  0, 0
};


// Water flow
Sensor_t sensor_flow =
{
  ZUNO_CHANNEL_WATER_FLOW,
  FLOW_MEASURE_INTERVAL_ms,
  FLOW_REPORT_INTERVAL_ms,
  FLOW_REPORT_INTERVAL_MIN_ms,
  { FLOW_DIFF_REPORT_lph },
  { 0 } , { 0 },
  0, 0
};


// Water temperature
OneWire ow(PIN_DS18B20);
DS18B20Sensor ds18b20(&ow);
byte addr1[8] = {0xFF};
bool g_temperature_sensor_found = false;
Sensor_t sensor_temperature =
{
  ZUNO_CHANNEL_WATER_TEMPERATURE,
  TEMPERATURE_MEASURE_INTERVAL_ms,
  TEMPERATURE_REPORT_INTERVAL_ms,
  TEMPERATURE_REPORT_INTERVAL_MIN_ms,
  { TEMPERATURE_DIFF_REPORT_dC },
  { 0 } , { 0 },
  0, 0
};


// EEPROM
uint32_t g_last_update_ms = 0;


// -----------------------------------------------------------------------------
// Z-Uno configuration
// -----------------------------------------------------------------------------
ZUNO_SETUP_DEBUG_MODE(DEBUG_ON);
ZUNO_SETUP_SLEEPING_MODE(ZUNO_SLEEPING_MODE_ALWAYS_AWAKE);
ZUNO_SETUP_ISR_INT0(int0_handler);

// Sensor multilevel type for water flow is not yet defined in Z-uno but defined
// in spec v9.
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


// -----------------------------------------------------------------------------
// Interrupt handlers
// -----------------------------------------------------------------------------
void int0_handler()
{
  g_new_pulse_ms = millis();
  if (g_pulse_sensor_triggered)
  {
    g_error = ERROR_INT_NOT_HANDLED_IN_TIME;
  }
  g_pulse_sensor_triggered = true;
  digitalWrite(PIN_PULSE, digitalRead(PIN_SENSOR));
}


// -----------------------------------------------------------------------------
// Main setup
// -----------------------------------------------------------------------------
void setup()
{
  // Setup serial port. Wait for input in 10 seconds.
  Serial.begin(115200);
  delay(10000);

  meterSetup();
  temperatureSetup();

  printVersion();
  printNodeId();
  printTempSensorAddress();
}


// -----------------------------------------------------------------------------
// Setup functions
// -----------------------------------------------------------------------------
void meterSetup(void)
{
  // Dry contacts of meters connect to these pins
  pinMode(PIN_SENSOR, INPUT_PULLUP);
  zunoExtIntMode(ZUNO_EXT_INT0, FALLING);

  // Get last meter values from EEPROM
  MeterData_t meter_data;
  EEPROM.get(EEPROM_ADDR, &meter_data, sizeof(meter_data));
  printMeterData(&meter_data);

  // Check data  
  if (crc_calc((uint8_t*)&meter_data.ticks, sizeof(meter_data) - 1) != meter_data.crc8)
  {
    // Invalid data - reset all
    Serial.println("Bad eeprom crc8 - init meter data");
    updateMeterData(0);
  }
}

void temperatureSetup(void)
{
  g_temperature_sensor_found = ds18b20.scanAloneSensor(addr1) == 1;
}


// -----------------------------------------------------------------------------
// Main loop
// -----------------------------------------------------------------------------
void loop()
{
  errorCheck();

  if (g_pulse_sensor_triggered)
  {
    uint32_t new_pulse_ms = g_new_pulse_ms;
    g_pulse_sensor_triggered = false;

    flowCheck(new_pulse_ms);
    meterCheck(new_pulse_ms);

    printWaterMeter(Sensor_getValueU32(&sensor_meter));
    printWaterFlow(Sensor_getValueU16(&sensor_flow));

    g_new_meter_data = true;
  }

  if (g_temperature_sensor_found)
  {
    temperatureCheck();
  }

  if (g_new_meter_data)
  {
    eepromUpdateCheck();
  }

  if (Sensor_isTimeToSendReport(&sensor_meter))
  {
    Serial.println("Sending water meter report");
    Sensor_sendReport(&sensor_meter);
  }

  if (Sensor_isTimeToSendReport(&sensor_flow))
  {
    Serial.println("Sending water flow report");
    Sensor_sendReport(&sensor_flow);
  }

  if (Sensor_isTimeToSendReport(&sensor_temperature))
  {
    Serial.println("Sending water temperature report");
    Sensor_sendReport(&sensor_temperature);
  }
}


// -----------------------------------------------------------------------------
// Check functions
// -----------------------------------------------------------------------------
void errorCheck(void)
{
  switch (g_error)
  {
    case ERROR_INT_NOT_HANDLED_IN_TIME:
    {
      Serial.println("ERROR: Interrupt not handled in time.");
      break;
    }
    default:
    {
      break;
    }
  }
  
  if (g_error != ERROR_NONE)
  {
    ++g_error_count;
    g_error = ERROR_NONE;
  }
}


void meterCheck(uint32_t new_pulse_ms)
{
  uint32_t ticks = Sensor_getValueU32(&sensor_meter);
  ++ticks;
  Sensor_setValueU(&sensor_meter, ticks);
  Sensor_setMeasuredTime(&sensor_meter, new_pulse_ms);
}


void flowCheck(uint32_t new_pulse_ms)
{
  uint32_t last_pulse_ms = Sensor_getMeasuredTime(&sensor_flow);
  uint16_t flow_lph = flowMeasure(new_pulse_ms, last_pulse_ms);
  Sensor_setValueU(&sensor_flow, flow_lph);
  Sensor_setMeasuredTime(&sensor_flow, new_pulse_ms);
}


void temperatureCheck(void)
{
  if (Sensor_isTimeToMeasure(&sensor_temperature))
  {
    int16_t tempC10 = temperatureMeasure();
 
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
}


// -----------------------------------------------------------------------------
// Measuring functions
// -----------------------------------------------------------------------------
uint16_t flowMeasure(uint32_t new_time, uint32_t last_time)
{
  // Calculate water flow in l/h
  uint16_t flow_lph = 0;
  
  if (last_time > 0)
  {
    uint32_t dt_ms = new_time - last_time;
    flow_lph = (uint16_t)((TICK_VALUE * MS_PER_HOUR) / dt_ms);
  }

  return flow_lph;
}


int16_t temperatureMeasure(void)
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


// -----------------------------------------------------------------------------
// EEPROM functions
// -----------------------------------------------------------------------------
void eepromUpdateCheck(void)
{
  // To save EEPROM from a lot of r/w operation 
  // write it once in EEPROM_UPDATE_INTERVAL_ms if data was updated
  if ((millis() - g_last_update_ms) > EEPROM_UPDATE_INTERVAL_ms)
  {
    g_last_update_ms =  millis();
    g_new_meter_data = false;

    uint32_t ticks = Sensor_getValueU32(&sensor_meter);
    updateMeterData(ticks);
    Serial.println("EEPROM updated");
  }
}


void updateMeterData(uint32_t ticks)
{
  MeterData_t meter_data;
  meter_data.ticks = ticks;
  meter_data.crc8 = crc_calc((uint8_t*)&meter_data.ticks, sizeof(meter_data) - 1);
  printMeterData(&meter_data);
  EEPROM.put(EEPROM_ADDR, &meter_data, sizeof(meter_data));
}


// -----------------------------------------------------------------------------
// Print functions
// -----------------------------------------------------------------------------
void printVersion(void)
{
  Serial.println(__FILE__ " v" VERSION ", " __DATE__ ", " __TIME__);
}


void printNodeId(void)
{
  byte node_id = zunoNID();

  if (node_id != 0)
  {
    Serial.print("Z-Wave NodeId: ");
    Serial.println(node_id);
  }
  else
  {
    Serial.println("WARN: Not included in any Z-Wave network");
  }
}


void printTempSensorAddress(void)
{
  if (g_temperature_sensor_found)
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


void printMeterData(MeterData_t* md_p)
{
  Serial.print("Meter data: ticks=");
  Serial.print(md_p->ticks);
  Serial.print(", crc=");
  printHex(&md_p->crc8, 1);
  Serial.println();
}


void printWaterMeter(uint32_t ticks)
{
  Serial.print("Water meter: ");
  Serial.print(ticks * TICK_VALUE);
  Serial.println(" l");
}


void printWaterFlow(uint16_t flow_lph)
{
  Serial.print("Water flow: ");
  Serial.print(flow_lph);
  Serial.println(" l/h");
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


// -----------------------------------------------------------------------------
// Getters and setters
// -----------------------------------------------------------------------------
void resetWaterMeter(void)
{
  Sensor_setValueU(&sensor_meter, 0);
  updateMeterData(0);
  Serial.println("Meter was reset");
}


dword getWaterMeter(void)
{
  return Sensor_getValueU32(&sensor_meter) * TICK_VALUE; // Unit: l (liter)
}


word getWaterFlow(void)
{
  return Sensor_getValueU16(&sensor_flow); // Unit: l/h (liter per hour)
}


word getWaterTemperature(void)
{
  return Sensor_getValueU16(&sensor_temperature); // Unit dC (deci degree Celsius)
}
