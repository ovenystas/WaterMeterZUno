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

// Uncomment to override default parameter values with the values in the defines.
#define PARAMETER_OVERRIDE

/* Limits of integral types.  */
/* Minimum of signed integral types.  */
# define INT8_MIN                (-128)
# define INT16_MIN                (-32767-1)
# define INT32_MIN                (-2147483647L-1L)
/* Maximum of signed integral types.  */
# define INT8_MAX                (127)
# define INT16_MAX                (32767)
# define INT32_MAX                (2147483647L)
/* Maximum of unsigned integral types.  */
# define UINT8_MAX                (255)
# define UINT16_MAX                (65535)
# define UINT32_MAX                (4294967295UL)

// Z-Uno channels
#define ZUNO_CHANNEL_WATER_METER 1
#define ZUNO_CHANNEL_WATER_FLOW 2
#define ZUNO_CHANNEL_WATER_TEMPERATURE 3

// Pins
#define PIN_SENSOR 17 // Is also INT0
#define PIN_DS18B20 11

#define MY_SERIAL Serial0

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
#define FLOW_REPORT_INTERVAL_ms 30000  // Default 900000
#define FLOW_REPORT_INTERVAL_MIN_ms SENSOR_MULTILEVEL_REPORT_INTERVAL_MIN_ms
#define FLOW_DIFF_REPORT_lph 1 // In l/h // TODO: Make this a Z-Wave configurable parameter

#define TEMPERATURE_MEASURE_INTERVAL_ms 30000
#define TEMPERATURE_REPORT_INTERVAL_ms 30000  // Default 900
#define TEMPERATURE_REPORT_INTERVAL_MIN_ms SENSOR_MULTILEVEL_REPORT_INTERVAL_MIN_ms
#define TEMPERATURE_DIFF_REPORT_dC 1 // In deci degrees C // TODO: Make this a Z-Wave configurable parameter

// Parameter default values
#define PARAM_DEFAULT_METER_REPORT_INTERVAL_s 900
#define PARAM_DEFAULT_METER_DIFF_REPORT_l 1
#define PARAM_DEFAULT_FLOW_REPORT_INTERVAL_s 900
#define PARAM_DEFAULT_FLOW_DIFF_REPORT_lph 1
#define PARAM_DEFAULT_TEMPERATURE_MEASURE_INTERVAL_s 60
#define PARAM_DEFAULT_TEMPERATURE_REPORT_INTERVAL_s 900
#define PARAM_DEFAULT_TEMPERATURE_DIFF_REPORT_dC 5
#define PARAM_DEFAULT_METER_RESET_VALUE_HIGH 0
#define PARAM_DEFAULT_METER_RESET_VALUE_LOW 0


#define MS_PER_HOUR 3600000UL

#define DIV_ROUND_CLOSEST(n, d) \
  ((((n) < 0) ^ ((d) < 0)) ? (((n) - (d) / 2) / (d)) : (((n) + (d) / 2) / (d)))


typedef enum
{
  ERROR_NONE = 0,
  ERROR_INT_NOT_HANDLED_IN_TIME,
  ERROR_INVALID_PARAMETER_NUMBER,
} Error_E;


typedef enum
{
  PARAM_BASE = 64,
  PARAM_METER_REPORT_INTERVAL_s = 0,
  PARAM_METER_DIFF_REPORT_l,
  PARAM_FLOW_REPORT_INTERVAL_s,
  PARAM_FLOW_DIFF_REPORT_lph,
  PARAM_TEMPERATURE_MEASURE_INTERVAL_s,
  PARAM_TEMPERATURE_REPORT_INTERVAL_s,
  PARAM_TEMPERATURE_DIFF_REPORT_dC,
  PARAM_METER_RESET_VALUE_HIGH,
  PARAM_METER_RESET_VALUE_LOW,
  PARAM_LENGTH,
  PARAM_MAX = 96,
} Parameter_E;


// -----------------------------------------------------------------------------
// Struct definitions and typedefs
// -----------------------------------------------------------------------------
typedef struct MeterData_s
{
  uint32_t ticks;
  uint8_t  crc8;
} MeterData_T;


// -----------------------------------------------------------------------------
// Variables
// -----------------------------------------------------------------------------
// Error handling
Error_E g_error = ERROR_NONE;
uint16_t g_error_count = 0;


// Water meter
uint32_t g_new_pulse_ms = 0;
bool g_new_meter_data = false;
bool g_pulse_sensor_triggered = false;
Sensor_T sensor_meter =
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
Sensor_T sensor_flow =
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
Sensor_T sensor_temperature =
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


// Parameters
uint16_t parameter[PARAM_LENGTH];
const uint16_t parameter_default[PARAM_LENGTH] =
{
  PARAM_DEFAULT_METER_REPORT_INTERVAL_s,
  PARAM_DEFAULT_METER_DIFF_REPORT_l,
  PARAM_DEFAULT_FLOW_REPORT_INTERVAL_s,
  PARAM_DEFAULT_FLOW_DIFF_REPORT_lph,
  PARAM_DEFAULT_TEMPERATURE_MEASURE_INTERVAL_s,
  PARAM_DEFAULT_TEMPERATURE_REPORT_INTERVAL_s,
  PARAM_DEFAULT_TEMPERATURE_DIFF_REPORT_dC,
  PARAM_DEFAULT_METER_RESET_VALUE_HIGH,
  PARAM_DEFAULT_METER_RESET_VALUE_LOW,
};

// -----------------------------------------------------------------------------
// Z-Uno configuration
// -----------------------------------------------------------------------------
//ZUNO_SETUP_PRODUCT_ID(0xAA, 0xBB); // To set 0x0111 / 0xAABB

ZUNO_SETUP_DEBUG_MODE(DEBUG_ON);
ZUNO_SETUP_SLEEPING_MODE(ZUNO_SLEEPING_MODE_ALWAYS_AWAKE);
ZUNO_SETUP_ISR_INT0(int0_handler);
ZUNO_SETUP_CFGPARAMETER_HANDLER(config_parameter_changed);

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
}


// -----------------------------------------------------------------------------
// Main setup
// -----------------------------------------------------------------------------
void setup()
{
  // Setup serial port. Wait for input in 10 seconds.
  MY_SERIAL.begin(115200);
  delay(10000);

  parameterSetup();
  meterSetup();
  temperatureSetup();

  printVersion();
  printNodeId();
  printTempSensorAddress();
  printParameters();
}


// -----------------------------------------------------------------------------
// Setup functions
// -----------------------------------------------------------------------------
/*
 * Load all Z-Wave device parameters from EEPROM if in network else load
 * from default values and also save in EEPROM.
 */
void parameterSetup(void)
{
  bool inNetwork = zunoInNetwork();
  
  if (inNetwork)
  {
    parameterLoad();
  }
  else
  {
    parameterSetDefault();
  }

#ifndef PARAMETER_OVERRIDE
  parameterCommit();
#endif
}


/*
 * Load all parameter values from EEPROM.
 */
void parameterLoad(void)
{
  for (uint8_t i = 0; i < PARAM_LENGTH; i++)
  {
    parameter[i] = zunoLoadCFGParam(PARAM_BASE + i);
  }
}


/*
 * Set all parameter values to default and also save in EEPROM.
 */
void parameterSetDefault(void)
{
  for (uint8_t i = 0; i < PARAM_LENGTH; i++)
  {
    parameter[i] = parameter_default[i];
    zunoSaveCFGParam(PARAM_BASE + i, parameter[i]);
  }
}


/*
 * Commit parameter values to be used with the sensors.
 */
void parameterCommit(void)
{
  sensor_meter.report_interval_ms = parameter[PARAM_METER_REPORT_INTERVAL_s] * 1000;
  sensor_meter.report_threshold.u16 = parameter[PARAM_METER_DIFF_REPORT_l];

  sensor_flow.report_interval_ms = parameter[PARAM_FLOW_REPORT_INTERVAL_s] * 1000;
  sensor_flow.report_threshold.u16 = parameter[PARAM_FLOW_DIFF_REPORT_lph];

  sensor_temperature.measure_interval_ms = parameter[PARAM_TEMPERATURE_MEASURE_INTERVAL_s] * 1000;
  sensor_temperature.report_interval_ms = parameter[PARAM_TEMPERATURE_REPORT_INTERVAL_s] * 1000;
  sensor_temperature.report_threshold.u16 = parameter[PARAM_TEMPERATURE_DIFF_REPORT_dC];
}


/*
 * Setup inductive sensor for reading water meter dial. 
 * Load previous stored water meter tick value from EEPROM.
 */
void meterSetup(void)
{
  // Dry contacts of meters connect to these pins
  pinMode(PIN_SENSOR, INPUT_PULLUP);
  zunoExtIntMode(ZUNO_EXT_INT0, FALLING);

  // Get last meter values from EEPROM
  MeterData_T meter_data;
  EEPROM.get(EEPROM_ADDR, &meter_data, sizeof(meter_data));
  printMeterData(&meter_data);

  // Check data  
  if (crc_calc((uint8_t*)&meter_data.ticks, sizeof(meter_data) - 1) != meter_data.crc8)
  {
    // Invalid data - reset all
    MY_SERIAL.println("Bad eeprom crc8 - init meter data");
    updateMeterData(0);
  }
}

/*
 * Setup temperature sensor.
 * Scan for one sensor on the 1-wire bus and store its address.
 * Sets global variable g_temperature_sensor_found.
 */
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
  else
  {
    flowDecay();
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
    MY_SERIAL.println("Sending water meter report");
    Sensor_sendReport(&sensor_meter);
  }

  if (Sensor_isTimeToSendReport(&sensor_flow))
  {
    MY_SERIAL.println("Sending water flow report");
    Sensor_sendReport(&sensor_flow);
  }

  if (Sensor_isTimeToSendReport(&sensor_temperature))
  {
    MY_SERIAL.println("Sending water temperature report");
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
      MY_SERIAL.println("ERROR: Interrupt not handled in time.");
      break;

    case ERROR_INVALID_PARAMETER_NUMBER:
      MY_SERIAL.println("ERROR: Invalid parameter number.");
      break;

    default:
      break;
  }
  
  if (g_error != ERROR_NONE)
  {
    ++g_error_count;
    MY_SERIAL.print("INFO: Total number of errors: ");
    MY_SERIAL.println(g_error_count);
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


void flowDecay(void)
{
  uint32_t now_ms = millis();
  uint32_t last_pulse_ms = Sensor_getMeasuredTime(&sensor_flow);
  uint16_t flow_lph = flowMeasure(now_ms, last_pulse_ms);
  if (flow_lph < Sensor_getValueU16(&sensor_flow))
  {
    Sensor_setValueU(&sensor_flow, flow_lph);
    printWaterFlow(Sensor_getValueU16(&sensor_flow));
  }
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
      MY_SERIAL.println("ERROR: Invalid temperature");
    }
    else
    {
      MY_SERIAL.print("TempC10: ");
      MY_SERIAL.println(tempC10);
    }
  }
}


// -----------------------------------------------------------------------------
// Measuring functions
// -----------------------------------------------------------------------------
/*
 * Measure water flow value in liter per hour based on time between last two
 * pulses.
 */
uint16_t flowMeasure(uint32_t new_time, uint32_t last_time)
{
  // Calculate water flow in l/h
  uint32_t flow_lph = 0;
  
  if (last_time > 0 && new_time > last_time)
  {
    uint32_t dt_ms = new_time - last_time;
    flow_lph = ((TICK_VALUE * MS_PER_HOUR) / dt_ms);
  }

  return flow_lph > UINT16_MAX ? UINT16_MAX : (uint16_t)flow_lph;
}


/*
 * Measure temperature by retreiving value from DS18B20 sensor.
 * Round value to deci degree Celsius.
 */
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
    MY_SERIAL.println("EEPROM updated");
  }
}


/*
 * Calculate CRC on ticks value and store them in EEPROM.
 */
void updateMeterData(uint32_t ticks)
{
  MeterData_T meter_data;
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
  MY_SERIAL.println(__FILE__ " v" VERSION ", " __DATE__ ", " __TIME__);
}


void printNodeId(void)
{
  byte node_id = zunoNID();

  if (node_id != 0)
  {
    MY_SERIAL.print("Z-Wave NodeId: ");
    MY_SERIAL.println(node_id);
  }
  else
  {
    MY_SERIAL.println("WARN: Not included in any Z-Wave network");
  }
}


void printTempSensorAddress(void)
{
  if (g_temperature_sensor_found)
  {
    MY_SERIAL.print("DS18B20 address: ");
    printHex(addr1, sizeof(addr1));
    MY_SERIAL.println();
  }
  else
  {
    MY_SERIAL.println("ERROR: DS18B20 sensor not found");
  }
}


void printParameters(void)
{
  MY_SERIAL.print("Parameters: ");
  for (uint8_t i = 0; i < PARAM_LENGTH; i++)
  {
    MY_SERIAL.print(PARAM_BASE + i);
    MY_SERIAL.print("=");
    MY_SERIAL.print(parameter[i]);
    MY_SERIAL.print(" ");
  }
  MY_SERIAL.println();
}


void printMeterData(MeterData_T* md_p)
{
  MY_SERIAL.print("Meter data: ticks=");
  MY_SERIAL.print(md_p->ticks);
  MY_SERIAL.print(", crc=");
  printHex(&md_p->crc8, 1);
  MY_SERIAL.println();
}


void printWaterMeter(uint32_t ticks)
{
  MY_SERIAL.print("Water meter: ");
  MY_SERIAL.print(ticks * TICK_VALUE);
  MY_SERIAL.println(" l");
}


void printWaterFlow(uint16_t flow_lph)
{
  MY_SERIAL.print("Water flow: ");
  MY_SERIAL.print(flow_lph);
  MY_SERIAL.println(" l/h");
}


/* 
 * Prints 8-bit data in hex with leading zeroes.
 */
void printHex(uint8_t* data_p, size_t length)
{
  for (size_t i = 0; i < length; i++)
  {
    if (data_p[i] < 0x10)
    {
      MY_SERIAL.print('0');
    }
    MY_SERIAL.print(data_p[i], HEX);
    MY_SERIAL.print(' ');
  }
}


// -----------------------------------------------------------------------------
// Z-Uno getters and setters
// -----------------------------------------------------------------------------
void resetWaterMeter(void)
{
  uint32_t resetValue = (uint32_t)parameter[PARAM_METER_RESET_VALUE_HIGH] << 16;
  resetValue |= (uint32_t)parameter[PARAM_METER_RESET_VALUE_LOW];

  Sensor_setValueU(&sensor_meter, resetValue);
  updateMeterData(resetValue);
  MY_SERIAL.print("INFO: Meter was reset to value ");
  MY_SERIAL.println(resetValue);
}


uint32_t getWaterMeter(void)
{
  return Sensor_getValueU32(&sensor_meter) * TICK_VALUE; // Unit: l (liter)
}


uint16_t getWaterFlow(void)
{
  return Sensor_getValueU16(&sensor_flow); // Unit: l/h (liter per hour)
}


uint16_t getWaterTemperature(void)
{
  return Sensor_getValueU16(&sensor_temperature); // Unit dC (deci degree Celsius)
}


// -----------------------------------------------------------------------------
// Z-Uno configuration parameter handler
// -----------------------------------------------------------------------------
void config_parameter_changed(uint8_t param, uint16_t value)
{
  if (param < PARAM_BASE || param >= (PARAM_BASE + PARAM_LENGTH))
  {
    MY_SERIAL.print("ERROR: Invalid parameter number=");
    MY_SERIAL.println(param);
    g_error = ERROR_INVALID_PARAMETER_NUMBER;
    return;
  }

  parameter[param - PARAM_BASE] = value;
  zunoSaveCFGParam(param, value);
  parameterCommit();

  MY_SERIAL.print("INFO: Parameter ");
  MY_SERIAL.print(param);
  MY_SERIAL.print(" changed to ");
  MY_SERIAL.println(value);
}
