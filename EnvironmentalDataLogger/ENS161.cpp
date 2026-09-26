/*
 * Lightweight, non-blocking ScioSense ENS161 driver for the Sasquatch
 * daughter-board environmental monitor.
 */

#include "ENS161.h"

// DEVICE_STATUS bit masks.
static const uint8_t ENS161_STATUS_ACTIVE   = 0x80;
static const uint8_t ENS161_STATUS_ERROR    = 0x40;
static const uint8_t ENS161_STATUS_VALIDITY = 0x0C;
static const uint8_t ENS161_STATUS_NEW_DATA = 0x02;

// Active-low, push-pull interrupt asserted for new processed output data.
// CONFIG bit 5 selects push-pull, bit 1 selects new DATA_x data, and bit 0
// enables the interrupt. Push-pull is required because the daughter-board D7
// interrupt connection has no external pull-up resistor.
static const uint8_t ENS161_DATA_INTERRUPT_CONFIG = 0x23;

// The datasheet requires 10 ms after RESET before normal communication.
static const uint32_t ENS161_RESET_TIME_MS = 10;


ENS161::ENS161(I2Cdev* i2c_bus, uint8_t address)
{
  _i2c_bus = i2c_bus;
  _address = address;
  _requestedMode = ENS161_LOW_POWER_MODE;
  _initializationState = INIT_IDLE;
  _readyAt = 0;
}


/* --------------------------------------------------------------------------
 * Non-blocking initialization
 * -------------------------------------------------------------------------- */

bool ENS161::startInitialization(ENS161Mode mode)
{
  if(_i2c_bus == nullptr) return false;

  // Accept the three gas-sensing modes plus deep sleep for a verified
  // power-audit shutdown. IDLE alone is not a completed initialization.
  if(mode != ENS161_DEEP_SLEEP_MODE &&
     mode != ENS161_STANDARD_MODE &&
     mode != ENS161_LOW_POWER_MODE &&
     mode != ENS161_ULTRA_LOW_POWER_MODE) return false;

  _requestedMode = mode;

  if(!writeOperatingMode(ENS161_RESET_MODE)) {
    failInitialization();
    return false;
  }

  _readyAt = millis() + ENS161_RESET_TIME_MS;
  _initializationState = INIT_WAIT_RESET;
  return true;
}


bool ENS161::serviceInitialization()
{
  switch(_initializationState)
  {
    case INIT_WAIT_RESET:
      // Signed subtraction preserves correct behavior across millis() wrap.
      if((int32_t)(millis() - _readyAt) < 0) return true;
      _initializationState = INIT_SET_IDLE;
      return true;

    case INIT_SET_IDLE:
      if(!writeOperatingMode(ENS161_IDLE_MODE)) {
        failInitialization();
        return false;
      }
      _initializationState = INIT_READ_PART_ID;
      return true;

    case INIT_READ_PART_ID:
    {
      uint16_t partID = 0;
      if(!getPartID(&partID) || partID != ENS161_EXPECTED_PART_ID) {
        failInitialization();
        return false;
      }
      // A sleeping sensor needs no data-ready interrupt configuration.
      _initializationState = (_requestedMode == ENS161_DEEP_SLEEP_MODE) ?
                             INIT_START_MEASUREMENT : INIT_CONFIGURE_INTERRUPT;
      return true;
    }

    case INIT_CONFIGURE_INTERRUPT:
      if(!configureDataReadyInterrupt()) {
        failInitialization();
        return false;
      }
      _initializationState = INIT_START_MEASUREMENT;
      return true;

    case INIT_START_MEASUREMENT:
      if(!writeOperatingMode(_requestedMode)) {
        failInitialization();
        return false;
      }
      _initializationState = INIT_VERIFY_MODE;
      return true;

    case INIT_VERIFY_MODE:
    {
      uint8_t mode = 0;
      if(!getOperatingMode(&mode) || mode != (uint8_t)_requestedMode) {
        failInitialization();
        return false;
      }
      _initializationState = INIT_COMPLETE;
      return true;
    }

    case INIT_IDLE:
    case INIT_COMPLETE:
      return true;

    case INIT_FAILED:
    default:
      return false;
  }
}


bool ENS161::initializationComplete() const
{
  return _initializationState == INIT_COMPLETE;
}


bool ENS161::initializationFailed() const
{
  return _initializationState == INIT_FAILED;
}


void ENS161::failInitialization()
{
  _initializationState = INIT_FAILED;
}


/* --------------------------------------------------------------------------
 * Register access and configuration
 * -------------------------------------------------------------------------- */

bool ENS161::writeOperatingMode(ENS161Mode mode)
{
  return _i2c_bus != nullptr &&
         _i2c_bus->writeByte(_address, ENS161_OPMODE, (uint8_t)mode);
}


bool ENS161::getPartID(uint16_t *partID)
{
  if(_i2c_bus == nullptr || partID == nullptr) return false;

  uint8_t rawData[2] = {0, 0};
  if(!_i2c_bus->readBytes(_address, ENS161_PART_ID, 2, rawData)) return false;

  // ENS161 multi-byte registers are little-endian.
  *partID = ((uint16_t)rawData[1] << 8) | rawData[0];
  return true;
}


bool ENS161::getOperatingMode(uint8_t *mode)
{
  if(_i2c_bus == nullptr || mode == nullptr) return false;
  return _i2c_bus->readByte(_address, ENS161_OPMODE, mode);
}


bool ENS161::getStatus(uint8_t *status)
{
  if(_i2c_bus == nullptr || status == nullptr) return false;
  return _i2c_bus->readByte(_address, ENS161_DEVICE_STATUS, status);
}


bool ENS161::dataReady(bool *ready)
{
  if(ready == nullptr) return false;

  uint8_t status = 0;
  if(!getStatus(&status)) return false;

  *ready = (status & ENS161_STATUS_NEW_DATA) != 0;
  return true;
}


bool ENS161::deepSleep()
{
  if(!writeOperatingMode(ENS161_DEEP_SLEEP_MODE)) return false;
  uint8_t mode = 0;
  return getOperatingMode(&mode) && mode == ENS161_DEEP_SLEEP_MODE;
}


bool ENS161::configureDataReadyInterrupt()
{
  if(_i2c_bus == nullptr) return false;
  if(!_i2c_bus->writeByte(_address, ENS161_CONFIG,
                          ENS161_DATA_INTERRUPT_CONFIG)) return false;

  uint8_t config = 0;
  return _i2c_bus->readByte(_address, ENS161_CONFIG, &config) &&
         config == ENS161_DATA_INTERRUPT_CONFIG;
}


bool ENS161::writeCompensation(float temperatureC, float relativeHumidity)
{
  if(_i2c_bus == nullptr) return false;

  // Bound inputs to the ENS161's documented environmental operating range.
  if(temperatureC < -40.0f) temperatureC = -40.0f;
  if(temperatureC >  85.0f) temperatureC =  85.0f;
  if(relativeHumidity <  0.0f) relativeHumidity =  0.0f;
  if(relativeHumidity > 95.0f) relativeHumidity = 95.0f;

  // TEMP_IN = Kelvin * 64; RH_IN = percent RH * 512.
  uint16_t rawTemperature = (uint16_t)(((temperatureC + 273.15f) * 64.0f) + 0.5f);
  uint16_t rawHumidity = (uint16_t)((relativeHumidity * 512.0f) + 0.5f);

  // Both registers are contiguous and little-endian.  Writing all four bytes
  // in one transaction updates each value when its MSB is received.
  uint8_t compensation[4] = {
    (uint8_t)rawTemperature,
    (uint8_t)(rawTemperature >> 8),
    (uint8_t)rawHumidity,
    (uint8_t)(rawHumidity >> 8)
  };

  return _i2c_bus->writeBytes(_address, ENS161_TEMP_IN, 4, compensation);
}


/* --------------------------------------------------------------------------
 * Processed measurement readout
 * -------------------------------------------------------------------------- */

bool ENS161::readData(ENS161Data *data, bool *sampleValid)
{
  if(_i2c_bus == nullptr || data == nullptr || sampleValid == nullptr) return false;

  *sampleValid = false;

  uint8_t status = 0;
  if(!getStatus(&status)) return false;

  // Read AQI-UBA, TVOC, eCO2 and AQI-S as one coherent register block.
  uint8_t rawData[7] = {0, 0, 0, 0, 0, 0, 0};
  if(!_i2c_bus->readBytes(_address, ENS161_DATA_AQI_UBA, 7, rawData)) return false;

  ENS161Data newData;
  newData.status = status;
  newData.aqiUBA = rawData[0] & 0x07;
  newData.tvoc = ((uint16_t)rawData[2] << 8) | rawData[1];
  newData.eco2 = ((uint16_t)rawData[4] << 8) | rawData[3];
  newData.aqiScioSense = ((uint16_t)rawData[6] << 8) | rawData[5];
  newData.validity = (ENS161Validity)((status & ENS161_STATUS_VALIDITY) >> 2);

  // Update caller data only after both I2C transactions completed.
  *data = newData;

  bool active = (status & ENS161_STATUS_ACTIVE) != 0;
  bool error = (status & ENS161_STATUS_ERROR) != 0;
  bool fresh = (status & ENS161_STATUS_NEW_DATA) != 0;

  *sampleValid = active && !error && fresh &&
                 newData.validity == ENS161_OUTPUT_NORMAL;
  return true;
}
