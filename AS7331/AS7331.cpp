#include "AS7331.h"

AS7331::AS7331(I2Cdev *i2cBus, uint8_t address) :
  _i2cBus(i2cBus), _address(address), _healthy(true)
{
}


bool AS7331::result(bool success)
{
  if(!success) _healthy = false;
  return success;
}


bool AS7331::getChipID(uint8_t *chipID)
{
  return result(_i2cBus->readByte(_address, AS7331_AGEN, chipID));
}


bool AS7331::readOperatingState(uint8_t *operatingState)
{
  return result(_i2cBus->readByte(_address, AS7331_OSR, operatingState));
}


bool AS7331::softwareReset()
{
  // Datasheet OSR programming example: 0x0A requests software reset while the
  // DOS field explicitly selects configuration state.
  return result(_i2cBus->writeByte(_address, AS7331_OSR,
                                   AS7331_OSR_SOFTWARE_RESET |
                                   AS7331_OSR_CONFIGURATION));
}


bool AS7331::enterConfiguration()
{
  return result(_i2cBus->writeByte(_address, AS7331_OSR,
                                   AS7331_OSR_CONFIGURATION));
}


bool AS7331::enterMeasurement(bool startMeasurement)
{
  uint8_t command = AS7331_OSR_MEASUREMENT;
  if(startMeasurement) command |= AS7331_OSR_START;
  return result(_i2cBus->writeByte(_address, AS7331_OSR, command));
}


bool AS7331::powerDown()
{
  // Datasheet OSR programming example: 0x42 selects configuration state with
  // power-down switched on.
  return result(_i2cBus->writeByte(_address, AS7331_OSR,
                                   AS7331_OSR_POWER_DOWN |
                                   AS7331_OSR_CONFIGURATION));
}


bool AS7331::powerUp()
{
  // PD = 0 disables power-down. The sensor remains in configuration state.
  return enterConfiguration();
}


bool AS7331::configure(AS7331MeasurementMode mode,
                       AS7331ConversionClock clock,
                       bool standbyEnabled,
                       uint8_t breakTime,
                       uint8_t gain,
                       uint8_t integrationTime)
{
  if(gain > 11 || integrationTime > 15) return result(false);

  uint8_t creg1 = (uint8_t)((gain << 4) | integrationTime);
  uint8_t creg3 = (uint8_t)(((uint8_t)mode << 6) |
                            (standbyEnabled ? 0x10 : 0x00) |
                            (uint8_t)clock);

  bool success = _i2cBus->writeByte(_address, AS7331_CREG1, creg1);
  success = _i2cBus->writeByte(_address, AS7331_CREG3, creg3) && success;
  success = _i2cBus->writeByte(_address, AS7331_BREAK, breakTime) && success;
  return result(success);
}


bool AS7331::startOneShot()
{
  // 0xC3 enters measurement state, starts one CMD conversion, and keeps PD
  // enabled so the sensor automatically returns to power-down when finished.
  return result(_i2cBus->writeByte(_address, AS7331_OSR,
                                   AS7331_OSR_START |
                                   AS7331_OSR_POWER_DOWN |
                                   AS7331_OSR_MEASUREMENT));
}


bool AS7331::readStatus(uint16_t *status)
{
  if(status == nullptr) return result(false);

  uint8_t raw[2] = {0, 0};
  if(!result(_i2cBus->readBytes(_address, AS7331_STATUS, 2, raw))) return false;

  // Address 0 is special: the first byte is OSR and the second byte is STATUS.
  // Keep OSR in the high byte so STATUS masks naturally apply to the low byte.
  *status = (uint16_t)(((uint16_t)raw[0] << 8) | raw[1]);
  return true;
}


bool AS7331::readAllData(AS7331Data *data)
{
  if(data == nullptr) return result(false);

  uint8_t raw[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  if(!result(_i2cBus->readBytes(_address, AS7331_TEMP, 8, raw))) return false;

  // Update the caller's data only after the complete eight-byte burst succeeds.
  AS7331Data newData;
  newData.temperature = (uint16_t)(((uint16_t)raw[1] << 8) | raw[0]);
  newData.uva         = (uint16_t)(((uint16_t)raw[3] << 8) | raw[2]);
  newData.uvb         = (uint16_t)(((uint16_t)raw[5] << 8) | raw[4]);
  newData.uvc         = (uint16_t)(((uint16_t)raw[7] << 8) | raw[6]);
  *data = newData;
  return true;
}


bool AS7331::healthy() const
{
  return _healthy && _i2cBus->healthy();
}


void AS7331::clearHealth()
{
  // Bus recovery is deliberately controlled by the main sketch.
  _healthy = true;
}
