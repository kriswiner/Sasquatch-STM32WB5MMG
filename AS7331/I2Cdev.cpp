#include "I2Cdev.h"

I2Cdev::I2Cdev(TwoWire *i2cBus) : _i2cBus(i2cBus), _healthy(true)
{
}


bool I2Cdev::readByte(uint8_t address, uint8_t registerAddress, uint8_t *destination)
{
  if(destination == nullptr) {
    _healthy = false;
    return false;
  }

  _i2cBus->beginTransmission(address);
  bool buffered = (_i2cBus->write(registerAddress) == 1);
  uint8_t error = _i2cBus->endTransmission(false); // Repeated START for read
  if(!buffered || error != 0) {
    _healthy = false;
    return false;
  }

  uint8_t received = _i2cBus->requestFrom(address, (uint8_t)1);
  if(received != 1 || !_i2cBus->available()) {
    while(_i2cBus->available()) _i2cBus->read();
    _healthy = false;
    return false;
  }

  *destination = (uint8_t)_i2cBus->read();
  while(_i2cBus->available()) _i2cBus->read();
  return true;
}


bool I2Cdev::readBytes(uint8_t address, uint8_t registerAddress,
                       uint8_t count, uint8_t *destination)
{
  if(destination == nullptr || count == 0) {
    _healthy = false;
    return false;
  }

  _i2cBus->beginTransmission(address);
  bool buffered = (_i2cBus->write(registerAddress) == 1);
  uint8_t error = _i2cBus->endTransmission(false); // Repeated START for read
  if(!buffered || error != 0) {
    _healthy = false;
    return false;
  }

  uint8_t received = _i2cBus->requestFrom(address, count);
  uint8_t copied = 0;
  while(_i2cBus->available() && copied < count) {
    destination[copied++] = (uint8_t)_i2cBus->read();
  }
  while(_i2cBus->available()) _i2cBus->read();

  bool success = (received == count && copied == count);
  if(!success) _healthy = false;
  return success;
}


bool I2Cdev::writeByte(uint8_t address, uint8_t registerAddress, uint8_t data)
{
  _i2cBus->beginTransmission(address);
  bool buffered = (_i2cBus->write(registerAddress) == 1);
  buffered &= (_i2cBus->write(data) == 1);
  uint8_t error = _i2cBus->endTransmission();

  bool success = buffered && (error == 0);
  if(!success) _healthy = false;
  return success;
}


bool I2Cdev::probe(uint8_t address)
{
  // A missing device during a diagnostic probe is not a bus-driver fault.
  _i2cBus->beginTransmission(address);
  return (_i2cBus->endTransmission() == 0);
}


bool I2Cdev::healthy() const
{
  return _healthy;
}


void I2Cdev::recover(uint32_t clock)
{
  _i2cBus->end();
  _i2cBus->begin();
  _i2cBus->setClock(clock);
  _healthy = true;
}
