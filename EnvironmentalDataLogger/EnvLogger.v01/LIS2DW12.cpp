/* Compact, error-reporting LIS2DW12 driver. */

#include "LIS2DW12.h"

LIS2DW12::LIS2DW12(I2Cdev *i2cBus)
{
  _i2cBus = i2cBus;
}


bool LIS2DW12::getChipID(uint8_t *chipID)
{
  return _i2cBus->readByte(LIS2DW12_ADDRESS, LIS2DW12_WHO_AM_I, chipID);
}


bool LIS2DW12::reset()
{
  uint8_t value = 0;
  if(!_i2cBus->readByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL2, &value)) return false;
  if(!_i2cBus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL2, value | 0x40)) return false;

  // Reset normally finishes quickly; this bounded setup-only poll cannot hang.
  uint32_t deadline = millis() + 10UL;
  do {
    if(!_i2cBus->readByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL2, &value)) return false;
    if((value & 0x40) == 0) return true;
  } while((int32_t)(millis() - deadline) < 0);
  return false;
}


bool LIS2DW12::initMeasurement(uint8_t fs, uint8_t odr, uint8_t mode,
                               uint8_t lowPowerMode, uint8_t bandwidth,
                               bool lowNoise, bool stationaryMode)
{
  uint8_t ctrl1 = (odr << 4) | (mode << 2) | lowPowerMode;
  uint8_t ctrl6 = (bandwidth << 6) | (fs << 4) | (lowNoise ? 0x04 : 0x00);
  uint8_t wakeThreshold = stationaryMode ?
                          (LIS2DW12_SLEEP_ON | LIS2DW12_WAKE_THRESHOLD_2) : 0x00;
  uint8_t wakeDuration = stationaryMode ?
                         (LIS2DW12_WAKE_DURATION_1 | LIS2DW12_STATIONARY) : 0x00;
  uint8_t value = 0;

  if(!_i2cBus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL2, 0x0C)) return false; // BDU + auto increment.
  if(!_i2cBus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL3, 0x00)) return false; // No interrupt behavior in bring-up.
  if(!_i2cBus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL4_INT1_PAD_CTRL, 0x00)) return false;
  if(!_i2cBus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL5_INT2_PAD_CTRL, 0x00)) return false;
  if(!_i2cBus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL6, ctrl6)) return false;
  if(!_i2cBus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_WAKE_UP_THS, wakeThreshold)) return false;
  if(!_i2cBus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_WAKE_UP_DUR, wakeDuration)) return false;
  if(!_i2cBus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL1, ctrl1)) return false;

  // Verify the low-power ODR and stationary-mode controls before reporting success.
  if(!_i2cBus->readByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL1, &value) || value != ctrl1) return false;
  if(!_i2cBus->readByte(LIS2DW12_ADDRESS, LIS2DW12_WAKE_UP_THS, &value) ||
     (value & (LIS2DW12_SLEEP_ON | 0x3F)) != wakeThreshold) return false;
  if(!_i2cBus->readByte(LIS2DW12_ADDRESS, LIS2DW12_WAKE_UP_DUR, &value) ||
     (value & (LIS2DW12_WAKE_DURATION_1 | LIS2DW12_STATIONARY)) != wakeDuration) return false;

  return true;
}


bool LIS2DW12::getStatus(uint8_t *status)
{
  return _i2cBus->readByte(LIS2DW12_ADDRESS, LIS2DW12_STATUS, status);
}


bool LIS2DW12::readAccelData(int16_t *destination)
{
  if(!destination) return false;
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  if(!_i2cBus->readBytes(LIS2DW12_ADDRESS, LIS2DW12_OUT_X_L, 6, rawData)) return false;

  // LP mode 2 supplies left-justified 14-bit samples.
  destination[0] = ((int16_t)(((uint16_t)rawData[1] << 8) | rawData[0])) >> 2;
  destination[1] = ((int16_t)(((uint16_t)rawData[3] << 8) | rawData[2])) >> 2;
  destination[2] = ((int16_t)(((uint16_t)rawData[5] << 8) | rawData[4])) >> 2;
  return true;
}


bool LIS2DW12::powerDown()
{
  uint8_t value = 0;
  if(!_i2cBus->readByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL1, &value)) return false;
  return _i2cBus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL1, value & 0x0F);
}


bool LIS2DW12::powerUp(uint8_t odr)
{
  uint8_t value = 0;
  if(!_i2cBus->readByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL1, &value)) return false;
  return _i2cBus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL1, (value & 0x0F) | (odr << 4));
}


bool LIS2DW12::configureFIFO(uint8_t fifoMode, uint8_t fifoThreshold)
{
  return _i2cBus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_FIFO_CTRL,
                            (fifoMode << 5) | (fifoThreshold & 0x1F));
}


bool LIS2DW12::FIFOsamples(uint8_t *samples)
{
  return _i2cBus->readByte(LIS2DW12_ADDRESS, LIS2DW12_FIFO_SAMPLES, samples);
}
