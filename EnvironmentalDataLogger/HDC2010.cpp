/*
 * Copyright (c) 2021 Tlera Corp.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Tlera Corp, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#include "HDC2010.h"


HDC2010::HDC2010(I2Cdev* i2c_bus)
{
  _i2c_bus = i2c_bus;
}


bool HDC2010::reset(uint8_t HDC2010_ADDRESS)
{
  // SOFT_RES is bit 7 of CONFIG1 (0x0E), not the interrupt status register.
  // This setup-only delay allows the EEPROM values and registers to reload.
  if(!_i2c_bus->writeByte(HDC2010_ADDRESS, HDC2010_CONFIG1, 0x80)) return false;
  delay(3);
  return true;
}


bool HDC2010::heaterOn(uint8_t HDC2010_ADDRESS)
{
  // Preserve the auto-measurement and interrupt settings while enabling the heater.
  uint8_t config = 0;
  if(!_i2c_bus->readByte(HDC2010_ADDRESS, HDC2010_CONFIG1, &config)) return false;
  return _i2c_bus->writeByte(HDC2010_ADDRESS, HDC2010_CONFIG1, config | 0x08);
}


bool HDC2010::heaterOff(uint8_t HDC2010_ADDRESS)
{
  // The heater is normally off; it is intended only for deliberate condensation recovery.
  uint8_t config = 0;
  if(!_i2c_bus->readByte(HDC2010_ADDRESS, HDC2010_CONFIG1, &config)) return false;
  return _i2c_bus->writeByte(HDC2010_ADDRESS, HDC2010_CONFIG1, config & ~0x08);
}


bool HDC2010::idle(uint8_t HDC2010_ADDRESS)
{
  // Force mode, heater off, interrupts disabled and no conversion trigger.
  if(!_i2c_bus->writeByte(HDC2010_ADDRESS, HDC2010_CONFIG1, 0x00)) return false;
  if(!_i2c_bus->writeByte(HDC2010_ADDRESS, HDC2010_CONFIG2, 0x00)) return false;
  if(!_i2c_bus->writeByte(HDC2010_ADDRESS, HDC2010_INT_EN, 0x00)) return false;

  bool idleState = false;
  return isIdle(HDC2010_ADDRESS, &idleState) && idleState;
}


bool HDC2010::isIdle(uint8_t HDC2010_ADDRESS, bool *idleState)
{
  if(idleState == nullptr) return false;

  uint8_t config1 = 0;
  uint8_t config2 = 0;
  uint8_t interruptEnable = 0;
  if(!_i2c_bus->readByte(HDC2010_ADDRESS, HDC2010_CONFIG1, &config1)) return false;
  if(!_i2c_bus->readByte(HDC2010_ADDRESS, HDC2010_CONFIG2, &config2)) return false;
  if(!_i2c_bus->readByte(HDC2010_ADDRESS, HDC2010_INT_EN, &interruptEnable)) return false;

  *idleState = ((config1 & 0x78) == 0) && // force mode and heater off
               ((config2 & 0x01) == 0) && interruptEnable == 0;
  return true;
}


bool HDC2010::readData(uint8_t HDC2010_ADDRESS, float *temperature, float *humidity)
{
  if(temperature == nullptr || humidity == nullptr) return false;

  uint8_t rawData[4] = {0, 0, 0, 0};

  // Read both results in one transaction so they come from the same conversion.
  // Reading the output registers also clears DRDY_STATUS if it is set.
  if(!_i2c_bus->readBytes(HDC2010_ADDRESS, HDC2010_TEMP_L, 4, rawData)) return false;

  uint16_t rawTemperature = ((uint16_t) rawData[1] << 8) | rawData[0];
  uint16_t rawHumidity    = ((uint16_t) rawData[3] << 8) | rawData[2];
  float newTemperature = ((float) rawTemperature) * (165.0f / 65536.0f) - 40.0f;
  float newHumidity    = ((float) rawHumidity)    * (100.0f / 65536.0f);

  *temperature = newTemperature;
  *humidity = newHumidity;
  return true;
}


bool HDC2010::getDevID(uint8_t HDC2010_ADDRESS, uint16_t *devID)
{
  if(devID == nullptr) return false;

  uint8_t rawData[2] = {0, 0};
  if(!_i2c_bus->readBytes(HDC2010_ADDRESS, HDC2010_DEV_ID_L, 2, rawData)) return false;

  *devID = ((uint16_t) rawData[1] << 8) | rawData[0];
  return true;
}


bool HDC2010::getManuID(uint8_t HDC2010_ADDRESS, uint16_t *manuID)
{
  if(manuID == nullptr) return false;

  uint8_t rawData[2] = {0, 0};
  if(!_i2c_bus->readBytes(HDC2010_ADDRESS, HDC2010_MANU_ID_L, 2, rawData)) return false;

  *manuID = ((uint16_t) rawData[1] << 8) | rawData[0];
  return true;
}


bool HDC2010::getIntStatus(uint8_t HDC2010_ADDRESS, uint8_t *status)
{
  if(status == nullptr) return false;
  return _i2c_bus->readByte(HDC2010_ADDRESS, HDC2010_INT_STATUS, status);
}


bool HDC2010::init(uint8_t HDC2010_ADDRESS, uint8_t hres, uint8_t tres, uint8_t freq)
{
  /*
   * The daughter-board logger uses timer-polled one-shot measurements, not the
   * HDC2010 DRDY/INT pin. Keep interrupts disabled so the direct MCU GPIO does
   * not change state or consume current after the first conversion.
   */
  if(!_i2c_bus->writeByte(HDC2010_ADDRESS, HDC2010_CONFIG1, freq << 4)) return false;

  // Set temperature and humidity resolution, measure both T/RH, but do not
  // trigger here. startEnvironmentalMeasurement() starts each one-shot sample.
  if(!_i2c_bus->writeByte(HDC2010_ADDRESS, HDC2010_CONFIG2, tres << 6 | hres << 4)) return false;
  if(!_i2c_bus->writeByte(HDC2010_ADDRESS, HDC2010_INT_EN, 0x00)) return false;

  bool idleState = false;
  return isIdle(HDC2010_ADDRESS, &idleState) && idleState;
}


bool HDC2010::triggerMeasurement(uint8_t HDC2010_ADDRESS)
{
  uint8_t configuration = 0;
  if(!_i2c_bus->readByte(HDC2010_ADDRESS, HDC2010_CONFIG2, &configuration)) return false;
  return _i2c_bus->writeByte(HDC2010_ADDRESS, HDC2010_CONFIG2, configuration | 0x01);
}
