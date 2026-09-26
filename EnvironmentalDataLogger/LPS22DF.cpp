/*
 * Copyright (c) 2020 Tlera Corp.  All rights reserved.
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
 * 
 * 
 * Library may be used freely and without limit with attribution.
 */

#include "LPS22DF.h"
#include "I2Cdev.h"

LPS22DF::LPS22DF(I2Cdev* i2c_bus)
{
  _i2c_bus = i2c_bus;
}


bool LPS22DF::getChipID(uint8_t *chipID)
{
  return _i2c_bus->readByte(LPS22DF_ADDRESS, LPS22DF_WHOAMI, chipID); // Read WHO_AM_I register for LPS22DF
}


bool LPS22DF::boot()
{
  uint8_t value;                                                     // Hold CTRL_REG2 while changing BOOT.
  if(!_i2c_bus->readByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG2, &value)) return false;
  if(!_i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG2, value | 0x80)) return false; // Reboot memory content.

  uint32_t start = millis();                                        // Bound the boot-completion wait.
  while(millis() - start < 100) {
    if(!_i2c_bus->readByte(LPS22DF_ADDRESS, LPS22DF_INT_SOURCE, &value)) return false;
    if(!(value & 0x80)) return true;                                // Finish when BOOT_ON clears.
  }

  return false;                                                     // Report a boot timeout.
}


bool LPS22DF::status(uint8_t *status)
{
  return _i2c_bus->readByte(LPS22DF_ADDRESS, LPS22DF_STATUS, status); // Read pressure and temperature status flags.
}


bool LPS22DF::Pressure(int32_t *pressure)
{
  if(!pressure) return false;                                      // Reject an invalid destination.

  uint8_t rawData[3] = {0, 0, 0};                                  // Hold 24-bit pressure register data.
  if(!_i2c_bus->readBytes(LPS22DF_ADDRESS, LPS22DF_PRESS_OUT_XL, 3, rawData)) return false;

  *pressure = (int32_t)(((uint32_t)rawData[2] << 24) | ((uint32_t)rawData[1] << 16) | ((uint32_t)rawData[0] << 8)) >> 8;
  return true;
}


bool LPS22DF::Temperature(int16_t *temperature)
{
  if(!temperature) return false;                                   // Reject an invalid destination.

  uint8_t rawData[2] = {0, 0};                                     // Hold 16-bit temperature register data.
  if(!_i2c_bus->readBytes(LPS22DF_ADDRESS, LPS22DF_TEMP_OUT_L, 2, rawData)) return false;

  *temperature = (int16_t)(((uint16_t)rawData[1] << 8) | rawData[0]);
  return true;
}


bool LPS22DF::readSample(int32_t *pressure, int16_t *temperature)
{
  if(!pressure || !temperature) return false;                     // Reject invalid destinations.

  // BDU and address auto-increment keep this pressure/temperature register
  // set coherent while all five bytes are returned in one I2C transaction.
  uint8_t rawData[5] = {0, 0, 0, 0, 0};
  if(!_i2c_bus->readBytes(LPS22DF_ADDRESS, LPS22DF_PRESS_OUT_XL,
                          sizeof(rawData), rawData)) return false;

  int32_t newPressure =
      (int32_t)(((uint32_t)rawData[2] << 24) |
                ((uint32_t)rawData[1] << 16) |
                ((uint32_t)rawData[0] << 8)) >> 8;
  int16_t newTemperature =
      (int16_t)(((uint16_t)rawData[4] << 8) | rawData[3]);

  *pressure = newPressure;                                        // Publish only a complete sample.
  *temperature = newTemperature;
  return true;
}


bool LPS22DF::reset()
{
  uint8_t value;                                                   // Hold CTRL_REG2 while changing SWRESET.
  if(!_i2c_bus->readByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG2, &value)) return false;
  if(!_i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG2, value | 0x04)) return false; // Start software reset.

  uint32_t start = millis();                                       // Bound the reset-completion wait.
  while(millis() - start < 100) {
    if(!_i2c_bus->readByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG2, &value)) return false;
    if(!(value & 0x04)) return true;                               // Finish when SWRESET clears.
  }

  return false;                                                    // Report a reset timeout.
    }


bool LPS22DF::powerDown()
{
  uint8_t value;                                                   // Hold CTRL_REG1 while changing ODR.
  if(!_i2c_bus->readByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG1, &value)) return false;
  if(!_i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG1, value & ~0x78)) return false; // Clear ODR bits.
  bool poweredDown = false;
  return isPoweredDown(&poweredDown) && poweredDown;
}


bool LPS22DF::isPoweredDown(bool *poweredDown)
{
  if(poweredDown == nullptr) return false;
  uint8_t ctrl1 = 0;
  if(!_i2c_bus->readByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG1, &ctrl1)) return false;
  *poweredDown = (ctrl1 & 0x78) == 0; // ODR[3:0] all zero selects power-down.
  return true;
}


bool LPS22DF::isIdle(bool *idle)
{
  if(idle == nullptr) return false;

  uint8_t ctrl1 = 0;
  uint8_t ctrl2 = 0;
  if(!_i2c_bus->readByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG1, &ctrl1)) return false;
  if(!_i2c_bus->readByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG2, &ctrl2)) return false;

  // ODR = 0000 selects power-down/one-shot. ONESHOT self-clears when the
  // conversion finishes, so both conditions together identify true idle.
  *idle = ((ctrl1 & 0x78) == 0) && ((ctrl2 & 0x01) == 0);
  return true;
}


bool LPS22DF::powerUp(uint8_t PODR)
{
  uint8_t value;                                                   // Hold CTRL_REG1 while changing ODR.
  if(!_i2c_bus->readByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG1, &value)) return false;
  value = value & ~0x78;                                           // Clear ODR bits.
  return _i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG1, value | ((PODR & 0x0F) << 3)); // Start continuous mode.
}


bool LPS22DF::oneShot()
{
  uint8_t value;                                                   // Hold CTRL_REG2 while changing ONE_SHOT.
  if(!_i2c_bus->readByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG2, &value)) return false;
  return _i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG2, value | 0x01); // Start one-shot conversion.
}


bool LPS22DF::waitForDataReady(uint32_t timeout)
{
  uint8_t value = 0;                                               // Hold STATUS while waiting for fresh data.
  uint32_t start = millis();                                       // Bound the data-ready wait.

  while(millis() - start < timeout) {
    if(!status(&value)) return false;
    if((value & 0x03) == 0x03) return true;                        // Finish when pressure and temperature are ready.
  }

  return false;                                                    // Report a data-ready timeout.
}


bool LPS22DF::configurationMatches(uint8_t PODR, uint8_t AVG, uint8_t LPF,
                                   bool enableLPF1)
{
  uint8_t ifCtrl = 0;
  uint8_t ctrl1 = 0;
  uint8_t ctrl2 = 0;
  uint8_t ctrl3 = 0;
  uint8_t ctrl4 = 0;
  if(!_i2c_bus->readByte(LPS22DF_ADDRESS, LPS22DF_IF_CTRL, &ifCtrl)) return false;
  if(!_i2c_bus->readByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG1, &ctrl1)) return false;
  if(!_i2c_bus->readByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG2, &ctrl2)) return false;
  if(!_i2c_bus->readByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG3, &ctrl3)) return false;
  if(!_i2c_bus->readByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG4, &ctrl4)) return false;

  const uint8_t expectedCtrl1 = ((PODR & 0x0F) << 3) | (AVG & 0x07);
  const uint8_t expectedCtrl2 =
      (enableLPF1 ? (((LPF & 0x01) << 5) | 0x10) : 0x00) | 0x08;

  return ifCtrl == 0x00 &&
         ctrl1 == expectedCtrl1 &&
         (ctrl2 & 0x38) == expectedCtrl2 && // Ignore transient ONE_SHOT/SWRESET/BOOT bits.
         ctrl3 == 0x01 &&
         ctrl4 == 0x00;
}


bool LPS22DF::Init(uint8_t PODR, uint8_t AVG, uint8_t LPF,
                   bool enableLPF1)
{
  // IF_CTRL, not I3C_IF_CTRL, owns the legacy I2C-interface pull controls.
  // The interrupt is unused in this logger. Retain its internal pull-down and
  // the CS pull-up, and leave I3C_IF_CTRL untouched.
  if(!_i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_IF_CTRL, 0x00)) return false;
  if(!_i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG1, ((PODR & 0x0F) << 3) | (AVG & 0x07))) return false;
  // LPF1 is optional; BDU remains enabled so a burst read is one coherent sample.
  uint8_t ctrl2 = 0x08;
  if(enableLPF1) ctrl2 |= ((LPF & 0x01) << 5) | 0x10;
  if(!_i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG2, ctrl2)) return false;
  // interrupt is push-pull (bit 1 = 0), active HIGH (bit 3 = 0) by default    
  if(!_i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG3, 0x01)) return false; // Enable auto increment of register addresses.
  // Interrupt sources are application-specific and are not needed by this
  // polling, nonblocking one-shot bring-up test.
  if(!_i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG4, 0x00)) return false;
  return configurationMatches(PODR, AVG, LPF, enableLPF1);
}


bool LPS22DF::FIFOStatus(uint8_t *dest)
{
  if(!dest) return false;                                         // Reject an invalid destination.

  uint8_t rawData[2]= {0, 0};                                     // Hold FIFO status registers.
  if(!_i2c_bus->readBytes(LPS22DF_ADDRESS, LPS22DF_FIFO_STATUS1, 2, rawData)) return false;
  dest[0] = rawData[0];
  dest[1] = rawData[1];
  return true;
}


bool LPS22DF::FIFOReset()
{
 return _i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_FIFO_CTRL, 0x00); // Disable watermark and enable BYPASS mode.
}


bool LPS22DF::initFIFO(uint8_t fmode, uint8_t wtm, bool stopOnWatermark)
{
  if(!_i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_FIFO_WTM, wtm & 0x7F)) return false; // Define watermark.
  return _i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_FIFO_CTRL, (stopOnWatermark ? 0x08 : 0x00) | (fmode & 0x07)); // Select FIFO mode.
}


bool LPS22DF::FIFOPressure(int32_t *pressure)
{
  if(!pressure) return false;                                      // Reject an invalid destination.

  uint8_t rawData[3] = {0, 0, 0};                                  // Hold FIFO pressure register data.
  if(!_i2c_bus->readBytes(LPS22DF_ADDRESS, LPS22DF_FIFO_DATA_OUT_PRESS_XL, 3, rawData)) return false;

  *pressure = (int32_t)(((uint32_t)rawData[2] << 24) | ((uint32_t)rawData[1] << 16) | ((uint32_t)rawData[0] << 8)) >> 8;
  return true;
}
