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
  if(!pressure || !temperature) return false;

  // With BDU and address auto-increment enabled, this single burst returns
  // one coherent pressure/temperature register set and consumes both values.
  uint8_t rawData[5] = {0, 0, 0, 0, 0};
  if(!_i2c_bus->readBytes(LPS22DF_ADDRESS, LPS22DF_PRESS_OUT_XL,
                          5, rawData)) return false;

  *pressure = (int32_t)(((uint32_t)rawData[2] << 24) |
                        ((uint32_t)rawData[1] << 16) |
                        ((uint32_t)rawData[0] << 8)) >> 8;
  *temperature =
      (int16_t)(((uint16_t)rawData[4] << 8) | rawData[3]);
  return true;
}


bool LPS22DF::configureInterrupts(uint8_t routing)
{
  // CTRL_REG4 routes independent sources to INT_DRDY. It is not a global
  // interrupt-enable register; enable only the sources the application uses.
  return _i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG4,
                             routing & 0x77);
}


bool LPS22DF::configurationMatches(uint8_t PODR, uint8_t AVG, uint8_t LPF,
                                   bool enableLPF1)
{
  uint8_t ctrl1 = 0;
  uint8_t ctrl2 = 0;
  if(!_i2c_bus->readByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG1,
                         &ctrl1)) return false;
  if(!_i2c_bus->readByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG2,
                         &ctrl2)) return false;

  const uint8_t expectedCtrl1 =
      (uint8_t)(((PODR & 0x0F) << 3) | (AVG & 0x07));
  const uint8_t expectedCtrl2 =
      (uint8_t)((enableLPF1 ? (((LPF & 0x01) << 5) | 0x10) : 0x00) |
                0x08);

  // ONE_SHOT self-clears and SWRESET/BOOT are transient, so compare only
  // persistent configuration bits owned by Init().
  return ctrl1 == expectedCtrl1 &&
         (ctrl2 & 0x38) == (expectedCtrl2 & 0x38);
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
  return _i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG1, value & ~0x78); // Clear ODR bits.
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


bool LPS22DF::Init(uint8_t PODR, uint8_t AVG, uint8_t LPF,
                   bool enableLPF1)
{
  // Keep IF_CTRL and I3C_IF_CTRL_ADD at their reset values. In particular,
  // I3C_IF_CTRL_ADD bit 7 is fixed at 1 and must not be cleared.
  if(!_i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG1, ((PODR & 0x0F) << 3) | (AVG & 0x07))) return false;
  // EN_LPFP (bit 4) optionally enables LPF1. LPFP_CFG (bit 5) selects
  // ODR/4 or ODR/9 only while LPF1 is enabled. BDU (bit 3) is always enabled.
  uint8_t ctrl2 = 0x08;
  if(enableLPF1) ctrl2 |= ((LPF & 0x01) << 5) | 0x10;
  if(!_i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG2,
                          ctrl2)) return false;
  // interrupt is push-pull (bit 1 = 0), active HIGH (bit 3 = 0) by default    
  if(!_i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG3, 0x01)) return false; // Enable auto increment of register addresses.
  // Interrupt sources are application-specific and start disabled.
  return configureInterrupts(0x00);
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
