/*
 * Copyright (c) 2018 Tlera Corp.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
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
 * IN THE SOFTWARE.
 */

#include "Arduino.h"
#include "I2CDev.h"


I2Cdev::I2Cdev(TwoWire* i2c_bus)
{
  _i2c_bus = i2c_bus;                                           // Save the selected I2C interface
}


bool I2Cdev::readByte(uint8_t address, uint8_t subAddress, uint8_t *dest)
{
  if(!dest) return false;                                       // Reject an invalid destination

  return _i2c_bus->transfer(address, &subAddress, 1,
                            dest, 1) == 0;                       // Composite register read
}


bool I2Cdev::readBytes(uint8_t address, uint8_t subAddress,
                       size_t count, uint8_t *dest)
{
  if(!dest || !count) return false;                             // Reject an invalid request

  return _i2c_bus->transfer(address, &subAddress, 1,
                            dest, count) == 0;                   // Composite register read
}


bool I2Cdev::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  const uint8_t txData[2] = {subAddress, data};
  return _i2c_bus->transfer(address, txData, sizeof(txData),
                            nullptr, 0) == 0;                    // Complete register write
}


bool I2Cdev::writeBytes(uint8_t address, uint8_t subAddress, uint8_t count, const uint8_t *data)
{
  if(!data || !count) return false;                             // Reject an invalid request

  uint8_t txData[256];
  txData[0] = subAddress;
  for(uint16_t i = 0; i < count; i++) txData[i + 1] = data[i];
  return _i2c_bus->transfer(address, txData, (size_t)count + 1,
                            nullptr, 0) == 0;                    // Complete register-block write
}


uint8_t I2Cdev::probeAddress(uint8_t address)
{
  if(address < 0x08 || address > 0x77) return 4;          // Reject reserved seven-bit addresses

  _i2c_bus->beginTransmission(address);                   // Address one valid seven-bit device
  return _i2c_bus->endTransmission();                     // Return the Wire acknowledgement result
}
