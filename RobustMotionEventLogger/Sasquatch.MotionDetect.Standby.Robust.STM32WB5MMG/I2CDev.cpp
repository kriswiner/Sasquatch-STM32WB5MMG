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

  _i2c_bus->beginTransmission(address);                         // Start the register-address transfer
  bool buffered = (_i2c_bus->write(subAddress) == 1);           // Queue the register address
  uint8_t error = _i2c_bus->endTransmission(false);             // Send it and retain the bus for a restart

  if(!buffered || error) return false;                          // Stop if the device did not acknowledge
  if(_i2c_bus->requestFrom(address, (uint8_t)1) != 1) return false; // Require exactly one returned byte
  if(!_i2c_bus->available()) return false;                      // Confirm that the receive buffer contains it

  *dest = (uint8_t)_i2c_bus->read();                            // Return the received register value
  return true;
}


bool I2Cdev::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest)
{
  if(!dest || !count) return false;                             // Reject an invalid request

  _i2c_bus->beginTransmission(address);                         // Start the register-address transfer
  bool buffered = (_i2c_bus->write(subAddress) == 1);           // Queue the first register address
  uint8_t error = _i2c_bus->endTransmission(false);             // Send it and retain the bus for a restart

  if(!buffered || error) return false;                          // Stop if the device did not acknowledge
  uint8_t received = _i2c_bus->requestFrom(address, count);     // Request the complete register block
  uint8_t i = 0;                                                // Count bytes copied to the destination

  while(_i2c_bus->available() && i < count) dest[i++] = _i2c_bus->read(); // Copy only requested bytes
  while(_i2c_bus->available()) _i2c_bus->read();                // Drain any unexpected excess bytes

  return received == count && i == count;                       // Accept only a complete transfer
}


bool I2Cdev::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  _i2c_bus->beginTransmission(address);                         // Start the register write
  bool buffered = (_i2c_bus->write(subAddress) == 1);           // Queue the register address
  buffered &= (_i2c_bus->write(data) == 1);                     // Queue the register value
  uint8_t error = _i2c_bus->endTransmission();                  // Send the complete transaction
  return buffered && !error;                                   // Report buffering or bus failure
}


bool I2Cdev::writeBytes(uint8_t address, uint8_t subAddress, uint8_t count, const uint8_t *data)
{
  if(!data || !count) return false;                             // Reject an invalid request

  _i2c_bus->beginTransmission(address);                         // Start the register-block write
  bool buffered = (_i2c_bus->write(subAddress) == 1);           // Queue the first register address

  for(uint8_t i = 0; i < count; i++) {
    if(_i2c_bus->write(data[i]) != 1) buffered = false;         // Queue data and detect buffer exhaustion
  }

  uint8_t error = _i2c_bus->endTransmission();                  // Send the complete transaction
  return buffered && !error;                                   // Report buffering or bus failure
}


void I2Cdev::I2Cscan()
{
  uint8_t devices = 0;                                         // Count responding devices
  Serial.println("Scanning...");

  for(uint8_t address = 1; address < 127; address++) {
    _i2c_bus->beginTransmission(address);                       // Address each valid seven-bit device
    uint8_t error = _i2c_bus->endTransmission();                // Read the acknowledgement result

    if(error == 0) {
      Serial.print("I2C device found at address 0x");
      if(address < 16) Serial.print("0");
      Serial.println(address, HEX);
      devices++;
    } else if(error == 4) {
      Serial.print("Unknown error at address 0x");
      if(address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if(devices == 0) Serial.println("No I2C devices found\n");
  else Serial.println("I2C scan complete\n");
}


bool I2Cdev::FSreadBytes(uint8_t address, uint8_t *dest)
{
  if(!dest) return false;                                       // Reject an invalid destination

  _i2c_bus->beginTransmission(address);                         // Address the fixed-frame device
  uint8_t error = _i2c_bus->endTransmission(false);             // Retain the bus for the read restart
  if(error) return false;                                       // Stop if the device did not acknowledge

  uint8_t received = _i2c_bus->requestFrom(address, (uint8_t)5); // Request the fixed five-byte frame
  uint8_t i = 0;                                                // Count bytes copied to the destination

  while(_i2c_bus->available() && i < 5) dest[i++] = _i2c_bus->read(); // Copy only the expected frame
  while(_i2c_bus->available()) _i2c_bus->read();                // Drain any unexpected excess bytes

  return received == 5 && i == 5;                              // Accept only a complete frame
}
