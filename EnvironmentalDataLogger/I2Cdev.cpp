/*
 * Copyright (c) 2018 Tlera Corp.  All rights reserved.
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

#include "Arduino.h"
#include "I2Cdev.h"

static void i2cdevReleaseLine(uint8_t pin)
{
  pinMode(pin, INPUT);
}


static void i2cdevDriveLineLow(uint8_t pin)
{
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}


static void i2cdevClearExternalWireBus()
{
  // If an external slave is stuck mid-byte it may hold SDA low after the STM32WB
  // I2C peripheral is reset. Clock SCL manually, then synthesize a STOP.
  i2cdevReleaseLine(SDA);
  i2cdevReleaseLine(SCL);
  delayMicroseconds(10);

  for(uint8_t ii = 0; ii < 9 && digitalRead(SDA) == LOW; ii++) {
    i2cdevDriveLineLow(SCL);
    delayMicroseconds(5);
    i2cdevReleaseLine(SCL);
    delayMicroseconds(5);
  }

  i2cdevDriveLineLow(SDA);
  delayMicroseconds(5);
  i2cdevReleaseLine(SCL);
  delayMicroseconds(5);
  i2cdevReleaseLine(SDA);
  delayMicroseconds(10);
}

I2Cdev::I2Cdev(TwoWire* i2c_bus)                                                                                                             // Class constructor
{
  _i2c_bus = i2c_bus;
}


I2Cdev::~I2Cdev()                                                                                                                            // Class destructor
{
}


bool I2Cdev::readByte(uint8_t address, uint8_t subAddress, uint8_t *dest)
{
  if(!dest) { _healthy = false; return false; }
#if I2CDEV_USE_WIRE_TRANSFER
  bool success = (_i2c_bus->transfer(address, &subAddress, 1,
                                     dest, 1) == 0);
  if(!success) _healthy = false;
  return success;
#else
  _i2c_bus->beginTransmission(address);         // Initialize the Tx buffer
  bool buffered = (_i2c_bus->write(subAddress) == 1); // Put slave register address in Tx buffer
  uint8_t error = _i2c_bus->endTransmission(false);   // Send the Tx buffer, but send a restart to keep connection alive
  if(!buffered || error) { _healthy = false; return false; }
  uint8_t received = _i2c_bus->requestFrom(address, (uint8_t)1);
  if(received != 1 || !_i2c_bus->available()) { _healthy = false; return false; }
  *dest = (uint8_t)_i2c_bus->read();             // Update caller data only after a complete transfer
  while(_i2c_bus->available()) _i2c_bus->read(); // Drain any unexpected excess bytes
  return true;
#endif
}


/**
* @fn: readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
*
* @brief: Read multiple bytes from an I2C device
* 
* @params: I2C slave device address, Register subAddress, number of btes to be read, aray to store the read data
* @returns: void
*/
bool I2Cdev::readBytes(uint8_t address, uint8_t subAddress, size_t count, uint8_t *dest)
{  
  if(!dest || !count) { _healthy = false; return false; }
#if I2CDEV_USE_WIRE_TRANSFER
  // Direct composite read; unlike portable Wire, this path is not constrained
  // by its intermediate receive buffer.
  bool success = (_i2c_bus->transfer(address, &subAddress, 1,
                                     dest, count) == 0);
  if(!success) _healthy = false;
  return success;
#else
  if(count > 255) { _healthy = false; return false; }
  _i2c_bus->beginTransmission(address);   // Initialize the Tx buffer
  bool buffered = (_i2c_bus->write(subAddress) == 1); // Put slave register address in Tx buffer
  uint8_t error = _i2c_bus->endTransmission(false);   // Send the Tx buffer, but send a restart to keep connection alive
  if(!buffered || error) { _healthy = false; return false; }
  uint8_t i = 0;
  uint8_t received = _i2c_bus->requestFrom(address, (uint8_t)count); // Read bytes from slave register address
  while (_i2c_bus->available() && i < (uint8_t)count) {
        dest[i++] = _i2c_bus->read(); }   // Put read results in the Rx buffer
  while (_i2c_bus->available()) _i2c_bus->read(); // Drain any unexpected extra bytes
  bool success = (received == count && i == count);
  if(!success) _healthy = false;
  return success;
#endif
}


/**
* @fn: writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data)
*
* @brief: Write one byte to an I2C device
* 
* @params: I2C slave device address, Register subAddress, data to be written
* @returns: void
*/
bool I2Cdev::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data)
{
#if I2CDEV_USE_WIRE_TRANSFER
  const uint8_t txData[2] = {regAddr, data};
  bool success = (_i2c_bus->transfer(devAddr, txData, sizeof(txData),
                                     nullptr, 0) == 0);
  if(!success) _healthy = false;
  return success;
#else
  _i2c_bus->beginTransmission(devAddr);  // Initialize the Tx buffer
  bool buffered = (_i2c_bus->write(regAddr) == 1); // Put slave register address in Tx buffer
  buffered &= (_i2c_bus->write(data) == 1);        // Put data in Tx buffer
  uint8_t error = _i2c_bus->endTransmission();     // Send the Tx buffer
  bool success = buffered && !error;
  if(!success) _healthy = false;
  return success;
#endif
}


bool I2Cdev::writeCommand(uint8_t address, uint8_t command)
{
#if I2CDEV_USE_WIRE_TRANSFER
  bool success = (_i2c_bus->transfer(address, &command, 1,
                                     nullptr, 0) == 0);
  if(!success) _healthy = false;
  return success;
#else
  _i2c_bus->beginTransmission(address);          // Start a command-only transaction
  bool buffered = (_i2c_bus->write(command) == 1);
  uint8_t error = _i2c_bus->endTransmission();
  bool success = buffered && !error;
  if(!success) _healthy = false;
  return success;
#endif
}


/**
* @fn: writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t data)
*
* @brief: Write multiple bytes to an I2C device
* 
* @params: I2C slave device address, Register subAddress, byte count, data array to be written
* @returns: void
*/
bool I2Cdev::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t count, const uint8_t *dest)
{
  if(!dest || !count) { _healthy = false; return false; }
  uint8_t temp[256];
  
  temp[0] = regAddr;
  for (uint8_t ii = 0; ii < count; ii++)
  { 
    temp[ii + 1] = dest[ii];
  }

#if I2CDEV_USE_WIRE_TRANSFER
  bool success = (_i2c_bus->transfer(devAddr, temp, (size_t)count + 1,
                                     nullptr, 0) == 0);
  if(!success) _healthy = false;
  return success;
#else
  _i2c_bus->beginTransmission(devAddr);  // Initialize the Tx buffer
  bool buffered = true;
  
  for (uint8_t jj = 0; jj < count + 1; jj++)
  {
  if(_i2c_bus->write(temp[jj]) != 1) buffered = false; // Put data in Tx buffer
  }
  
  uint8_t error = _i2c_bus->endTransmission(); // Send the Tx buffer
  bool success = buffered && !error;
  if(!success) _healthy = false;
  return success;
#endif
}



uint8_t I2Cdev::u1_CRC_8_u1u1( uint8_t u1ArgBeforeData , uint8_t u1ArgAfterData)
{
  unsigned char u1TmpLooper = 0;
  unsigned char u1TmpOutData = 0;
  unsigned short  u2TmpValue = 0;
  uint16_t  dPOLYNOMIAL8  =   0x8380;

  u2TmpValue = (unsigned short)(u1ArgBeforeData ^ u1ArgAfterData);
  u2TmpValue <<= 8;

  for( u1TmpLooper = 0 ; u1TmpLooper < 8 ; u1TmpLooper++ ){
    if( u2TmpValue & 0x8000 ){
      u2TmpValue ^= dPOLYNOMIAL8;
    }
    u2TmpValue <<= 1;
  }

  u1TmpOutData = (unsigned char)(u2TmpValue >> 8);

  return( u1TmpOutData );
}


bool I2Cdev::readBytes16(uint8_t address, uint8_t subAddress, uint16_t *dest)
{
  if(!dest) { _healthy = false; return false; }
  uint8_t u1Calc = 0;
  uint8_t u1CRC8 = 0;
  uint8_t tmp[3] = {0, 0, 0};

  if(!readBytes(address, subAddress, 3, tmp)) return false; // Read data and CRC atomically.

  u1Calc = u1_CRC_8_u1u1( 0x00   , address<<1 );     // Write Address
  u1Calc = u1_CRC_8_u1u1( u1Calc , subAddress );     // Command
  u1Calc = u1_CRC_8_u1u1( u1Calc , (address<<1) + 1 ); // Read Address
  u1Calc = u1_CRC_8_u1u1( u1Calc , tmp[0] );         // Data LOW
  u1CRC8 = u1_CRC_8_u1u1( u1Calc , tmp[1] );         // Data HIGH

  if(tmp[2] == u1CRC8) {
  *dest = (uint16_t) ((uint16_t) tmp[1] << 8) | tmp[0];
  return true;
  }
  _healthy = false;                               // Treat a failed CRC as an invalid I2C transfer
  return false;
}


bool I2Cdev::writeBytes16(uint8_t address, uint8_t subAddress, uint16_t data)
{  
  uint8_t u1Calc = 0;
  uint8_t u1CRC8 = 0;
  uint8_t tmp[4] = {0, 0, 0, 0};
  
  tmp[0] =  subAddress;
  tmp[1] =  data & 0x00FF;
  tmp[2] = (data & 0xFF00) >> 8;
  
  u1Calc = u1_CRC_8_u1u1( 0x00 ,  address<<1 );  // Address
  u1Calc = u1_CRC_8_u1u1( u1Calc , tmp[0] );  // Command
  u1Calc = u1_CRC_8_u1u1( u1Calc , tmp[1] );  // LSB
  u1CRC8 = u1_CRC_8_u1u1( u1Calc , tmp[2] );  // MSB

  tmp[3] = u1CRC8;

#if I2CDEV_USE_WIRE_TRANSFER
  bool success = (_i2c_bus->transfer(address, tmp, sizeof(tmp),
                                     nullptr, 0) == 0);
#else
  _i2c_bus->beginTransmission(address);  // Initialize the Tx buffer
  
  bool buffered = true;
  for (uint8_t jj = 0; jj < 4; jj++)
  {
  if(_i2c_bus->write(tmp[jj]) != 1) buffered = false; // Put data in Tx buffer
  }
  
  uint8_t error = _i2c_bus->endTransmission(); // Send the Tx buffer
  bool success = buffered && !error;
#endif
  if(!success) _healthy = false;
  return success;
}


bool I2Cdev::probe(uint8_t address)
{
  // A missing address is normal during a diagnostic scan, so probe results do
  // not alter the latched health state used for actual register transfers.
  _i2c_bus->beginTransmission(address);
  return _i2c_bus->endTransmission() == 0;
}


bool I2Cdev::healthy()
{
  return _healthy;                                // Preserve the first failure until the main loop handles it
}


void I2Cdev::clearHealthFault()
{
  _healthy = true;                                // Clear only the software fault latch; do not touch the bus.
}


void I2Cdev::recover(uint32_t clock)
{
  _i2c_bus->end();                                // Reset the STM32WB I2C peripheral after a failed transfer
  if(_i2c_bus == &Wire) i2cdevClearExternalWireBus(); // Clock loose only the daughter-board bus
  _i2c_bus->begin();                              // Return the bus to master mode
  _i2c_bus->setClock(clock);                      // Restore the configured bus rate
#if defined(WIRE_HAS_CLOCK_LOW_TIMEOUT)
  if(_i2c_bus == &Wire) _i2c_bus->setClockLowTimeout(25000);
#endif
  _healthy = true;                                // Allow the next scheduled sensor access to retry normally
}
