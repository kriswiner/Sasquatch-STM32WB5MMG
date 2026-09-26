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

#ifndef _I2CDEV_H_
#define _I2CDEV_H_

#include <Wire.h>

// STM32WB supplies transfer(), a complete write/repeated-start/read operation
// without the traditional Arduino Wire buffer limit. Set this to 0 when using
// a core that does not provide TwoWire::transfer().
#ifndef I2CDEV_USE_WIRE_TRANSFER
#define I2CDEV_USE_WIRE_TRANSFER 1
#endif

class I2Cdev {
    public:
                                        I2Cdev(TwoWire*);
                                        ~I2Cdev();                                                                                                                     // Class destructor for durable instances
         bool                           readByte(uint8_t address, uint8_t subAddress, uint8_t *dest);
         bool                           readBytes(uint8_t address, uint8_t subAddress, size_t count, uint8_t *dest);
         bool                           writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
         bool                           writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t count, const uint8_t *dest);
         bool                           writeCommand(uint8_t address, uint8_t command);
         uint8_t                        u1_CRC_8_u1u1( uint8_t u1ArgBeforeData , uint8_t u1ArgAfterData);
         bool                           readBytes16(uint8_t address, uint8_t subAddress, uint16_t *dest);
         bool                           writeBytes16(uint8_t address, uint8_t subAddress, uint16_t data);
         bool                           probe(uint8_t address);
         bool                           healthy();
         void                           clearHealthFault();
         void                           recover(uint32_t clock = 400000);
         
    private:
         TwoWire*                       _i2c_bus;                                                                                                                      // Class constructor argument
         bool                           _healthy = true;                                                                                                               // Latched false after an incomplete transfer
};

#endif //_I2CDEV_H_
