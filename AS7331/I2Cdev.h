/*
 * Small, presentation-free I2C helper for robust sensor drivers.
 *
 * Every transfer returns true only when the complete transaction succeeds.
 * A failed transfer latches healthy() false until recover() is called by the
 * main sketch.  Nothing in this driver writes to Serial.
 */

#ifndef _I2CDEV_H_
#define _I2CDEV_H_

#include <Arduino.h>
#include <Wire.h>

class I2Cdev
{
  public:
    I2Cdev(TwoWire *i2cBus);

    bool readByte(uint8_t address, uint8_t registerAddress, uint8_t *destination);
    bool readBytes(uint8_t address, uint8_t registerAddress, uint8_t count, uint8_t *destination);
    bool writeByte(uint8_t address, uint8_t registerAddress, uint8_t data);

    bool probe(uint8_t address);
    bool healthy() const;
    void recover(uint32_t clock = 400000);

  private:
    TwoWire *_i2cBus;
    bool _healthy;
};

#endif
