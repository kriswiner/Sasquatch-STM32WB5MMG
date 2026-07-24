/*
 * Robust AS7331 UVA/UVB/UVC sensor driver.
 *
 * The driver performs checked register transfers and contains no delays or
 * Serial output. Timing, retry policy, and user messages belong to the main
 * sketch.
 */

#ifndef _AS7331_H_
#define _AS7331_H_

#include <Arduino.h>
#include "I2Cdev.h"

#define AS7331_ADDRESS                 0x74 // A1 = LOW, A0 = LOW
#define AS7331_CHIP_ID                 0x21

// Register addresses while the device is in configuration state.
#define AS7331_OSR                     0x00
#define AS7331_AGEN                    0x02
#define AS7331_CREG1                   0x06
#define AS7331_CREG2                   0x07
#define AS7331_CREG3                   0x08
#define AS7331_BREAK                   0x09

// Register addresses while the device is in measurement state.
#define AS7331_STATUS                  0x00
#define AS7331_TEMP                    0x01

// Explicit OSR commands. Avoid OR-ing new states into an old OSR value.
#define AS7331_OSR_START               0x80
#define AS7331_OSR_POWER_DOWN          0x40
#define AS7331_OSR_SOFTWARE_RESET      0x08
#define AS7331_OSR_MEASUREMENT         0x03
#define AS7331_OSR_CONFIGURATION       0x02

// Low STATUS byte returned after the high OSR byte at measurement address 0.
#define AS7331_STATUS_OUTCONVOF        0x80
#define AS7331_STATUS_MRESOF           0x40
#define AS7331_STATUS_ADCOF            0x20
#define AS7331_STATUS_LDATA            0x10
#define AS7331_STATUS_NDATA            0x08
#define AS7331_STATUS_NOTREADY         0x04
#define AS7331_STATUS_STANDBY          0x02
#define AS7331_STATUS_POWER_DOWN       0x01

enum AS7331MeasurementMode : uint8_t {
  AS7331_CONT_MODE = 0x00,
  AS7331_CMD_MODE  = 0x01,
  AS7331_SYNS_MODE = 0x02,
  AS7331_SYND_MODE = 0x03
};

enum AS7331ConversionClock : uint8_t {
  AS7331_1024_KHZ = 0x00,
  AS7331_2048_KHZ = 0x01,
  AS7331_4096_KHZ = 0x02,
  AS7331_8192_KHZ = 0x03
};

struct AS7331Data {
  uint16_t temperature;
  uint16_t uva;
  uint16_t uvb;
  uint16_t uvc;
};

class AS7331
{
  public:
    AS7331(I2Cdev *i2cBus, uint8_t address = AS7331_ADDRESS);

    bool getChipID(uint8_t *chipID);
    bool readOperatingState(uint8_t *operatingState);
    bool softwareReset();
    bool enterConfiguration();
    bool enterMeasurement(bool startMeasurement = false);
    bool powerDown();
    bool powerUp();

    bool configure(AS7331MeasurementMode mode,
                   AS7331ConversionClock clock,
                   bool standbyEnabled,
                   uint8_t breakTime,
                   uint8_t gain,
                   uint8_t integrationTime);

    bool startOneShot();
    bool readStatus(uint16_t *status);
    bool readAllData(AS7331Data *data);

    bool healthy() const;
    void clearHealth();

  private:
    bool result(bool success);

    I2Cdev *_i2cBus;
    uint8_t _address;
    bool _healthy;
};

#endif
