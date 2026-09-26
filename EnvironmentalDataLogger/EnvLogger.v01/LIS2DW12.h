/*
 * Compact LIS2DW12 driver used by the Sasquatch daughter-board firmware.
 * The sensor is on the Sasquatch internal Wire1 bus, not the daughter-board bus.
 */

#ifndef LIS2DW12_h
#define LIS2DW12_h

#include <Arduino.h>
#include "I2Cdev.h"

#define LIS2DW12_ADDRESS                  0x19
#define LIS2DW12_WHO_AM_I                 0x0F
#define LIS2DW12_CTRL1                    0x20
#define LIS2DW12_CTRL2                    0x21
#define LIS2DW12_CTRL3                    0x22
#define LIS2DW12_CTRL4_INT1_PAD_CTRL      0x23
#define LIS2DW12_CTRL5_INT2_PAD_CTRL      0x24
#define LIS2DW12_CTRL6                    0x25
#define LIS2DW12_STATUS                   0x27
#define LIS2DW12_OUT_X_L                  0x28
#define LIS2DW12_FIFO_CTRL                0x2E
#define LIS2DW12_FIFO_SAMPLES             0x2F
#define LIS2DW12_WAKE_UP_THS              0x34
#define LIS2DW12_WAKE_UP_DUR              0x35

#define LIS2DW12_SLEEP_ON                 0x40
#define LIS2DW12_WAKE_THRESHOLD_2         0x02
#define LIS2DW12_WAKE_DURATION_1          0x20
#define LIS2DW12_STATIONARY               0x10

typedef enum {
  LIS2DW12_LP_MODE_1 = 0x00,
  LIS2DW12_LP_MODE_2 = 0x01,
  LIS2DW12_LP_MODE_3 = 0x02,
  LIS2DW12_LP_MODE_4 = 0x03
} LIS2DW12LowPowerMode;

typedef enum {
  LIS2DW12_MODE_LOW_POWER   = 0x00,
  LIS2DW12_MODE_HIGH_PERF   = 0x01,
  LIS2DW12_MODE_SINGLE_CONV = 0x02
} LIS2DW12OperatingMode;

typedef enum {
  LIS2DW12_ODR_POWER_DOWN = 0x00,
  LIS2DW12_ODR_12_5_1_6Hz = 0x01,
  LIS2DW12_ODR_12_5Hz     = 0x02,
  LIS2DW12_ODR_25Hz       = 0x03,
  LIS2DW12_ODR_50Hz       = 0x04,
  LIS2DW12_ODR_100Hz      = 0x05,
  LIS2DW12_ODR_200Hz      = 0x06
} LIS2DW12OutputDataRate;

typedef enum {
  LIS2DW12_FS_2G  = 0x00,
  LIS2DW12_FS_4G  = 0x01,
  LIS2DW12_FS_8G  = 0x02,
  LIS2DW12_FS_16G = 0x03
} LIS2DW12FullScale;

typedef enum {
  LIS2DW12_BW_FILT_ODR2  = 0x00,
  LIS2DW12_BW_FILT_ODR4  = 0x01,
  LIS2DW12_BW_FILT_ODR10 = 0x02,
  LIS2DW12_BW_FILT_ODR20 = 0x03
} LIS2DW12Bandwidth;

// Sensor-qualified names prevent collisions with FIFO modes in other drivers.
typedef enum {
  LIS2DW12_FIFO_BYPASS         = 0x00,
  LIS2DW12_FIFO_MODE           = 0x01,
  LIS2DW12_FIFO_CONT_TO_FIFO   = 0x03,
  LIS2DW12_FIFO_BYPASS_TO_CONT = 0x04,
  LIS2DW12_FIFO_CONTINUOUS     = 0x06
} LIS2DW12FIFOMode;

class LIS2DW12
{
  public:
    LIS2DW12(I2Cdev *i2cBus);
    bool getChipID(uint8_t *chipID);
    bool reset();
    bool initMeasurement(uint8_t fs, uint8_t odr, uint8_t mode,
                         uint8_t lowPowerMode, uint8_t bandwidth,
                         bool lowNoise, bool stationaryMode);
    bool getStatus(uint8_t *status);
    bool readAccelData(int16_t *destination);
    bool powerDown();
    bool powerUp(uint8_t odr);
    bool configureFIFO(uint8_t fifoMode, uint8_t fifoThreshold);
    bool FIFOsamples(uint8_t *samples);

  private:
    I2Cdev *_i2cBus;
};

#endif
