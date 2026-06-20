/* 9/18/21 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 *  The LIS2DW12 is an inexpensive (~$1), three-axis, medium-resolution (12- or 14-bit), ultra-low power
 *  (<1 uA low power mode) accelerometer in a tiny 2 mm x 2 mm LGA12 package with a 192-byte FIFO,
 *  two multifunction interrupts and widely configurable sample rate (1.6 - 1600 Hz), full range (2 - 16 g),
 *  low power modes, and interrupt detection behaviors. This accelerometer is a nice choice for motion-based
 *  wake/sleep, tap detection, step counting, and simple orientation estimation.
 *  
 *  Library may be used freely and without limit with attribution.
 */

#include "LIS2DW12.h"
#include "I2CDev.h"


LIS2DW12::LIS2DW12(I2Cdev* i2c_bus)
{
  _i2c_bus = i2c_bus;                                             // Save the selected I2C interface
}


bool LIS2DW12::getChipID(uint8_t *chipID)
{
  return _i2c_bus->readByte(LIS2DW12_ADDRESS, LIS2DW12_WHO_AM_I, chipID); // Read the device identity
}


bool LIS2DW12::getStatus(uint8_t *status)
{
  return _i2c_bus->readByte(LIS2DW12_ADDRESS, LIS2DW12_STATUS, status); // Read the sensor status flags
}


bool LIS2DW12::init(uint8_t fs, uint8_t odr, uint8_t mode, uint8_t lpMode, uint8_t bw, bool lowNoise, bool stMode)
{
  uint8_t ctrl6 = (bw << 6) | (fs << 4) | (lowNoise ? 0x04 : 0x00); // Combine bandwidth, scale, and noise settings

  if(!_i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL1, odr << 4 | mode << 2 | lpMode)) return false; // Set ODR and power mode
  if(!_i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL6, ctrl6)) return false; // Set bandwidth, scale, and noise mode

  _aRes = 0.000244f * (1 << fs);                                  // Calculate 14-bit acceleration resolution

  if(!_i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL2, 0x08 | 0x04)) return false; // Enable BDU and register increment
  if(!_i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL3, 0x10)) return false; // Select push-pull, active-high, latched interrupts
  if(!_i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL4_INT1_PAD_CTRL, 0x20)) return false; // Route wake-up to INT1
  if(!_i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL5_INT2_PAD_CTRL, 0x40)) return false; // Route sleep-state change to INT2
  if(!_i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_WAKE_UP_THS, 0x40 | 0x02)) return false; // Enable sleep detection and set threshold
  if(!_i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_WAKE_UP_DUR, stMode ? 0x30 : 0x20)) return false; // Set wake duration and stationary mode
  if(!_i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL_REG7, 0x80 | 0x40 | 0x20)) return false; // Pulse DRDY and combine INT2 with INT1

  return true;
}


bool LIS2DW12::activateWakeOnMotionInterrupt()
{
  uint8_t value;                                                     // Hold the existing INT1 routing
  if(!_i2c_bus->readByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL4_INT1_PAD_CTRL, &value)) return false;
  return _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL4_INT1_PAD_CTRL, value | 0x20); // Enable wake-up without changing other routes
}


bool LIS2DW12::deactivateWakeOnMotionInterrupt()
{
  uint8_t value;                                                     // Hold the existing INT1 routing
  if(!_i2c_bus->readByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL4_INT1_PAD_CTRL, &value)) return false;
  return _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL4_INT1_PAD_CTRL, value & ~0x20); // Disable wake-up without changing other routes
}


bool LIS2DW12::Compensation(uint8_t fs, uint8_t odr, uint8_t mode, uint8_t lpMode, uint8_t bw, bool lowNoise, float *offset)
{
  if(!offset) return false;                                          // Reject an invalid destination

  int16_t temp[3] = {0, 0, 0};                                      // Hold one acceleration sample
  int32_t sum[3] = {0, 0, 0};                                       // Accumulate 32 samples without overflow
  uint8_t ctrl6 = (bw << 6) | (fs << 4) | (lowNoise ? 0x04 : 0x00); // Combine bandwidth, scale, and noise settings

  if(!_i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL2, 0x08 | 0x04)) return false; // Enable BDU and register increment
  if(!_i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL3, 0x00)) return false; // Disable self-test and interrupt options
  if(!_i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL4_INT1_PAD_CTRL, 0x00)) return false; // Disable INT1 routing
  if(!_i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL5_INT2_PAD_CTRL, 0x00)) return false; // Disable INT2 routing
  if(!_i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL6, ctrl6)) return false; // Set bandwidth, scale, and noise mode
  if(!_i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL1, odr << 4 | mode << 2 | lpMode)) return false; // Start sampling

  _aRes = 0.000244f * (1 << fs);                                  // Calculate 14-bit acceleration resolution

  if(!waitForDataReady()) return false;                             // Wait for the first complete sample
  if(!readAccelData(temp)) return false;                            // Read and discard the first sample

  for(uint8_t i = 0; i < 32; i++) {
    if(!waitForDataReady()) return false;                           // Wait for the next sample
    if(!readAccelData(temp)) return false;                          // Read all three axes
    sum[0] += temp[0];                                              // Accumulate X-axis offset
    sum[1] += temp[1];                                              // Accumulate Y-axis offset
    sum[2] += temp[2];                                              // Accumulate Z-axis offset
  }

  offset[0] = (float)sum[0] * _aRes / 32.0f;                       // Calculate X-axis offset in g
  offset[1] = (float)sum[1] * _aRes / 32.0f;                       // Calculate Y-axis offset in g
  offset[2] = (float)sum[2] * _aRes / 32.0f;                       // Calculate Z-axis offset in g

  if(offset[2] > +0.5f) offset[2] -= 1.0f;                         // Remove positive gravity from Z
  if(offset[2] < -0.5f) offset[2] += 1.0f;                         // Remove negative gravity from Z

  return true;
}


bool LIS2DW12::reset()
{
  uint8_t value;                                                    // Hold CTRL2 while reset completes
  if(!_i2c_bus->readByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL2, &value)) return false;
  if(!_i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL2, value | 0x40)) return false; // Start software reset

  uint32_t start = millis();                                       // Bound the reset-completion wait

  while(millis() - start < 100) {
    if(!_i2c_bus->readByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL2, &value)) return false;
    if(!(value & 0x40)) return true;                                // Finish when SW_RESET clears
  }

  return false;                                                     // Report a reset timeout
}


bool LIS2DW12::selfTest(float *destination)
{
  if(!destination) return false;                                    // Reject an invalid destination

  int16_t temp[3] = {0, 0, 0};                                     // Hold one acceleration sample
  int32_t positive[3] = {0, 0, 0};                                 // Accumulate positive self-test samples
  int32_t nominal[3] = {0, 0, 0};                                  // Accumulate nominal samples
  const float STres = 0.488f;                                      // Use mg/LSB at 4 g high-performance mode

  if(!_i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL2, 0x08 | 0x04)) return false; // Enable BDU and register increment
  if(!_i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL3, 0x00)) return false; // Disable self-test initially
  if(!_i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL4_INT1_PAD_CTRL, 0x00)) return false; // Disable INT1 routing
  if(!_i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL5_INT2_PAD_CTRL, 0x00)) return false; // Disable INT2 routing
  if(!_i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL6, 0x10)) return false; // Select 4 g and ODR/2 filtering
  if(!_i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL1, 0x40 | 0x04)) return false; // Select 50 Hz high-performance mode

  delay(100);                                                       // Allow the normal output to settle
  if(!waitForDataReady() || !readAccelData(temp)) { disableSelfTest(); return false; } // Discard the first sample

  for(uint8_t i = 0; i < 5; i++) {
    if(!waitForDataReady() || !readAccelData(temp)) { disableSelfTest(); return false; }
    nominal[0] += temp[0];                                         // Accumulate nominal X
    nominal[1] += temp[1];                                         // Accumulate nominal Y
    nominal[2] += temp[2];                                         // Accumulate nominal Z
  }

  if(!_i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL3, 0x40)) { disableSelfTest(); return false; } // Enable positive self-test
  delay(100);                                                       // Allow the self-test output to settle
  if(!waitForDataReady() || !readAccelData(temp)) { disableSelfTest(); return false; } // Discard the first sample

  for(uint8_t i = 0; i < 5; i++) {
    if(!waitForDataReady() || !readAccelData(temp)) { disableSelfTest(); return false; }
    positive[0] += temp[0];                                        // Accumulate self-test X
    positive[1] += temp[1];                                        // Accumulate self-test Y
    positive[2] += temp[2];                                        // Accumulate self-test Z
  }

  destination[0] = (float)(positive[0] - nominal[0]) * STres / 5.0f; // Calculate X response in mg
  destination[1] = (float)(positive[1] - nominal[1]) * STres / 5.0f; // Calculate Y response in mg
  destination[2] = (float)(positive[2] - nominal[2]) * STres / 5.0f; // Calculate Z response in mg

  return disableSelfTest();                                        // Power down and disable self-test
}


bool LIS2DW12::readAccelData(int16_t *destination)
{
  if(!destination) return false;                                   // Reject an invalid destination

  uint8_t rawData[6] = {0};                                       // Hold X, Y, and Z register bytes
  if(!_i2c_bus->readBytes(LIS2DW12_ADDRESS, LIS2DW12_OUT_X_L, 6, rawData)) return false;

  destination[0] = ((int16_t)((uint16_t)rawData[1] << 8 | rawData[0])) >> 2; // Build signed 14-bit X
  destination[1] = ((int16_t)((uint16_t)rawData[3] << 8 | rawData[2])) >> 2; // Build signed 14-bit Y
  destination[2] = ((int16_t)((uint16_t)rawData[5] << 8 | rawData[4])) >> 2; // Build signed 14-bit Z
  return true;
}


bool LIS2DW12::readTempData(int16_t *temperature)
{
  if(!temperature) return false;                                   // Reject an invalid destination

  uint8_t raw;                                                     // Hold the signed eight-bit temperature register
  if(!_i2c_bus->readByte(LIS2DW12_ADDRESS, LIS2DW12_OUT_T, &raw)) return false;
  *temperature = (int8_t)raw;                                      // Sign-extend the raw value to 16 bits
  return true;
}


bool LIS2DW12::readRawTempData(uint8_t *temperature)
{
  return _i2c_bus->readByte(LIS2DW12_ADDRESS, LIS2DW12_OUT_T, temperature); // Return the unmodified register byte
}


bool LIS2DW12::powerDown()
{
  uint8_t value;                                                    // Hold the existing CTRL1 settings
  if(!_i2c_bus->readByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL1, &value)) return false;
  return _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL1, value & 0x0F); // Clear only the ODR bits
}


bool LIS2DW12::powerUp(uint8_t odr)
{
  uint8_t value;                                                    // Hold the existing CTRL1 settings
  if(!_i2c_bus->readByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL1, &value)) return false;
  return _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL1, (value & 0x0F) | (odr << 4)); // Restore the selected ODR
}


bool LIS2DW12::getWakeSource(uint8_t *source)
{
  return _i2c_bus->readByte(LIS2DW12_ADDRESS, LIS2DW12_WAKE_UP_SRC, source); // Read wake-source flags
}


bool LIS2DW12::configureFIFO(uint8_t FIFOMode, uint8_t FIFOThreshold)
{
  uint8_t value = ((FIFOMode & 0x07) << 5) | (FIFOThreshold & 0x1F); // Constrain mode and threshold fields
  return _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_FIFO_CTRL, value); // Apply FIFO configuration
}


bool LIS2DW12::FIFOsamples(uint8_t *samples)
{
  return _i2c_bus->readByte(LIS2DW12_ADDRESS, LIS2DW12_FIFO_SAMPLES, samples); // Read sample count and FIFO flags
}


bool LIS2DW12::waitForDataReady()
{
  uint32_t start = millis();                                       // Bound the data-ready wait
  uint8_t status;                                                   // Hold the current status flags

  while(millis() - start < 1000) {
    if(!getStatus(&status)) return false;                           // Stop on an I2C failure
    if(status & 0x01) return true;                                  // Finish when a new sample is ready
  }

  return false;                                                     // Report a data-ready timeout
}


bool LIS2DW12::disableSelfTest()
{
  bool success = _i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL1, 0x00); // Power down the sensor
  if(!_i2c_bus->writeByte(LIS2DW12_ADDRESS, LIS2DW12_CTRL3, 0x00)) success = false; // Disable self-test
  return success;
}


uint16_t LIS2DW12::FloattoHalf(float f)
{
  uint32_t x;
  memcpy(&x, &f, sizeof(x));

  uint16_t sign = (x >> 16) & 0x8000;
  uint32_t rawExponent = (x >> 23) & 0xFF;
  uint32_t mantissa = x & 0x007FFFFF;

  if(rawExponent == 0xFF)
    return sign | (mantissa ? 0x7E00 : 0x7C00);       // NaN or infinity

  int32_t exponent = (int32_t)rawExponent - 127 + 15;

  if(exponent >= 31)
    return sign | 0x7C00;                             // Finite overflow

  if(exponent <= 0) {
    if(exponent < -10) return sign;                   // Underflow to zero

    mantissa |= 0x00800000;
    uint32_t shift = 14 - exponent;
    uint32_t halfMantissa = mantissa >> shift;
    uint32_t remainder = mantissa & ((1UL << shift) - 1);
    uint32_t halfway = 1UL << (shift - 1);

    if(remainder > halfway ||
       (remainder == halfway && (halfMantissa & 1)))
      halfMantissa++;                                 // Round to nearest even

    return sign | halfMantissa;
  }

  uint32_t halfMantissa = mantissa >> 13;
  uint32_t remainder = mantissa & 0x1FFF;

  if(remainder > 0x1000 ||
     (remainder == 0x1000 && (halfMantissa & 1))) {
    halfMantissa++;                                   // Round to nearest even

    if(halfMantissa == 0x0400) {
      halfMantissa = 0;
      if(++exponent >= 31) return sign | 0x7C00;
    }
  }

  return sign | (exponent << 10) | halfMantissa;
}


float LIS2DW12::HalftoFloat(uint16_t h)
{
  uint32_t sign = (uint32_t)(h & 0x8000) << 16;
  uint32_t exponent = (h >> 10) & 0x1F;
  uint32_t mantissa = h & 0x03FF;
  uint32_t x;

  if(exponent == 0) {
    if(mantissa == 0) {
      x = sign;                                      // Signed zero
    } else {
      int32_t e = -14;                               // Normalize subnormal
      while(!(mantissa & 0x0400)) {
        mantissa <<= 1;
        e--;
      }
      mantissa &= 0x03FF;
      x = sign | ((uint32_t)(e + 127) << 23) | (mantissa << 13);
    }
  } else if(exponent == 0x1F) {
    x = sign | 0x7F800000 | (mantissa << 13);         // Infinity or NaN
  } else {
    x = sign | ((exponent + 112) << 23) | (mantissa << 13);
  }

  float f;
  memcpy(&f, &x, sizeof(f));
  return f;
}
