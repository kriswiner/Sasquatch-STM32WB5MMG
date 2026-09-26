/*
 * Copyright (c) 2026 Tlera Corp.  All rights reserved.
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
 *  3. Neither the name of Tlera Corp, nor the names of its contributors may be
 *     used to endorse or promote products derived from this Software without
 *     specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 */

#ifndef ENS161_h
#define ENS161_h

#include <Arduino.h>
#include "I2Cdev.h"

/*
 * ScioSense ENS161 digital metal-oxide multi-gas sensor
 *
 * Datasheet:
 * https://www.sciosense.com/wp-content/uploads/2024/12/ENS161-Datasheet.pdf
 *
 * This driver deliberately contains no Serial output and no delay().  The
 * main sketch owns presentation, interrupt handling, recovery and policy.
 */

// ENS161 register addresses used by this lightweight driver.
#define ENS161_PART_ID          0x00
#define ENS161_OPMODE           0x10
#define ENS161_CONFIG           0x11
#define ENS161_TEMP_IN          0x13
#define ENS161_RH_IN            0x15
#define ENS161_DEVICE_STATUS    0x20
#define ENS161_DATA_AQI_UBA     0x21

// Daughter-board hardware selects the high address by tying ADDR to VDDIO.
#define ENS161_ADDRESS          0x53
#define ENS161_EXPECTED_PART_ID 0x0161

// Operating modes.  ULP mode is documented in the newer ENS161 application
// notes and current ScioSense Arduino library even though it was absent from
// the early preliminary datasheet.
enum ENS161Mode : uint8_t
{
  ENS161_DEEP_SLEEP_MODE       = 0x00,
  ENS161_IDLE_MODE             = 0x01,
  ENS161_STANDARD_MODE         = 0x02, // one output sample per second
  ENS161_LOW_POWER_MODE        = 0x03, // one output sample per minute
  ENS161_ULTRA_LOW_POWER_MODE  = 0x04, // one output sample every five minutes
  ENS161_RESET_MODE            = 0xF0
};

// DEVICE_STATUS validity field (bits 3:2).
enum ENS161Validity : uint8_t
{
  ENS161_OUTPUT_NORMAL         = 0,
  ENS161_OUTPUT_WARMUP         = 1,
  ENS161_OUTPUT_INITIAL_START  = 2,
  ENS161_OUTPUT_INVALID        = 3
};

// A complete processed data set read from registers 0x21 through 0x27.
struct ENS161Data
{
  uint8_t status;          // Raw DEVICE_STATUS value for diagnostics.
  uint8_t aqiUBA;          // UBA air-quality index, normally 1 through 5.
  uint16_t tvoc;           // Equivalent total VOC concentration in ppb.
  uint16_t eco2;           // Equivalent CO2 concentration in ppm.
  uint16_t aqiScioSense;   // Relative ScioSense air-quality index, 0 to 500.
  ENS161Validity validity; // Normal, warm-up, initial start or invalid.
};

class ENS161
{
  public:
  ENS161(I2Cdev* i2c_bus, uint8_t address = ENS161_ADDRESS);

  /*
   * Begin or restart non-blocking sensor initialization.
   *
   * The requested mode may also be DEEP_SLEEP when a verified shutdown is
   * required for power auditing. Call serviceInitialization() repeatedly until
   * initializationComplete() or initializationFailed() becomes true.  Each
   * service call performs at most one short I2C transaction; the required
   * reset wait is managed with millis() rather than delay().
   */
  bool startInitialization(ENS161Mode mode = ENS161_LOW_POWER_MODE);
  bool serviceInitialization();
  bool initializationComplete() const;
  bool initializationFailed() const;

  // Identification and status helpers used by bring-up and recovery code.
  bool getPartID(uint16_t *partID);
  bool getOperatingMode(uint8_t *mode);
  bool getStatus(uint8_t *status);
  bool dataReady(bool *ready);

  // Issue an immediate deep-sleep command. For a verified startup transition,
  // prefer startInitialization(ENS161_DEEP_SLEEP_MODE) and service it to
  // completion; this direct helper is retained for an already-running sensor.
  bool deepSleep();

  // Configure an active-low, push-pull new-data interrupt. The Sasquatch
  // Daughter v01 D7 connection has no external interrupt pull-up.
  bool configureDataReadyInterrupt();

  // Supply current ambient conditions from the HDC2010 for gas compensation.
  bool writeCompensation(float temperatureC, float relativeHumidity);

  /*
   * Read one processed ENS161 report.
   *
   * A true return value means both I2C transfers completed.  sampleValid is
   * true only for fresh data with normal validity and no sensor error.  This
   * distinction lets the main sketch separate bus faults from expected sensor
   * warm-up or initial-start behavior.
   */
  bool readData(ENS161Data *data, bool *sampleValid);

  private:
  enum InitializationState : uint8_t
  {
    INIT_IDLE,
    INIT_WAIT_RESET,
    INIT_SET_IDLE,
    INIT_READ_PART_ID,
    INIT_CONFIGURE_INTERRUPT,
    INIT_START_MEASUREMENT,
    INIT_VERIFY_MODE,
    INIT_COMPLETE,
    INIT_FAILED
  };

  bool writeOperatingMode(ENS161Mode mode);
  void failInitialization();

  I2Cdev* _i2c_bus;
  uint8_t _address;
  ENS161Mode _requestedMode;
  InitializationState _initializationState;
  uint32_t _readyAt;
};

#endif
