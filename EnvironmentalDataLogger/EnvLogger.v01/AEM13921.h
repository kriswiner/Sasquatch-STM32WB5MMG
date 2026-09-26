/*
 * Copyright (c) 2026 Tlera Corp.  All rights reserved.
 *
 * Lightweight AEM13921 energy-harvesting PMIC driver.
 */

#ifndef AEM13921_h
#define AEM13921_h

#include <Arduino.h>
#include "I2Cdev.h"

// The AEM13921 uses the fixed 7-bit I2C address 0x51.
#define AEM13921_ADDRESS 0x51

// Registers used by this deliberately conservative driver.
#define AEM13921_VERSION      0x00
#define AEM13921_SRC1REGU0    0x01
#define AEM13921_SRC1REGU1    0x02
#define AEM13921_SRC2REGU0    0x03
#define AEM13921_SRC2REGU1    0x04
#define AEM13921_VOVDIS       0x05
#define AEM13921_VCHRDY       0x06
#define AEM13921_VOVCH        0x07
#define AEM13921_BST1CFG      0x08
#define AEM13921_BST2CFG      0x09
#define AEM13921_BUCKCFG      0x0A
#define AEM13921_VCHRDYBUCK   0x0B
#define AEM13921_CHG5V        0x0C
#define AEM13921_TEMPCOLDCH   0x0D
#define AEM13921_TEMPHOTCH    0x0E
#define AEM13921_TEMPCOLDDIS  0x0F
#define AEM13921_TEMPHOTDIS   0x10
#define AEM13921_TEMPPROTECT  0x11
#define AEM13921_SRCLOW       0x12
#define AEM13921_APM          0x13
#define AEM13921_APMACC       0x14
#define AEM13921_IRQEN0       0x15
#define AEM13921_IRQEN1       0x16
#define AEM13921_CTRL         0x17
#define AEM13921_IRQFLG0      0x18
#define AEM13921_IRQFLG1      0x19
#define AEM13921_STATUS0      0x1A
#define AEM13921_STATUS1      0x1B
#define AEM13921_APM0SRC1     0x1C
#define AEM13921_APMERR       0x27
#define AEM13921_TEMP         0x28
#define AEM13921_PN0          0xE0

// CTRL register fields.
#define AEM13921_CTRL_UPDATE    0x01
#define AEM13921_CTRL_SYNCBUSY  0x04

// APM register fields.
#define AEM13921_APM_SRC1_ENABLE   0x01
#define AEM13921_APM_SRC2_ENABLE   0x02
#define AEM13921_APM_LOAD_ENABLE   0x04
#define AEM13921_APM_5V_ENABLE     0x08
#define AEM13921_APM_POWER_MODE    0x10
#define AEM13921_APM_116MS_WINDOW  0x20

// IRQEN1 and IRQFLG1 APM fields.
#define AEM13921_IRQ_APM_DONE   0x40
#define AEM13921_IRQ_APM_ERROR  0x80

/*
 * AEM13921 storage/APM test-policy switches.
 *
 * For the present SRC1/SOL solar-input diagnostic:
 * - enable the SRC1 boost converter at its conservative x1 timing,
 * - keep SRC2, the 5 V charger, buck, and optional AEM IRQs disabled,
 * - enable APM so the sketch can distinguish source voltage from useful
 *   harvested power.
 *
 * This should let the real solar cell be tested without adding 5 V charger
 * or buck/load complications.
 */
#ifndef AEM13921_ENABLE_SRC1_BOOST
#define AEM13921_ENABLE_SRC1_BOOST 1
#endif

#ifndef AEM13921_ENABLE_SRC2_BOOST
#define AEM13921_ENABLE_SRC2_BOOST 0
#endif

#ifndef AEM13921_ENABLE_5V_CHARGER
#define AEM13921_ENABLE_5V_CHARGER 0
#endif

#ifndef AEM13921_ENABLE_APM
#define AEM13921_ENABLE_APM 1
#endif

#ifndef AEM13921_ENABLE_BUCK_CONVERTER
#define AEM13921_ENABLE_BUCK_CONVERTER 0
#endif

#ifndef AEM13921_ENABLE_5V_CHANGE_IRQ
#define AEM13921_ENABLE_5V_CHANGE_IRQ 0
#endif

// IRQFLG0 event fields. Reading IRQFLG0 clears these flags.
#define AEM13921_IRQ_I2C_READY       0x01
#define AEM13921_IRQ_OVERDISCHARGE   0x02
#define AEM13921_IRQ_CHARGE_READY    0x04
#define AEM13921_IRQ_OVERCHARGE      0x08
#define AEM13921_IRQ_SOURCE_LOW      0x10
#define AEM13921_IRQ_CHARGE_TEMP     0x20
#define AEM13921_IRQ_DISCHARGE_TEMP  0x40
#define AEM13921_IRQ_5V_CHANGED      0x80

// STATUS0 fields describe current conditions rather than past events.
#define AEM13921_STATUS_OVERDISCHARGED  0x01
#define AEM13921_STATUS_CHARGE_READY    0x02
#define AEM13921_STATUS_OVERCHARGED     0x04
#define AEM13921_STATUS_SOURCE1_LOW     0x08
#define AEM13921_STATUS_SOURCE2_LOW     0x10
#define AEM13921_STATUS_5V_CONNECTED    0x20

// STATUS1 temperature-protection fields.
#define AEM13921_STATUS_CHARGE_COLD     0x01
#define AEM13921_STATUS_CHARGE_HOT      0x02
#define AEM13921_STATUS_DISCHARGE_COLD  0x04
#define AEM13921_STATUS_DISCHARGE_HOT   0x08

struct AEM13921Status
{
  uint8_t status0; // Storage, source and 5 V charger conditions.
  uint8_t status1; // NTC charge/discharge temperature conditions.
};

struct AEM13921InterruptFlags
{
  uint8_t flags0; // Protection, source and 5 V charger events.
  uint8_t flags1; // MPPT, measurement and APM events.
};

struct AEM13921Measurements
{
  uint8_t temperatureRaw;
  uint8_t storageRaw;
  uint8_t source1Raw;
  uint8_t source2Raw;
  float temperatureC;
  float storageVoltage;
  bool temperatureValid;
};

struct AEM13921APMData
{
  uint32_t source1;     // 24-bit APM value from SRC1 to STO.
  uint32_t source2;     // 24-bit APM value from SRC2 to STO.
  uint32_t load;        // 24-bit APM value from STO to LOAD.
  uint16_t charge5V;    // 16-bit 5 V charger duty counter.
  uint8_t error;        // APMERR register.
};

class AEM13921
{
  public:
  AEM13921(I2Cdev* i2c_bus, uint8_t address = AEM13921_ADDRESS);

  /*
   * Verify that the responding device is an AEM13921.
   *
   * This method is read-only. It does not select I2C configuration or alter
   * any charger, storage, boost, buck, MPPT or temperature-protection setting.
   */
  bool begin();

  bool getVersion(uint8_t *version);
  bool getPartNumber(char partNumber[6]);
  bool getControl(uint8_t *control);
  bool configuredByI2C(bool *enabled);
  bool synchronizationBusy(bool *busy);
  bool waitForSynchronization(uint32_t timeoutMs = 100UL);

  bool readRegister(uint8_t reg, uint8_t *value);
  bool writeRegister(uint8_t reg, uint8_t value);
  bool writeAndVerifyRegister(uint8_t reg, uint8_t value);
  bool useGPIOConfiguration(uint32_t timeoutMs = 100UL);
  bool useI2CConfiguration(uint32_t timeoutMs = 100UL);

  /*
   * Conservative first-pass software configuration.
   *
   * This mode intentionally disables both source boost converters through
   * I2C. Per the datasheet, that is one SLEEP CONDITION, so it lets us test
   * whether the AEM13921 can stay I2C-readable without the 300 uA behavior
   * seen in hardware-pin harvesting mode. The 5 V charger remains enabled
   * with CV limiting, thermal protection remains enabled, and APM is enabled.
   */
  bool configureStorageAPMMode(uint32_t timeoutMs = 100UL);

  /*
   * Low-power disable path. This disables optional harvester, measurement and
   * IRQ paths before the MCU requests hardware ship mode.
   */
  bool forceDisable(uint32_t timeoutMs = 100UL);

  // Reading both IRQ flag registers clears the corresponding latched events
  // and releases the active-high IRQ output when no enabled event remains.
  bool readInterruptFlags(AEM13921InterruptFlags *flags);

  bool readStatus(AEM13921Status *status);

  /*
   * Read the NTC, battery and source monitor codes.
   *
   * The daughter board uses RDIV = 22 kohm and a 10 kohm, B = 3380 K NTC.
   * Source monitor codes are preserved raw because their documented transfer
   * function is discontinuous; this avoids hiding information in a lossy
   * approximation during initial board characterization.
   */
  bool readMeasurements(AEM13921Measurements *measurements,
                        float dividerOhms = 22000.0f,
                        float thermistorR0Ohms = 10000.0f,
                        float thermistorBeta = 3380.0f);

  bool readAPMData(AEM13921APMData *data);

  private:
  I2Cdev* _i2c_bus;
  uint8_t _address;
};

#endif
