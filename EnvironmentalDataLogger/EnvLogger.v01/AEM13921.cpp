/*
 * Lightweight, read-mostly e-peas AEM13921 driver for initial daughter-board
 * bring-up and characterization.
 */

#include "AEM13921.h"
#include <math.h>

// The five part-number registers contain the ASCII string "12931".
static const char AEM13921_EXPECTED_PART_NUMBER[6] = "12931";


AEM13921::AEM13921(I2Cdev* i2c_bus, uint8_t address)
{
  _i2c_bus = i2c_bus;
  _address = address;
}


/* --------------------------------------------------------------------------
 * Identification and configuration-source checks
 * -------------------------------------------------------------------------- */

bool AEM13921::begin()
{
  if(_i2c_bus == nullptr) return false;

  char partNumber[6] = {0, 0, 0, 0, 0, 0};
  if(!getPartNumber(partNumber)) return false;

  for(uint8_t index = 0; index < 5; index++) {
    if(partNumber[index] != AEM13921_EXPECTED_PART_NUMBER[index]) return false;
  }

  // VERSION is not compared with a fixed value so newer silicon revisions
  // remain compatible. Reading it verifies access to the normal register map.
  uint8_t version = 0;
  return getVersion(&version);
}


bool AEM13921::getVersion(uint8_t *version)
{
  if(_i2c_bus == nullptr || version == nullptr) return false;
  return _i2c_bus->readByte(_address, AEM13921_VERSION, version);
}


bool AEM13921::getPartNumber(char partNumber[6])
{
  if(_i2c_bus == nullptr || partNumber == nullptr) return false;

  uint8_t rawData[5] = {0, 0, 0, 0, 0};
  if(!_i2c_bus->readBytes(_address, AEM13921_PN0, 5, rawData)) return false;

  for(uint8_t index = 0; index < 5; index++) partNumber[index] = (char)rawData[index];
  partNumber[5] = '\0';
  return true;
}


bool AEM13921::getControl(uint8_t *control)
{
  return readRegister(AEM13921_CTRL, control);
}


bool AEM13921::configuredByI2C(bool *enabled)
{
  if(enabled == nullptr) return false;

  uint8_t control = 0;
  if(!getControl(&control)) return false;

  *enabled = (control & AEM13921_CTRL_UPDATE) != 0;
  return true;
}


bool AEM13921::synchronizationBusy(bool *busy)
{
  if(busy == nullptr) return false;

  uint8_t control = 0;
  if(!getControl(&control)) return false;

  *busy = (control & AEM13921_CTRL_SYNCBUSY) != 0;
  return true;
}


bool AEM13921::waitForSynchronization(uint32_t timeoutMs)
{
  uint32_t startMs = millis();

  while((millis() - startMs) <= timeoutMs) {
    bool busy = false;
    if(!synchronizationBusy(&busy)) return false;
    if(!busy) return true;
    delay(1);
  }

  return false;
}


bool AEM13921::readRegister(uint8_t reg, uint8_t *value)
{
  if(_i2c_bus == nullptr || value == nullptr) return false;
  return _i2c_bus->readByte(_address, reg, value);
}


bool AEM13921::writeRegister(uint8_t reg, uint8_t value)
{
  if(_i2c_bus == nullptr) return false;
  return _i2c_bus->writeByte(_address, reg, value);
}


bool AEM13921::writeAndVerifyRegister(uint8_t reg, uint8_t value)
{
  uint8_t readback = 0;
  return writeRegister(reg, value) &&
         readRegister(reg, &readback) &&
         readback == value;
}


bool AEM13921::useGPIOConfiguration(uint32_t timeoutMs)
{
  if(!writeRegister(AEM13921_CTRL, 0x00)) return false;
  if(!waitForSynchronization(timeoutMs)) return false;

  bool enabled = true;
  return configuredByI2C(&enabled) && !enabled;
}


bool AEM13921::useI2CConfiguration(uint32_t timeoutMs)
{
  if(!writeRegister(AEM13921_CTRL, AEM13921_CTRL_UPDATE)) return false;
  if(!waitForSynchronization(timeoutMs)) return false;

  bool enabled = false;
  return configuredByI2C(&enabled) && enabled;
}


bool AEM13921::configureStorageAPMMode(uint32_t timeoutMs)
{
  /*
   * This is intentionally not a final solar-harvesting configuration.
   *
   * It is a SRC1/SOL power-diagnostic and commissioning configuration:
   * - VOVDIS near 3.4 V.
   * - VCHRDY and VCHRDYBUCK near 3.6 V.
   * - VOVCH at 4.20 V as a LiPo safety cutoff.
   * - Buck converter disabled because the daughter board does not use LOAD.
   * - 5 V charger disabled so only the solar input is being tested.
   * - SRC1 boost optionally enabled; SRC2 remains disabled.
   * - APM optionally enabled for SRC1, LOAD and 5 V charging.
   * - Optional 5 V-change IRQ disabled unless explicitly requested.
   */
  const uint8_t vovdis_3v394       = 0x35;
  const uint8_t vchrdy_3v600       = 0x3D;
  const uint8_t vovch_4v200        = 0x50;
  const uint8_t bst1_config =
#if AEM13921_ENABLE_SRC1_BOOST
                                  0x01;
#else
                                  0x00;
#endif
  const uint8_t bst2_config =
#if AEM13921_ENABLE_SRC2_BOOST
                                  0x01;
#else
                                  0x00;
#endif
  const uint8_t buck_config =
#if AEM13921_ENABLE_BUCK_CONVERTER
                                  0x30;
#else
                                  0x00;
#endif
  const uint8_t chg5v_config =
#if AEM13921_ENABLE_5V_CHARGER
                                   (0x17 << 2) | 0x02 | 0x01;
#else
                                   0x00;
#endif
  const uint8_t apm_config =
#if AEM13921_ENABLE_APM
                                 AEM13921_APM_SRC1_ENABLE |
                                 AEM13921_APM_LOAD_ENABLE |
                                 AEM13921_APM_5V_ENABLE |
                                 AEM13921_APM_POWER_MODE;
#else
                                 0x00;
#endif
  const uint8_t apm_accumulator =
#if AEM13921_ENABLE_APM
                                       0xFF;
#else
                                       0x00;
#endif
  const uint8_t irqen0_config =
#if AEM13921_ENABLE_5V_CHANGE_IRQ
                                  AEM13921_IRQ_5V_CHANGED;
#else
                                  0x00;
#endif
  const uint8_t irqen1_config =
#if AEM13921_ENABLE_APM
                                  AEM13921_IRQ_APM_DONE | AEM13921_IRQ_APM_ERROR;
#else
                                  0x00;
#endif

  if(!waitForSynchronization(timeoutMs)) return false;

  if(!writeAndVerifyRegister(AEM13921_VOVDIS, vovdis_3v394)) return false;
  if(!writeAndVerifyRegister(AEM13921_VCHRDY, vchrdy_3v600)) return false;
  if(!writeAndVerifyRegister(AEM13921_VCHRDYBUCK, vchrdy_3v600)) return false;
  if(!writeAndVerifyRegister(AEM13921_VOVCH, vovch_4v200)) return false;

  // 0 C to 45 C charge range and -20 C to 65 C discharge range for 10k/3380 NTC.
  if(!writeAndVerifyRegister(AEM13921_TEMPCOLDCH, 0x90)) return false;
  if(!writeAndVerifyRegister(AEM13921_TEMPHOTCH, 0x2E)) return false;
  if(!writeAndVerifyRegister(AEM13921_TEMPCOLDDIS, 0xC6)) return false;
  if(!writeAndVerifyRegister(AEM13921_TEMPHOTDIS, 0x1B)) return false;
  if(!writeAndVerifyRegister(AEM13921_TEMPPROTECT, 0x01)) return false;

  // Enable only the selected harvester path. Keep other power paths quiet.
  if(!writeAndVerifyRegister(AEM13921_BST1CFG, bst1_config)) return false;
  if(!writeAndVerifyRegister(AEM13921_BST2CFG, bst2_config)) return false;
  if(!writeAndVerifyRegister(AEM13921_BUCKCFG, buck_config)) return false;
  if(!writeAndVerifyRegister(AEM13921_CHG5V, chg5v_config)) return false;

  // In strict quiet mode these all write 0. When APM is enabled, APMACC=0xFF
  // wakes the MCU about once per minute instead of every 233 ms.
  if(!writeAndVerifyRegister(AEM13921_APM, apm_config)) return false;
  if(!writeAndVerifyRegister(AEM13921_APMACC, apm_accumulator)) return false;
  if(!writeAndVerifyRegister(AEM13921_IRQEN0, irqen0_config)) return false;
  if(!writeAndVerifyRegister(AEM13921_IRQEN1, irqen1_config)) return false;

  return useI2CConfiguration(timeoutMs);
}


bool AEM13921::forceDisable(uint32_t timeoutMs)
{
  /*
   * Disable optional paths before handing control back to the external
   * ship-mode circuit. Normal active configuration keeps VOVCH at 4.20 V as
   * the LiPo safety cutoff; the previous diagnostic workaround is no longer
   * used.
   */

  if(!waitForSynchronization(timeoutMs)) return false;

  if(!writeAndVerifyRegister(AEM13921_BST1CFG, 0x00)) return false;
  if(!writeAndVerifyRegister(AEM13921_BST2CFG, 0x00)) return false;
  if(!writeAndVerifyRegister(AEM13921_BUCKCFG, 0x00)) return false;
  if(!writeAndVerifyRegister(AEM13921_CHG5V, 0x00)) return false;
  if(!writeAndVerifyRegister(AEM13921_APM, 0x00)) return false;
  if(!writeAndVerifyRegister(AEM13921_APMACC, 0x00)) return false;
  if(!writeAndVerifyRegister(AEM13921_IRQEN0, 0x00)) return false;
  if(!writeAndVerifyRegister(AEM13921_IRQEN1, 0x00)) return false;

  AEM13921InterruptFlags flags;
  if(!readInterruptFlags(&flags)) return false;

  return useI2CConfiguration(timeoutMs);
}


/* --------------------------------------------------------------------------
 * Interrupt and present-condition status
 * -------------------------------------------------------------------------- */

bool AEM13921::readInterruptFlags(AEM13921InterruptFlags *flags)
{
  if(_i2c_bus == nullptr || flags == nullptr) return false;

  // Reading IRQFLG0 and IRQFLG1 clears their latched events. Read both in one
  // transaction so one servicing pass cannot accidentally leave IRQ asserted.
  uint8_t rawData[2] = {0, 0};
  if(!_i2c_bus->readBytes(_address, AEM13921_IRQFLG0, 2, rawData)) return false;

  AEM13921InterruptFlags newFlags;
  newFlags.flags0 = rawData[0];
  newFlags.flags1 = rawData[1];
  *flags = newFlags;
  return true;
}


bool AEM13921::readStatus(AEM13921Status *status)
{
  if(_i2c_bus == nullptr || status == nullptr) return false;

  uint8_t rawData[2] = {0, 0};
  if(!_i2c_bus->readBytes(_address, AEM13921_STATUS0, 2, rawData)) return false;

  AEM13921Status newStatus;
  newStatus.status0 = rawData[0];
  newStatus.status1 = rawData[1];
  *status = newStatus;
  return true;
}


/* --------------------------------------------------------------------------
 * Low-rate monitor data
 * -------------------------------------------------------------------------- */

bool AEM13921::readMeasurements(AEM13921Measurements *measurements,
                                float dividerOhms,
                                float thermistorR0Ohms,
                                float thermistorBeta)
{
  if(_i2c_bus == nullptr || measurements == nullptr) return false;
  if(dividerOhms <= 0.0f || thermistorR0Ohms <= 0.0f || thermistorBeta <= 0.0f)
    return false;

  uint8_t rawData[4] = {0, 0, 0, 0};
  if(!_i2c_bus->readBytes(_address, AEM13921_TEMP, 4, rawData)) return false;

  AEM13921Measurements newMeasurements;
  newMeasurements.temperatureRaw = rawData[0];
  newMeasurements.storageRaw = rawData[1];
  newMeasurements.source1Raw = rawData[2];
  newMeasurements.source2Raw = rawData[3];

  // VSTO = 4.8 V * DATA / 256, per datasheet section 9.23.
  newMeasurements.storageVoltage = 4.8f * (float)rawData[1] / 256.0f;

  newMeasurements.temperatureC = 0.0f;
  newMeasurements.temperatureValid = false;

  // End codes cannot produce a useful finite thermistor resistance.
  if(rawData[0] > 0 && rawData[0] < 255) {
    float code = (float)rawData[0];
    float thermistorOhms = dividerOhms * code / (256.0f - code);
    float kelvin = thermistorBeta /
                   (logf(thermistorOhms / thermistorR0Ohms) +
                    thermistorBeta / 298.15f);

    if(isfinite(kelvin) && kelvin > 0.0f) {
      newMeasurements.temperatureC = kelvin - 273.15f;
      newMeasurements.temperatureValid = true;
    }
  }

  *measurements = newMeasurements;
  return true;
}


bool AEM13921::readAPMData(AEM13921APMData *data)
{
  if(_i2c_bus == nullptr || data == nullptr) return false;

  uint8_t rawData[12] = {0};
  if(!_i2c_bus->readBytes(_address, AEM13921_APM0SRC1, 12, rawData)) return false;

  AEM13921APMData newData;
  newData.source1 = ((uint32_t)rawData[2] << 16) |
                    ((uint32_t)rawData[1] << 8)  |
                     (uint32_t)rawData[0];
  newData.source2 = ((uint32_t)rawData[5] << 16) |
                    ((uint32_t)rawData[4] << 8)  |
                     (uint32_t)rawData[3];
  newData.load = ((uint32_t)rawData[8] << 16) |
                 ((uint32_t)rawData[7] << 8)  |
                  (uint32_t)rawData[6];
  newData.charge5V = ((uint16_t)rawData[10] << 8) | rawData[9];
  newData.error = rawData[11];

  *data = newData;
  return true;
}
