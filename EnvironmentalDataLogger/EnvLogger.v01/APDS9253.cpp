/******************************************************************************
 *
 * Copyright (c) 2021 Tlera Corporation  All rights reserved.
 *
 * This library is open-source and freely available for all to use with attribution.
 * 
 * All rights reserved.
 *****************************************************************************
 */
#include "APDS9253.h"
#include "I2Cdev.h"


APDS9253::APDS9253(I2Cdev* i2c_bus)
{
  _i2c_bus = i2c_bus;
}

// set bit 4 to 1 for software reset
bool APDS9253::reset()
{
   uint8_t temp;
   if(!_i2c_bus->readByte(APDS9253_ADDR, APDS9253_MAIN_CTRL, &temp)) return false;
   return _i2c_bus->writeByte(APDS9253_ADDR, APDS9253_MAIN_CTRL, temp | 0x10);
}

 
bool APDS9253::getChipID(uint8_t *chipID)
{
   return _i2c_bus->readByte(APDS9253_ADDR, APDS9253_PART_ID, chipID);
}


bool APDS9253::init(uint8_t RGBmode, uint8_t LS_res, uint8_t LS_rate, uint8_t LS_gain)
{
   if(!_i2c_bus->writeByte(APDS9253_ADDR, APDS9253_MAIN_CTRL, RGBmode << 2)) return false;
   if(!_i2c_bus->writeByte(APDS9253_ADDR, APDS9253_LS_MEAS_RATE, (LS_res << 4) | LS_rate)) return false;
   if(!_i2c_bus->writeByte(APDS9253_ADDR, APDS9253_LS_GAIN, LS_gain)) return false;
   return _i2c_bus->writeByte(APDS9253_ADDR, APDS9253_INT_CFG, 0x10); // select green channel but leave its interrupt disabled
}


bool APDS9253::enable()
{
   uint8_t temp;
   if(!_i2c_bus->readByte(APDS9253_ADDR, APDS9253_MAIN_CTRL, &temp)) return false;
   if(!_i2c_bus->writeByte(APDS9253_ADDR, APDS9253_MAIN_CTRL, temp & ~(0x02))) return false; // clear LS_EN bit
   return _i2c_bus->writeByte(APDS9253_ADDR, APDS9253_MAIN_CTRL, temp | 0x02);               // set LS_EN bit
}


bool APDS9253::disable()
{
   uint8_t temp;
   if(!_i2c_bus->readByte(APDS9253_ADDR, APDS9253_MAIN_CTRL, &temp)) return false;
   if(!_i2c_bus->writeByte(APDS9253_ADDR, APDS9253_MAIN_CTRL, temp & ~(0x02))) return false;
   bool disabled = false;
   return isDisabled(&disabled) && disabled;
}


bool APDS9253::isDisabled(bool *disabled)
{
   if(disabled == nullptr) return false;
   uint8_t mainControl = 0;
   if(!_i2c_bus->readByte(APDS9253_ADDR, APDS9253_MAIN_CTRL, &mainControl)) return false;
   *disabled = (mainControl & 0x02) == 0; // LS_EN must remain clear.
   return true;
}


bool APDS9253::getRGBiRdata(uint32_t *destination)
{
  if(!destination) return false;

  uint8_t rawData[12] = {0};
  if(!_i2c_bus->readBytes(APDS9253_ADDR, APDS9253_LS_DATA_IR_0, 12, rawData)) return false; // one coherent RGB+IR conversion

  destination[3] = (((uint32_t) (rawData[2]  & 0x0F)) << 16) | (((uint32_t) rawData[1])  << 8) | rawData[0];  // ir
  destination[1] = (((uint32_t) (rawData[5]  & 0x0F)) << 16) | (((uint32_t) rawData[4])  << 8) | rawData[3];  // green
  destination[2] = (((uint32_t) (rawData[8]  & 0x0F)) << 16) | (((uint32_t) rawData[7])  << 8) | rawData[6];  // blue
  destination[0] = (((uint32_t) (rawData[11] & 0x0F)) << 16) | (((uint32_t) rawData[10]) << 8) | rawData[9];  // red
  return true;
}


bool APDS9253::getStatus(uint8_t *status)
{
    return _i2c_bus->readByte(APDS9253_ADDR, APDS9253_MAIN_STATUS, status);
}
