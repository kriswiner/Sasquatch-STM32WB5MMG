
The Sasquatch Daughter Board is intended as an update of the [Rodan](https://github.com/kriswiner/Rodan) Environmental Data logger concept. Both use the STM32WB5MMG module as BLE-enabled host MCU. However, unlike the Rodan series (v.01a, v.01b) which is an integrated, single pcb assembly, the Sasquatch Daughter is intended to mount onto the Sasquatch development board and make use of its internal 16 MByte QSPI flash, embedded battery voltage monitor, and LIS2DW12 accelerometer as well as its 3V0 power rail. The Sasquatch   has an APDS9253 RGB iR ambient light sensor, an HDC2010 humidity and temperature sensor, an LPS22DF barometer, and an ENS161 eCO2/TVOC gas sensor. The daughter board also has the LC709204F fuel gauge, but we have found this unreliable (prone to I3C latchup) as wll as out of production; so the fuel gauge is not used in this application. In addition, the daughter board has an ePeas AEM13921 energy harvester for powering the intended 105 mAH LiPo battery using a Panasonic [AM-5412CAF](https://www.digikey.com/en/products/detail/panasonic-energy/AM-5412CAR-DGK-T/2165192) amorphous silicon solar cell. 

<img width="460" height="754" alt="SasquatchDaughter Bottom" src="https://github.com/user-attachments/assets/0c9fcb8e-6d1e-4eda-8f9e-9c7ddf32ed86" />
<img width="449" height="752" alt="SasquatchDaughter Top" src="https://github.com/user-attachments/assets/05d8c4ce-021d-4151-bfff-7640d6c397dd" />

*View of the Sasquatch Daughter Board bottom and top from Altium 3D view*

<img width="3014" height="1789" alt="SasquatchEnvLogger" src="https://github.com/user-attachments/assets/bddee9c3-16e7-47b2-9474-25d512cbc7c4" />

*Complete Environmental Logger with Daughter board mounted onto the Sasquatch Development board, AM5412 solar cell soldered to the SRC+/- of the AEM13921, and power being supplied and current measured bu the Nordic Power Profiler Kit II.*

The STM32WB5 MCU communicated with all four sensors and the ePeas AEM13921 via its external I2C bus. The on-Sasquatch LIS2DW12 accelerometer is on the internal I2C bus. 
