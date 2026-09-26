
The Sasquatch Daughter Board is intended as an update of the [Rodan](https://github.com/kriswiner/Rodan) Environmental Data logger concept. Both use the STM32WB5MMG module as BLE-enabled host MCU. However, unlike the Rodan series (v.01a, v.01b) which is an integrated, single pcb assembly, the Sasquatch Daughter is intended to mount onto the Sasquatch development board and make use of its internal 16 MByte QSPI flash, embedded battery voltage monitor, and LIS2DW12 accelerometer as well as its 3V0 power rail. The Sasquatch   has an APDS9253 RGB iR ambient light sensor, an HDC2010 humidity and temperature sensor, an LPS22DF barometer, and an ENS161 eCO2/TVOC gas sensor. 

In addition to environmental sensors, the daughter board has an ePeas AEM13921 energy harvester for powering the intended [105 mAH LiPo](https://www.adafruit.com/product/1570) battery using a Panasonic [AM-5412CAR](https://www.digikey.com/en/products/detail/panasonic-energy/AM-5412CAR-DGK-T/2165192) amorphous silicon solar cell. The daughter board also has an LC709204F fuel gauge, but we have found this unreliable (prone to I2C latchup) as well as being out of production and completely unsupported; so the fuel gauge is not used in this application.

<img width="460" height="754" alt="SasquatchDaughter Bottom" src="https://github.com/user-attachments/assets/0c9fcb8e-6d1e-4eda-8f9e-9c7ddf32ed86" />
<img width="449" height="752" alt="SasquatchDaughter Top" src="https://github.com/user-attachments/assets/05d8c4ce-021d-4151-bfff-7640d6c397dd" />

*View of the Sasquatch Daughter board bottom and top from Altium 3D view*

<img width="3014" height="1789" alt="SasquatchEnvLogger" src="https://github.com/user-attachments/assets/bddee9c3-16e7-47b2-9474-25d512cbc7c4" />

*Complete Environmental Logger with Daughter board mounted onto the Sasquatch Development board, AM5412 solar cell soldered to the SRC1+/- of the AEM13921, and power being supplied and current measured by the Nordic Power Profiler Kit II.*

The STM32WB5 MCU communicates with all four Daughter board sensors and the ePeas AEM13921 via its external I2C bus. The on-Sasquatch LIS2DW12 accelerometer is on the internal I2C bus. 

The logger treats the sensor array as a set of low-duty-cycle measurement devices rather than continuously powered data streams. On each logging interval, the firmware starts the required one-shot measurements, waits only as long as needed for conversion to complete, reads each sensor in a deterministic sequence over I2C, and then returns devices to their low-power state where supported. Environmental data from the HDC2010 humidity/temperature sensor, LPS22DF pressure sensor, APDS9253 RGB/IR/light sensor, ENS161 gas sensor, and LIS2DW12 accelerometer are collected into a single timestamped record and written to QSPI flash. The code also preserves a compact quality byte for each record so occasional stale or partial readings can be identified without cluttering the primary data log. This approach minimizes average current while still producing complete, synchronized environmental snapshots suitable for long-duration field deployment.

The AEM13921 energy harvester is managed as an explicitly controlled charging subsystem rather than being left continuously active. The firmware normally holds the AEM in its lowest-power disabled/ship state and wakes it only when the battery voltage is below the selected charging threshold and the light sensor indicates useful illumination. After enabling the AEM, the firmware configures it over I2C, monitors the SRC1 available-power measurement and storage voltage, and keeps harvesting active only when the source appears strong enough to provide useful charge. If the battery reaches the upper voltage limit, illumination is too weak, or harvesting does not appear productive after a short evaluation period, the firmware returns the AEM to ship mode and waits before trying again. This policy avoids paying the active harvester current cost during dim or unproductive conditions while still allowing the logger to maintain its battery from ordinary solar exposure.

P0 — AEM off/idle. Battery does not need charging, or policy is waiting.  
P1 — Wake requested. Conditions look favorable, so the firmware is enabling the AEM.  
P2 — Evaluating. AEM is awake; firmware is checking whether harvesting is actually useful.  
P3 — Charging. AEM remains active because source power appears useful.  
P4 — Storage high. AEM disabled because battery/storage voltage is at the upper limit.  
P5 — Light too low. AEM disabled because ambient light is below the policy threshold.  
P6 — No useful harvest. AEM disabled because source power was insufficient after evaluation.  
P7 — Unavailable/fault. AEM did not respond or could not be configured; retry later.  

*AEM13921 chargin policy enforced by the STM32WB5 MCU*

*<img width="1056" height="575" alt="SasquatchDaughter BLEAEM 091726" src="https://github.com/user-attachments/assets/c7620943-6bd2-478b-95a6-c7fd11874fba" />
