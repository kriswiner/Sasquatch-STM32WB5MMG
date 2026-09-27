
The Sasquatch Daughter Board is intended as an update of the [Rodan](https://github.com/kriswiner/Rodan) Environmental Data logger concept. Both use the STM32WB5MMG module as BLE-enabled host MCU. However, unlike the Rodan series (v.01a, v.01b) which is an integrated, single pcb assembly, the Sasquatch Daughter is intended to mount onto the Sasquatch development board and make use of its internal 16 MByte QSPI flash, embedded battery voltage monitor, and LIS2DW12 accelerometer as well as its 3V0 power rail. The Sasquatch   has an APDS9253 RGBIR ambient light sensor, an HDC2010 humidity and temperature sensor, an LPS22DF barometer, and an ENS161 eCO2/TVOC gas sensor. 

In addition to environmental sensors, the daughter board has an ePeas AEM13921 energy harvester for powering the intended [105 mAH LiPo](https://www.adafruit.com/product/1570) battery using a Panasonic [AM-5412CAR](https://www.digikey.com/en/products/detail/panasonic-energy/AM-5412CAR-DGK-T/2165192) amorphous silicon solar cell (or 5V USB). The daughter board also has an LC709204F fuel gauge, but we have found this unreliable (prone to I2C latchup) as well as being out of production and completely unsupported; so the fuel gauge is not used in this application.

<img width="460" height="754" alt="SasquatchDaughter Bottom" src="https://github.com/user-attachments/assets/0c9fcb8e-6d1e-4eda-8f9e-9c7ddf32ed86" />
<img width="449" height="752" alt="SasquatchDaughter Top" src="https://github.com/user-attachments/assets/05d8c4ce-021d-4151-bfff-7640d6c397dd" />

*View of the Sasquatch Daughter board bottom and top from Altium 3D view*

<img width="3014" height="1789" alt="SasquatchEnvLogger" src="https://github.com/user-attachments/assets/bddee9c3-16e7-47b2-9474-25d512cbc7c4" />

*Complete Environmental Logger with Daughter board mounted onto the Sasquatch Development board, AM5412 solar cell soldered to the SRC1+/- of the AEM13921, and power being supplied and current measured by the Nordic Power Profiler Kit II.*

The STM32WB5 MCU communicates with all four Daughter board sensors and the ePeas AEM13921 via its external I2C bus. The on-Sasquatch LIS2DW12 accelerometer is on the internal I2C bus. 

The logger treats the sensors as a set of low-duty-cycle measurement devices rather than continuously powered data streams. On each logging interval (typically once per minute), the firmware starts the required one-shot measurements, waits only as long as needed for conversion to complete, reads each sensor in a deterministic sequence over I2C, and then returns devices to their lowest-power state where supported. Environmental data from the HDC2010 humidity/temperature sensor, LPS22DF pressure sensor, APDS9253 RGB/IR/light sensor, ENS161 gas sensor, and LIS2DW12 accelerometer are collected into a single timestamped record and written to QSPI flash. The code also preserves a compact quality byte for each record so occasional stale or partial readings can be identified without cluttering the primary data log. This approach minimizes average current while still producing complete, synchronized environmental snapshots suitable for long-duration field deployment.

The QSPI log uses fixed 256-byte pages with a 32-byte page header and four 56-byte records per page. Each record is a compact, timestamped environmental snapshot containing: temperature, humidity, pressure, gas readings, RGB/IR/lux, battery voltage, AEM charging telemetry, acceleration, AEM policy state, and one quality byte. Records are appended to the last written record so the logger can be interrupted, if necessary, or multiple sessions can be recorded without losing data. At a once-per-minute data logging cadence, the 16 MByte QSPI flash can hold ~180 days worth of environmental data. At once-per-five minute cadence, which matches the lowest-power rate of the ENS161 gas sensor and is adequate for slowly varying environmental conditions, this duration can be extended out to years. A helper sketch reads the flash after an experiment concludes, translates the logged data into proper units, and outputs the data in CSV format for easy import into a spreadsheet for display and analysis.

Sasquatch Daughter QSPI CSV dump  
quality_bits: bit0=primary_env_partial, bit1=ens_not_valid, bit2=battery_invalid, bit3=aem_unavailable, bit4=i2c_warning, bit5=flash_warning, bit6=reserved, bit7=reserved  
sequence,timestamp_utc,quality,hdc_temp_c,humidity_pct,pressure_hpa,lps_temp_c,ens_aqi,tvoc_ppb,eco2_ppm,red,green,blue,ir,ambient_lux,battery_v,vsto_v,aem_temp_c,aem_apm_src1_k,accel_x_g,accel_y_g,accel_z_g,aem_policy  
0,09/26/2026 18:03:00,0x02,22.25,58.11,977.683,22.65,0,0,0,4,7,3,5,15.12,3.992,0.000,0.0,0,0.6507,-0.1352,-0.7261,5  
1,09/26/2026 18:04:00,0x02,21.66,56.83,977.662,22.20,0,0,400,2,4,2,1,8.64,3.973,0.000,0.0,0,0.6703,-0.2006,-0.7549,5  
2,09/26/2026 18:05:00,0x0A,21.12,59.69,977.953,21.64,0,0,400,149,254,161,385,548.64,3.973,0.000,0.0,0,0.5029,-0.9926,-0.0576,1  
3,09/26/2026 18:06:00,0x0A,20.37,61.49,978.421,20.99,0,0,400,177,323,263,185,697.68,3.975,0.000,0.0,0,-0.0256,-1.0831,0.1032,2  
4,09/26/2026 18:07:00,0x02,19.90,63.09,978.500,20.50,0,0,400,99,182,129,215,393.12,3.968,4.068,20.8,3648,0.0966,0.1789,-0.9543,3  
5,09/26/2026 18:08:00,0x02,19.86,63.43,978.486,20.38,0,0,400,98,179,127,213,386.64,3.966,4.068,20.8,3933,0.1088,0.1835,-0.9540,3  
6,09/26/2026 18:09:00,0x02,19.82,63.45,978.480,20.30,1,0,400,97,178,126,213,384.48,3.966,4.012,20.4,3667,0.1147,0.1896,-0.9536,3  
7,09/26/2026 18:10:00,0x02,19.53,64.87,978.464,20.08,1,0,400,97,177,126,213,382.32,3.975,4.068,20.4,3665,0.1218,0.1947,-0.9465,3  
8,09/26/2026 18:11:00,0x02,19.19,64.90,978.433,19.88,1,0,400,97,177,125,214,382.32,3.973,4.050,20.4,3932,0.1286,0.1984,-0.9487,3  
9,09/26/2026 18:12:00,0x02,19.29,66.54,978.407,19.85,1,0,400,98,177,125,215,382.32,3.964,4.031,19.9,3933,0.1286,0.1957,-0.9453,3  
10,09/26/2026 18:13:00,0x02,19.42,65.48,978.394,19.92,1,0,400,98,177,125,215,382.32,3.957,4.050,20.4,3964,0.1413,0.2047,-0.9389,3  
11,09/26/2026 18:14:00,0x02,19.53,64.85,978.410,20.02,1,0,400,98,178,125,214,384.48,3.966,4.050,19.9,4001,0.1427,0.2069,-0.9416,3  
12,09/26/2026 18:15:00,0x02,19.51,64.98,978.385,20.03,1,0,400,96,176,124,214,380.16,3.966,4.068,20.4,3976,0.1493,0.2037,-0.9379,3  
13,09/26/2026 18:16:00,0x02,19.47,65.43,978.389,19.98,1,0,400,97,177,124,216,382.32,3.966,4.068,20.4,3953,0.1510,0.2142,-0.9345,3  
14,09/26/2026 18:17:00,0x02,19.38,64.59,978.391,19.89,1,0,400,98,177,124,216,382.32,3.966,4.050,19.9,3939,0.1535,0.2123,-0.9409,3  
15,09/26/2026 18:18:00,0x02,19.30,65.42,978.419,19.82,1,0,400,97,177,124,215,382.32,3.964,4.068,19.9,3966,0.1635,0.2057,-0.9421,3  
16,09/26/2026 18:19:00,0x02,19.23,64.99,978.408,19.75,1,0,400,97,176,124,216,380.16,3.964,4.031,19.9,3940,0.1713,0.2159,-0.9289,3  
17,09/26/2026 18:20:00,0x02,19.18,66.20,978.427,19.75,1,0,400,97,177,122,216,382.32,3.968,4.050,19.9,3939,0.1774,0.2191,-0.9287,3  
18,09/26/2026 18:21:00,0x02,19.17,65.14,978.422,19.75,1,0,400,98,177,122,218,382.32,3.968,4.068,19.9,3938,0.1874,0.2333,-0.9235,3  
19,09/26/2026 18:22:01,0x02,19.22,66.30,978.434,19.80,1,0,400,97,175,120,217,378.00,3.966,4.068,19.9,3939,0.2142,0.2440,-0.9179,3  

*Typical QSPI logging data in CSV format from the serial monitor output*

The AEM13921 energy harvester is managed as an explicitly controlled charging subsystem rather than being left continuously active. The firmware normally holds the AEM in its lowest-power disabled/ship state and wakes it only when the battery voltage is below the selected charging threshold and the light sensor indicates useful illumination. After enabling the AEM, the firmware configures it over I2C, monitors the SRC1 available-power measurement and storage voltage, and keeps harvesting active only when the source appears strong enough to provide useful charge. If the battery reaches the upper voltage limit, illumination is too weak, or harvesting does not appear productive after a short evaluation period, the firmware returns the AEM to ship mode and waits before trying again. This policy avoids paying the active harvester current cost during dim or unproductive conditions while still allowing the logger to maintain its battery from ordinary solar exposure.

P0 — AEM off/idle. Battery does not need charging, or policy is waiting.  
P1 — Wake requested. Conditions look favorable, so the firmware is enabling the AEM.  
P2 — Evaluating. AEM is awake; firmware is checking whether harvesting is actually useful.  
P3 — Charging. AEM remains active because source power appears useful.  
P4 — Storage high. AEM disabled because battery/storage voltage is at the upper limit.  
P5 — Light too low. AEM disabled because ambient light is below the policy threshold.  
P6 — No useful harvest. AEM disabled because source power was insufficient after evaluation.  
P7 — Unavailable/fault. AEM did not respond or could not be configured; retry later.  

*AEM13921 charging policy enforced by the STM32WB5 MCU*

<img width="1056" height="575" alt="SasquatchDaughter BLEAEM 091726" src="https://github.com/user-attachments/assets/c7620943-6bd2-478b-95a6-c7fd11874fba" />

*Five days of AEM13921 logging data showing the AEM charging state transitions (active harvesting about half the time) and battery voltage, which stays within a narrow range despite being located in partial shade where direct sun is available for only a few hours each day*

Any or all of the sensors as well as the AEM13921 and BLE can be enabled via switches at the top of the Arduino Environmental Logger sketch. Enabling BLE on-demand reporting is convenient for checking on the deployed enironmental logger to make sure it is still functioning, and to compare the charging state with the light conditions, and to check the charge of the battery, etc. The BLE NUS service is advertised for a 3-second window every two minutes to conserve power; on-demand BLE uses ~100 uA on average, which is a significant part of the total ~150 uA average current usage of the sensors + Sasquatch. So far, the ~250 uA power usage of the sensors + Sasquatch + on-demand BLE is still low enough that the AEM harvester can keep the battery more or less fully charged indefinitely even with partial, intermittent sun exposure on the solar cell. We use Adafruit's BlueFruit NUS console app on the smartphone to query the device status via BLE:

<temp>C,<humidity>%,<pressure>hPa,<lux>lx,AQI<aqi>,<tvoc>ppb,<eco2>ppm  
<battery>V,<rsoc>%,<vsto>V,<src1_apm_k>K,<CHG|noCHG>,0x<status0>,P<aem_policy>  

22.4C,58.1%,982.7hPa,1432lx,AQI1,12ppb,421ppm  
4.01V,0%,4.08V,5230K,CHG,0x82,P3  

*Typical on-demand BLE report available on the smartphone*


