# AS7331 Sasquatch Breadboard Logger

This repository contains a robust, ultra-low-power AS7331 UVA/UVB/UVC demonstration for the Tlera Sasquatch STM32WB5MMG board. The sensor is connected to the external `Wire` I2C bus, with its push-pull READY output connected to GPIO 8. Measurements use AS7331 command mode with gain 8 and integration-time setting 6. Interrupt and timer callbacks only set flags and wake the MCU; the main loop services the resulting state machines and otherwise enters `STM32WB.stop()`. Measured STOP current was approximately 5–6 µA.

The main sketch records one verified, CRC-protected 256-byte QSPI page per measurement without erasing or wrapping the flash. Normal measurements occur every 10 seconds; AS7331 temperatures at or above 50 °C enter a one-minute thermal-recheck interval, with normal operation resuming at or below 45 °C. A 1 ms green heartbeat follows each successful flash write and readback verification, while a steady red LED indicates a latched sensor, timer, or flash failure. Set `serialDebug` to `false` for battery operation, and erase the QSPI flash with a separate erase sketch before starting a clean experiment.

Two outdoor test datasets are included in [`test-data`](test-data). They demonstrated more than four hours of reliable operation, valid sensor status on every stored measurement, correct thermal entry and recovery, and no invalid CRC pages. UVA and UVB produced useful relative environmental time series; the UVC channel is retained for diagnostic purposes and should not be interpreted as terrestrial UVC irradiance. The companion sketch reads the QSPI log without modifying it and emits CSV suitable for plotting or spreadsheet analysis.

## Typical CSV output

```text
session_id,sequence,timestamp,sample,temp_raw,temp_c,uva_raw,uva_uw_cm2,uvb_raw,uvb_uw_cm2,uvc_raw,uvc_nominal_uw_cm2,osr,status,flags,gain,time,clock,divider
# session_page=2
# session_sequence=2
# session_id=228307176
# format_uuid=AS7331-UVLOG-V01
# sample_interval_ms=10000
# thermal_recheck_ms=60000
228307176,3,2026-7-23 14:26:34.90,1,1912,28.70,541,329.67459,65,51.79720,76,29.09432,0x43,0xB,0x1,8,6,0,255
228307176,4,2026-7-23 14:26:44.90,2,1939,30.05,301,183.42339,73,58.17224,93,35.60226,0x43,0xB,0x1,8,6,0,255
228307176,5,2026-7-23 14:26:54.90,3,1974,31.80,8141,4960.96240,77,61.35976,82,31.39124,0x43,0xB,0x1,8,6,0,255
228307176,6,2026-7-23 14:27:4.90,4,1989,32.55,8233,5017.02539,80,63.75040,83,31.77406,0x43,0xB,0x1,8,6,0,255
228307176,7,2026-7-23 14:27:14.90,5,2012,33.70,8315,5066.99463,81,64.54728,83,31.77406,0x43,0xB,0x1,8,6,0,255
228307176,8,2026-7-23 14:27:24.90,6,2031,34.65,8326,5073.69775,81,64.54728,83,31.77406,0x43,0xB,0x1,8,6,0,255
228307176,9,2026-7-23 14:27:34.90,7,2053,35.75,8310,5063.94775,81,64.54728,83,31.77406,0x43,0xB,0x1,8,6,0,255
228307176,10,2026-7-23 14:27:44.90,8,2052,35.70,8305,5060.90088,81,64.54728,83,31.77406,0x43,0xB,0x1,8,6,0,255
228307176,11,2026-7-23 14:27:54.90,9,2057,35.95,8304,5060.29150,81,64.54728,83,31.77406,0x43,0xB,0x1,8,6,0,255
228307176,12,2026-7-23 14:28:4.90,10,2073,36.75,8301,5058.46338,81,64.54728,83,31.77406,0x43,0xB,0x1,8,6,0,255
```

## Example unobstructed outdoor test

<img width="1200" height="820" alt="AS7331_unobstructed_run" src="https://github.com/user-attachments/assets/f6a6f3cb-3eaf-4a1c-8428-d60d80e37b61" />
