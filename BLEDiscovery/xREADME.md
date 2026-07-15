# BLE_Discovery_STM32WB5MMG

`BLE_Discovery_STM32WB5MMG` is a BLE discovery and logging sketch for the Sasquatch STM32WB5MMG board using the TleraCorp STM32WB Arduino core and ST BLE stack. It started as a compact BLE scanner, but is now intended as a small battery-powered BLE environment recorder: it scans nearby BLE advertisers, keeps a fixed-size in-RAM discovery table, periodically writes compact summary records to the board's 16 MByte QSPI flash, and allows the captured binary log to be decoded later with a companion dump sketch.

The scanner is meant for practical BLE device discovery rather than raw packet capture. It tracks up to 48 recently visible devices, including address, address type, connectability, advertisement/scan-response flags, last RSSI, best RSSI, smoothed RSSI, seen count, local name when available, a representative service UUID, and manufacturer ID when present. RSSI smoothing makes proximity estimates less jumpy, and the logged records are sorted by smoothed RSSI so the strongest nearby devices are retained first.

The sketch separates fast BLE observation from slower flash logging. BLE reports are consumed whenever `BLE.report()` has a packet available, and each report updates the in-RAM discovery table immediately. The 10-second summary timer does not collect packets itself; it simply snapshots the current table to QSPI flash. This keeps the log compact: it records what the BLE environment looked like during each interval rather than attempting to save every advertisement packet.

## Scanning Behavior

The sketch supports both passive and active scanning. Passive scanning listens without sending scan requests, which reduces the scanner's RF visibility but may reveal less information. Active scanning can collect scan-response data, which may include extra names or service data. The summary and log records use simple flags:

- `C`: connectable
- `A`: advertisement seen
- `R`: scan response seen

For richer discovery, use:

```cpp
const bool PASSIVE_SCAN = false;
```

For a quieter scanner, use passive mode:

```cpp
const bool PASSIVE_SCAN = true;
```

The BLE scan duty cycle is also exposed near the top of the sketch:

```cpp
// BLE scan timing uses 0.625 ms units. Duty cycle = window / interval.
const uint16_t BLE_SCAN_INTERVAL = 0x00A0;      // 100 ms interval, 50 ms window = 50% duty cycle.
const uint16_t BLE_SCAN_WINDOW = 0x0050;
// const uint16_t BLE_SCAN_INTERVAL = 0x0320;   // 500 ms interval, 50 ms window = 10% duty cycle.
// const uint16_t BLE_SCAN_WINDOW = 0x0050;
// const uint16_t BLE_SCAN_INTERVAL = 0x0640;   // 1000 ms interval, 50 ms window = 5% duty cycle.
// const uint16_t BLE_SCAN_WINDOW = 0x0050;
```

The default values match the TleraCorp STM32WB BLE library defaults and produce a 50% scan duty cycle. BLE discovery is probabilistic: nearby devices advertise periodically, with timing that the scanner does not control, while the scanner only listens during its scan windows. Discovery therefore depends on advertiser interval, scanner interval, scanner window, RF conditions, and a bit of timing luck. Longer scan intervals or shorter scan windows can reduce average current, but they also increase discovery latency and the chance of missing brief or infrequent advertisers. This tradeoff is discussed in the Bluetooth LE advertising/discovery overview and in BLE neighbor-discovery latency work such as Kindt et al.'s [Neighbor Discovery Latency in BLE-Like Protocols](https://arxiv.org/abs/1509.04366) and [Optimizing BLE-Like Neighbor Discovery](https://arxiv.org/abs/2009.04199).

For slow human-carried device discovery, a lower scan duty cycle may be nearly as effective as the default while saving substantial power. The included 50%, 10%, and 5% examples are starting points for empirical testing. Compare similar field runs by looking at unique addresses, average and peak active devices, persistence categories, and whether the final device identity table still feels rich enough for the application.

## Serial Output

Serial output is optional. This matters for battery-powered operation, because a sketch that waits forever for USB Serial will not run when powered only from a LiPo.

The main controls are near the top of the sketch:

```cpp
const bool ENABLE_FLASH_LOGGING = true;         // Append compact discovery summaries to QSPI flash.
const bool SERIAL_DEBUG = false;                // Set false for headless battery logging.
const bool PRINT_LIVE_SUMMARY = false;          // Useful on the bench; off for quiet field logging.
const bool PASSIVE_SCAN = false;                // False enables active scan and scan-response data.
const bool VERBOSE_OUTPUT = false;              // Set true for full decoded packet output over Serial.
const uint16_t BLE_SCAN_INTERVAL = 0x00A0;      // 100 ms interval, 50 ms window = 50% duty cycle.
const uint16_t BLE_SCAN_WINDOW = 0x0050;
```

For untethered logging, use `SERIAL_DEBUG = false`; the sketch does not wait for USB Serial before starting. For bench testing with the Arduino Serial Monitor, use `SERIAL_DEBUG = true`. If `PRINT_LIVE_SUMMARY` is enabled, the sketch prints a compact summary every 10 seconds showing active devices, RSSI, best RSSI, seen count, address type, `C/A/R` flags, identity hints, battery voltage, and MCU temperature. `VERBOSE_OUTPUT` can be enabled for detailed per-packet advertisement decoding, but it is normally too much output for field use.

## LED Indication

The Sasquatch RGB LED is used as the basic field indication system. The LED is active-low on this board.

- Blue 1 ms blink: a BLE report was received.
- Green 1 ms blink: the 10-second summary/log interval fired and battery voltage is above the low threshold.
- Red 100 ms blink: the summary/log interval fired and battery voltage is below `3.60 V`.

These indications are especially useful when Serial is disabled. Blue blinking confirms BLE activity; green/red summary blinks confirm that the timed logging loop is still alive.

## Flash Logging

When enabled, the sketch appends binary log pages to QSPI flash:

```cpp
const bool ENABLE_FLASH_LOGGING = true;
```

The log is append-only and page based. A session page is written at startup, then every 10 seconds the scanner writes one or more device pages containing the strongest active devices. Each 256-byte device page holds up to 10 compact device records, so a full 48-device summary uses up to 5 pages. Every 5 minutes, an environment checkpoint page records battery voltage and MCU temperature. The current environmental interval is:

```cpp
const unsigned long ENVIRONMENT_INTERVAL_MS = 300000;
```

The log format includes page numbers, sequence numbers, marker bytes, and CRC checks so the dump sketch can stop cleanly at the first erased page or detect a damaged page. Because logging is append-only, run a flash erase sketch before a clean field test unless you intentionally want to append a new session after previous data.

## Dump And Analysis

Use `BLE_Log_Dump_STM32WB5MMG` to read the binary flash log and print human-readable output over Serial at 115200 baud. The dump sketch can print raw reconstructed rows, an analysis summary, or both:

```cpp
const bool PRINT_RAW_ROWS = true;
const bool PRINT_ANALYSIS_SUMMARY = true;
```

Raw rows show each logged device record with batch number, address, type, flags, RSSI values, age, seen count delta, manufacturer ID, UUID hash, and name hash. The analysis summary distills the log into higher-value metadata:

- intervals analyzed
- unique addresses seen
- average and peak active devices
- readable ten-minute rollups with zero-valued fields suppressed
- manufacturer summary
- persistence summary
- device identity table with first/last seen time, seen percentage, RSSI statistics, and proximity class

The current dump sketch reads the append-only log successfully across resets, but long logs with multiple startup session pages should be interpreted with care until the reader is made fully session-aware. Multiple session pages are useful because they reveal resets or power interruptions, but later session pages can complicate timestamp reconstruction in the current analysis output.

Typical output:

<img width="1080" height="851" alt="Output1" src="https://github.com/user-attachments/assets/0d066b6d-c301-4aa2-b92a-11348ce8eff3" />
<img width="1076" height="710" alt="Output2" src="https://github.com/user-attachments/assets/95c55624-1e92-4eee-b507-04d4a8309157" />

## Suggested Field Workflow

1. Erase QSPI flash.
2. Set `SERIAL_DEBUG = false` and `ENABLE_FLASH_LOGGING = true`.
3. Upload `BLE_Discovery_STM32WB5MMG`.
4. Power from a mechanically secure LiPo connection.
5. Place the board so the antenna is not pressed against the body or shielded by metal.
6. Let the device run.
7. Reconnect over USB and upload `BLE_Log_Dump_STM32WB5MMG`.
8. Capture the dump output with the Serial Monitor or a terminal program such as CoolTerm.

For best RF results, use a small plastic case or sleeve, strain-relieve the battery lead, and keep the antenna side facing outward. A bare board in a pocket next to the body is a harsh test: it can stress the battery connector and severely attenuate 2.4 GHz BLE signals.

## Requirements

Before running BLE sketches on a fresh STM32WB board, install the wireless stack using the core's `FWUpdate` example and verify it with `FWInfo`. Open `BLE_Discovery_STM32WB5MMG.ino` in the Arduino IDE, select the appropriate STM32WB5MMG board/core, upload, and use the Serial Monitor at 9600 baud when Serial debug output is enabled.
