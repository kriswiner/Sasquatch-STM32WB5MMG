# BLE_Discovery_STM32WB5MMG

`BLE_Discovery_STM32WB5MMG` is a BLE discovery and logging sketch for the Sasquatch STM32WB5MMG board using the TleraCorp STM32WB Arduino core and ST BLE stack. It started as a compact BLE scanner, but is now intended as a small battery-powered BLE environment recorder: it scans nearby BLE advertisers, keeps a fixed-size in-RAM discovery table, periodically writes compact summary records to the board's 16 MByte QSPI flash, and allows the captured binary log to be decoded later with a companion dump sketch.

The scanner is meant for practical BLE device discovery rather than raw packet capture. It tracks up to 48 recently visible devices, including address, address type, connectability, advertisement/scan-response flags, last RSSI, best RSSI, smoothed RSSI, seen count, local name when available, a representative service UUID, and manufacturer ID when present. RSSI smoothing makes proximity estimates less jumpy, and the logged records are sorted by smoothed RSSI so the strongest nearby devices are retained first.

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

## Serial Output

Serial output is optional. This matters for battery-powered operation, because a sketch that waits forever for USB Serial will not run when powered only from a LiPo.

The main controls are near the top of the sketch:

```cpp
const bool SERIAL_DEBUG = false;       // Set false for headless battery logging.
const bool WAIT_FOR_SERIAL = false;    // Leave false so battery-only logging starts immediately.
const bool PRINT_STARTUP_INFO = true;
const bool PRINT_LIVE_SUMMARY = true;
const bool VERBOSE_OUTPUT = false;
```

For untethered logging, use `SERIAL_DEBUG = false` and `WAIT_FOR_SERIAL = false`. For bench testing with the Arduino Serial Monitor, use `SERIAL_DEBUG = true`. If `PRINT_LIVE_SUMMARY` is enabled, the sketch prints a compact summary every 10 seconds showing active devices, RSSI, best RSSI, seen count, address type, `C/A/R` flags, identity hints, battery voltage, and MCU temperature. `VERBOSE_OUTPUT` can be enabled for detailed per-packet advertisement decoding, but it is normally too much output for field use.

Some typical output (excerpted):

BLE discovery analysis
----------------------
Intervals analyzed: 38
Unique addresses:   28
Average devices:    3.5
Peak devices:       15

Ten-minute rollups
start_time,intervals,avg_active,peak_active,strong_obs,medium_obs,weak_obs,apple_obs,st_obs,google_obs,named_obs,public_obs,random_static_obs
12:53:49,38,3.5,15,6,0,127,41,0,0,36,37,96

Manufacturer summary
Apple:        12
ST:           0
Google:       0
Microsoft:    2
Nordic:       0
TI:           0
Other known:  8
Unknown:      6

Persistence summary
Persistent >=80%: 0
Occasional 20-80%: 1
Transient <20%: 27

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

## Suggested Field Workflow

1. Erase QSPI flash.
2. Set `SERIAL_DEBUG = false`, `WAIT_FOR_SERIAL = false`, and `ENABLE_FLASH_LOGGING = true`.
3. Upload `BLE_Discovery_STM32WB5MMG`.
4. Power from a mechanically secure LiPo connection.
5. Place the board so the antenna is not pressed against the body or shielded by metal.
6. Let the device run.
7. Reconnect over USB and upload `BLE_Log_Dump_STM32WB5MMG`.
8. Capture the dump output with the Serial Monitor or a terminal program such as CoolTerm.

For best RF results, use a small plastic case or sleeve, strain-relieve the battery lead, and keep the antenna side facing outward. A bare board in a pocket next to the body is a harsh test: it can stress the battery connector and severely attenuate 2.4 GHz BLE signals.

## Requirements

Before running BLE sketches on a fresh STM32WB board, install the wireless stack using the core's `FWUpdate` example and verify it with `FWInfo`. Open `BLE_Discovery_STM32WB5MMG.ino` in the Arduino IDE, select the appropriate STM32WB5MMG board/core, upload, and use the Serial Monitor at 9600 baud when Serial debug output is enabled.
