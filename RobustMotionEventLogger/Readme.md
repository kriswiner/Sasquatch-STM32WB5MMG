 Robust Sasquatch low-power motion-event logger.

  This application uses the LIS2DW12 accelerometer to watch for motion while the
  STM32WB5MMG remains in its lowest-power STANDBY mode. A motion threshold event
  wakes the MCU, which reads the accelerometer FIFO, converts the XYZ samples to
  compact half-floats, adds the wake source, sensor configuration, RTC time,
  temperature, and battery voltage, and writes the complete event to one 256-byte
  QSPI flash page. The MCU then returns to STANDBY, while the accelerometer waits
  for inactivity before rearming the next motion event.

  The sketch is designed to preserve previously collected data across ordinary
  STANDBY wakes, battery changes, and unexpected resets. SRAM2 normally retains
  the next flash page number so each wake avoids an energy-consuming flash search.
  If that retained state is lost or invalid, a bounded binary search locates the
  first erased page and resumes appending without erasing earlier records. Each
  record contains its physical page number, format version, completion marker,
  and CRC-16 so incomplete or corrupted pages are rejected during recovery and
  readback.

  Startup is treated as an operator-supervised commissioning test. The sketch
  verifies sensor communication, reset, self-test limits, offset calibration,
  motion and FIFO configuration, and flash availability before deployment;
  a fatal sensor initialization failure produces a continuous 1 Hz red indication.
  During normal operation, I2C, FIFO, sensor, and flash results are checked before
  data is committed. Incomplete motion events are discarded, flash failures stop
  further writes to protect existing records, and transient runtime sensor faults
  trigger a lightweight reconfiguration with low-power retries after timed wakes.

  Record format version 2:
     bytes   0-191  32 XYZ samples, three half-floats per sample
     bytes 192-220  version, sample count, page, event/configuration, RTC, system, UID
     bytes 221-252  reserved for future metadata
     bytes 253-254  CRC-16 over bytes 0-252
     byte        255 record marker

There is the main applications sketch as well as two helper sketches: one to completely erase the flash before starting a logging application (takes !70 seconds) and one to read all of the flash pages that have been written and output to the serial monitor in CSV format.

Recommend that the flash erase sketch is run before first use of the data logging application to reset flash and put flash into a known initial state. Thereafter, data may be logged until the flash is full.

Typical STANDBY current is ~4.5 uA and each wake event uses ~1 uA to log the data.
