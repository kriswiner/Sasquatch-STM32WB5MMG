## Robust wake-on-motion detection and data logging

**[Sasquatch](https://www.tindie.com/products/tleracorp/sasquatch-stm32wb5mmg-development-board/)** is an ultra-low-power, BLE-enabled development board based on the [STM32WB5MMG](https://www.st.com/en/microcontrollers-microprocessors/stm32wb5mmg.html) module. It combines an STM32WB55 MCU, an ST [LIS2DW12](https://www.st.com/en/mems-and-sensors/lis2dw12.html) 3-axis accelerometer, and a 16-MByte Macronix MX25L12835F QSPI flash memory in a minimal platform for practical wake-on-motion detection and data logging.

Possible applications include monitoring a door for entry and exit events, detecting movement of infrequently handled equipment, recording earthquake motion, and tracking the handling of packages during shipment. In the latter case, purchasers of high-value products and their insurers may want to know whether a package has been dropped, tipped over, exposed to unusual temperatures, or otherwise subjected to an event that could cause damage. A useful event record therefore includes the inertial motion, temperature, battery voltage, and an RTC time/date stamp. Sasquatch can collect this information at sufficiently low average power for long-term battery operation.

The heart of the application is the LIS2DW12 accelerometer. It monitors for motion while the STM32WB5MMG is in a low-power state and signals the MCU when acceleration exceeds a programmable threshold. The robust sketches support the two STM32WB low-power modes of practical interest: STOP and STANDBY. STOP preserves the RTC and SRAM and provides a rapid, state-preserving interrupt wake. STANDBY uses less quiescent current, but retains only a limited portion of SRAM and resumes through a reset-like startup path. It also requires a suitable STM32WB wakeup pin; Sasquatch connects the LIS2DW12 interrupt to PC12 (Arduino D33) for this purpose.

### Post-trigger FIFO capture

The LIS2DW12 contains a 32-sample FIFO, with six bytes per XYZ sample and 192 bytes for a complete FIFO. Earlier versions of this project operated the FIFO continuously and attempted to use CONTINUOUS-to-FIFO mode to preserve the motion event. Testing showed that this did not provide the desired clean set of post-trigger samples: most of the FIFO could describe the known stationary state before the threshold crossing.

The robust STOP sketch instead controls the FIFO explicitly in software. While the device is motionless, the LIS2DW12 continues monitoring the motion threshold but its FIFO remains in BYPASS mode. A threshold interrupt wakes the MCU, which clears any pre-trigger FIFO state, selects FIFO mode with a threshold of 31, routes the FIFO-threshold interrupt, and returns to STOP. At 12.5 Hz, the watermark interrupt occurs about 2.5 seconds later. The MCU wakes again and reads all 31 raw XYZ samples in one 186-byte `Wire.transfer` transaction. The first entry is treated as the mode-transition sample and discarded, leaving 30 trusted post-trigger samples spanning approximately 2.4 seconds.

This interrupt-driven sequence deliberately favors a simple and repeatable event definition over extracting the absolute maximum number of FIFO samples. It captures the motion after the threshold crossing, avoids a continuously running FIFO, requires no timing callback to decide when the acquisition is complete, and substantially reduces average power while the monitored object is stationary. FIFO data are read in chronological order, oldest sample first.

The wake-on-motion interrupt is disabled for the duration of an event so that a single physical movement cannot create multiple records. After the FIFO has been read, the sensor waits for its sleep-on-no-motion condition. The FIFO is returned to BYPASS mode and motion detection is then rearmed for the next event.

The present robust STANDBY sketch still needs to be converted to this same post-trigger FIFO sequence and retested. Until that work is complete, the robust STOP sketch is the validated reference implementation.

### Robust operation and flash logging

Startup is treated as an operator-supervised commissioning test. The sketch verifies I2C communication, the LIS2DW12 identity and reset, self-test limits, offset calibration, motion and FIFO configuration, and QSPI flash availability before deployment. Fatal initialization failures produce a persistent red LED indication. During operation, I2C, FIFO, sensor, and flash results are checked before an event is committed. Incomplete motion events are discarded, flash failures stop further writes to protect existing data, and transient sensor faults trigger lightweight reconfiguration attempts on timed wakes.

Each valid motion event occupies one 256-byte QSPI flash page. The 30 XYZ samples are converted to milligravity units and stored as three 16-bit half-floats per sample. The record also includes the number of valid samples, format version, physical flash page, initiating wake-source bits, accelerometer range and data rate, RTC time and date, battery voltage, MCU temperature, and the 96-bit MCU unique identifier. Unused bytes remain reserved for future metadata, including barometric motion information. A CRC-16 protects bytes 0 through 252 and a final completion marker distinguishes a fully written record from an erased, partial, or corrupted page.

Record format version 2:

    bytes   0-179  30 trusted XYZ samples, three half-floats per sample
    bytes 180-191  unused sample space, left erased
    byte      192  valid sample count
    bytes 193-220  version, page, event/configuration, RTC, system data, and UID
    bytes 221-252  reserved for future metadata
    bytes 253-254  CRC-16 over bytes 0-252
    byte      255  completed-record marker

The next flash page number is retained in backup SRAM so an ordinary STOP wake does not require an energy-consuming scan. If that retained state is missing or invalid after a reset or battery interruption, a bounded binary search locates the first erased page and resumes appending without erasing earlier records. The retained page and fault state use complementary values and a validity marker to detect interrupted updates. 

The repository also includes helper sketches to erase the complete flash before commissioning and to read written pages to the serial monitor in CSV form. Erasing the flash before first use puts it into a known state; thereafter, records can be appended until the available event-page region is full.

Typical results:
<img width="851" height="607" alt="Capture" src="https://github.com/user-attachments/assets/984be349-5ff9-4acc-9806-c27b7a989ff1" />

_Data from a single motion event captured on flash and transferred to a spreadsheet for plotting._


### Configuration and measured power

The current STOP reference uses a +/-2 g full-scale range, 12.5 Hz output data rate, low-power mode 1, and a motion threshold of approximately 62.5 mg. This gives roughly 1 mg resolution and is well suited to human-scale handling motion. More demanding impacts can be captured by increasing the full-scale range, output data rate, or performance mode at the cost of resolution, capture duration, or power.

With the corrected duty-cycled FIFO sequence, measured STOP current was approximately 6.22 to 6.28 uA. In a repeatable ten-minute test containing one RTC alarm per minute and five motion events, average current was approximately 8.12 uA, or about 0.38 uA of ten-minute average current per handled event above the measured STOP baseline. These figures are measurements of the present breadboard test system rather than guaranteed product specifications, but they demonstrate that a practical motion-event logger can operate at an average current well below 10 uA at a modest event rate.

At 10 uA, one year of idealized operation consumes about 88 mAh. A real deployment should include allowance for battery self-discharge, temperature, pulse-current capability, regulator losses, event rate, and end-of-life voltage rather than selecting a cell from nominal capacity alone.

The next development step is to bring the robust STANDBY implementation into conformance with the validated STOP FIFO sequence. A subsequent integrated version will add an LPS22DF pressure sensor so each motion event can include a slowly varying pressure baseline and a short post-trigger altitude history, allowing true elevation changes to be distinguished from transient pressure artifacts.
