# Sasquatch-STM32WB5MMG
**Arduino-compatible, USB programmable, BLE-enabled, STM32WB55-based development platform**

Some test sketches to demonstrate how to make use of the Sasquatch development board.

**Sasquatch is:**

--64 MHz Cortex M4 MCU with 1 MB flash and 256 KB SRAM for application firmware, 30 GPIOs exposed to the user;

--Robust, power-and memory-efficient Arduino core written from scratch properly handling all errata with no gaps;

--Programmable using USB via the Arduino IDE or using ST-LINK and the SWD programming header;

--Embedded RF antenna for BLE (Arduino core supports both peripheral and central roles);

--On-board 16 MB QSPI NOR flash, 3-axis accelerometer, rgb led, battery voltage monitor;

--Small (20.5 mm x 50.8 mm), ultra-low-power (~2.1 uA sleep) development platform for breadboarding and prototyping.

Initial testing with the above sketch shows that the STOP mode current is ~4.85 uA when the LIS2DW12 accel is always on for wake-on-motion/sleep-on-no-motion functionality.

![pinmap]( )

Available for sale at [Tindie](https://www.tindie.com/products/tleracorp/sasquatch-stm32wb5mmg-development-board/).
 
 
