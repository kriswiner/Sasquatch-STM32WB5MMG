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

![pinmap]([https://private-user-images.githubusercontent.com/6698410/391770744-d7677ce3-75cb-4397-b807-f825bce08898.jpg?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3Nzc5NDI2MDcsIm5iZiI6MTc3Nzk0MjMwNywicGF0aCI6Ii82Njk4NDEwLzM5MTc3MDc0NC1kNzY3N2NlMy03NWNiLTQzOTctYjgwNy1mODI1YmNlMDg4OTguanBnP1gtQW16LUFsZ29yaXRobT1BV1M0LUhNQUMtU0hBMjU2JlgtQW16LUNyZWRlbnRpYWw9QUtJQVZDT0RZTFNBNTNQUUs0WkElMkYyMDI2MDUwNSUyRnVzLWVhc3QtMSUyRnMzJTJGYXdzNF9yZXF1ZXN0JlgtQW16LURhdGU9MjAyNjA1MDVUMDA1MTQ3WiZYLUFtei1FeHBpcmVzPTMwMCZYLUFtei1TaWduYXR1cmU9YzM5YTZlY2QwYzQ5MmZhNDYyMTkxODM0MzQ1NDFjMjc4Y2I3MWRlMTY4ZWFhMGQzOGY3NzQ1YjM3ZDkwNWQ3MSZYLUFtei1TaWduZWRIZWFkZXJzPWhvc3QmcmVzcG9uc2UtY29udGVudC10eXBlPWltYWdlJTJGanBlZyJ9.1O5a0BnmwvsJ5UDzKGsweG0DjKgT4A3Xi5YOIE2RWBE)

Available for sale at [Tindie](https://www.tindie.com/products/tleracorp/sasquatch-stm32wb5mmg-development-board/).
 
 
