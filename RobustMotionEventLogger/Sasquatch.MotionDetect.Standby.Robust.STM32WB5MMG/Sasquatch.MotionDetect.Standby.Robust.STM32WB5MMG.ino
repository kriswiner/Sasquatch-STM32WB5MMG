/*
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
*/

#include "Arduino.h"
#include "STM32WB.h"
#include "I2Cdev.h"
#include "LIS2DW12.h"
#include "SFLASH.h"
#include "RTC.h"
//#include "BLE.h"

//BLEUart SerialBLE(BLE_UART_PROTOCOL_NORDIC);  // instantiate Nordic UART service

#define I2C_BUS_1    Wire1              // Define the internal I2C bus (Wire1 instance)  

I2Cdev             i2c_1(&I2C_BUS_1);   // Instantiate the I2Cdev object and point to the desired I2C bus //

// Internal STM32WB variables
float VBAT, VDDA, Temperature;
uint16_t iVBAT, iVDDA, iTemperature;
uint32_t UID[3] = {0, 0, 0};
volatile bool USBConnected = false; 
bool SerialDebug = false;

// QSPI flash variables
uint8_t  mid;
uint16_t did;

// SRAM2 makes ordinary STANDBY wakes inexpensive: no QSPI search is needed.
// Magic and inverse fields detect lost or partially updated retained state;
// only then does recoverLogState() search the persistent flash records.
struct RetainedLogState {
  uint32_t magic;
  uint16_t page;
  uint16_t pageInverse;
  uint8_t  flashFault;
  uint8_t  flashFaultInverse;
};

volatile RetainedLogState __SECTION_BBRAM_NOINIT logState; // Preserve ordered retained-state updates across resets.
const uint32_t LOG_STATE_MAGIC = 0x53415351;
const uint16_t MAX_LOG_PAGES = 0xFFFF;
const uint32_t FLASH_TIMEOUT_MS = 1000;
const uint8_t  RECORD_MARKER = 0x73;
const uint8_t  RECORD_VERSION = 2;
const uint16_t SAMPLE_COUNT_BYTE = 192;
const uint16_t VERSION_BYTE = 193;
const uint16_t PAGE_MSB_BYTE = 194;
const uint16_t PAGE_LSB_BYTE = 195;
const uint16_t WAKE_SOURCE_BYTE = 196;
const uint16_t FULL_SCALE_BYTE = 197;
const uint16_t DATA_RATE_BYTE = 198;
const uint16_t RTC_SECONDS_BYTE = 199;
const uint16_t RTC_MINUTES_BYTE = 200;
const uint16_t RTC_HOURS_BYTE = 201;
const uint16_t RTC_DAY_BYTE = 202;
const uint16_t RTC_MONTH_BYTE = 203;
const uint16_t RTC_YEAR_BYTE = 204;
const uint16_t VBAT_MSB_BYTE = 205;
const uint16_t VBAT_LSB_BYTE = 206;
const uint16_t TEMPERATURE_MSB_BYTE = 207;
const uint16_t TEMPERATURE_LSB_BYTE = 208;
const uint16_t UID_START_BYTE = 209;
const uint8_t  UID_LENGTH = 12;
const uint16_t CRC_MSB_BYTE = 253;
const uint16_t CRC_LSB_BYTE = 254;
const uint16_t MARKER_BYTE = 255;
uint8_t  flashPage[256];                      // array to hold the data for flash page write

bool logStateValid()
{
  return (logState.magic == LOG_STATE_MAGIC) &&
         ((uint16_t)(logState.page ^ logState.pageInverse) == 0xFFFF) &&
         ((uint8_t)(logState.flashFault ^ logState.flashFaultInverse) == 0xFF) &&
         (logState.flashFault <= 1);
}

void setLogState(uint16_t page, bool flashFault)
{
  logState.magic = 0;                           // Invalidate state while its fields are changing.
  logState.page = page;                         // Store the next page to be written.
  logState.pageInverse = ~page;                 // Complement detects an interrupted update.
  logState.flashFault = flashFault;             // A write fault stops unsafe page reuse.
  logState.flashFaultInverse = ~logState.flashFault;
  logState.magic = LOG_STATE_MAGIC;             // Write magic last so incomplete state is rejected.
}

//RTC time variables
uint8_t Seconds, Minutes, Hours, Day, Month, Year;
volatile bool alarmFlag = false; // for RTC alarm interrupt

// define rgb led pins and colors
#define greenLed 22 // green led active LOW
#define redLed   23 // red led active LOW
#define blueLed  24 // blue led active LOW

// simple allowed colors
#define red     0
#define green   1
#define blue    2
#define yellow  3
#define magenta 4
#define cyan    5
#define white   6

// types of reset possible
#define RESET_HARDWARE         0  // hardware reset
#define RESET_SOFTWARE         1
#define RESET_WAKEUP           2  // reset on wake from STANDBY
#define RESET_WATCHDOG         3
#define RESET_FAULT            4
#define RESET_ASSERT           5
#define RESET_PANIC            6

// wake up reasons
#define WAKEUP_PIN_1           0x00000001
#define WAKEUP_PIN_2           0x00000002
#define WAKEUP_PIN_3           0x00000004   // LIS2DW12 accel interrupt for wake from STANDBY on motion
#define WAKEUP_PIN_4           0x00000008
#define WAKEUP_PIN_5           0x00000010
#define WAKEUP_PIN_RESET       0x00000100   
#define WAKEUP_WATCHDOG        0x00000200
#define WAKEUP_TIMEOUT         0x00000400   // wake from STANDBY on timeout


//LIS2DW12 definitions

// wake-on-motion and sleep-on-no-motion interrupts on same LIS2DW12 interrupt (INT1), 
// this is PC12 which is a tamper and wakeup pin
#define LIS2DW12_intPin   33    

// Specify LIS2DW12 accel sensor parameters  
LPMODE   lpMode = LIS2DW12_LP_MODE_1;      // choices are low power modes 1, 2, 3, or 4, LP_MODE_1 data is 12-bit, LP_MODE_2,3,4 data is 14-bit
MODE     Mode   = LIS2DW12_MODE_LOW_POWER; // choices are low power, high performance, and one shot modes
ODR      odr    = LIS2DW12_ODR_12_5Hz;     // 1.6 Hz in lpMode, max is 200 Hz in LpMode
FS       fs     = LIS2DW12_FS_2G;          // choices are 2, 4, 8, or 16 g
BW_FILT  bw     = LIS2DW12_BW_FILT_ODR4;   // choices are ODR divided by 2, 4, 10, or 20
FIFOMODE fifoMode = CONT_TO_FIFO;          // capture 32 samples of data before wakeup event, about 2.5 secs at 12.5 Hz, 20 sec at 1.6 Hz
bool lowNoise = false;                     // low noise or lowest power
// when in stationary mode, sample rate odr is constant in wake and sleep states,
// when not in stationary mode, sample rate is 12.5 Hz in sleep state and whatever odr is set in wake state
bool stMode = false;                       // lowest power usage when in stationary mode and odr is set to 1.6 Hz
                      
float aRes = 0.000244f * (1 << fs);        // scale resolutions per LSB for the sensor at 14-bit data 
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t tempCount;      // 8-bit signed temperature output
uint8_t rawTempCount;   // raw temperature output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float ax, ay, az;       // variables to hold latest sensor data values 
uint16_t iax, iay, iaz; // variables to hold latest sensor data values as half-floats 
float __SECTION_BBRAM_DATA offset[3] = {-0.0361f, -0.0774f, 0.0145f}; // preserve LIS2DW12 offsets in SRAM2 on wakeup from STANDBY
float stress[3];        // holds results of the self test
uint8_t LIS2DW12_status = 0, wakeSource = 0, FIFOstatus = 0, numFIFOSamples = 0;
bool __SECTION_BBRAM_DATA sensorFault = false; // Retry LIS2DW12 recovery after a failed STANDBY wake.

LIS2DW12 LIS2DW12(&i2c_1); // instantiate LIS2DW12 class


// Start of setup
void setup() {
  if(SerialDebug) {Serial.begin(115200);}
//  while (!Serial) { }

  // Configure MCU GPIOs
  pinMode(greenLed, OUTPUT); digitalWrite(greenLed, HIGH);  // set rgb leds as output, active LOW
  pinMode(  redLed, OUTPUT); digitalWrite(redLed,   HIGH);
  pinMode( blueLed, OUTPUT); digitalWrite(blueLed,  HIGH);
  pinMode(LIS2DW12_intPin, INPUT);  // define LIS2DW12 wake/sleep interrupt pin as WB5MMG input

  STM32WB.getUID(UID);
  if(SerialDebug) {Serial.print("STM32WB5 MCU UID = 0x"); Serial.print(UID[0], HEX); Serial.print(UID[1], HEX); Serial.println(UID[2], HEX);} 

  // instantiate internal wire port
  I2C_BUS_1.begin(); // set master mode 
  I2C_BUS_1.setClock(400000); // I2C frequency at 400 kHz  

/*
  BLE.begin();
  BLE.setLocalName("Sasquatch Event Tracker");
  BLE.setAdvertisedServiceUuid(SerialBLE.uuid());
  BLE.addService(SerialBLE);    
  BLE.advertise();
  delay(1000);
*/

  uint32_t resetCause = STM32WB.resetCause();
  bool sensorOK = true;                                  // Collect runtime LIS2DW12 operation results.

  // Normal STANDBY wakes use SRAM2 without reading flash. Other resets verify or recover the page.
  if(!logStateValid()) {
    if(!recoverLogState()) setLogState(0, true);
  }
  else if((resetCause != RESET_WAKEUP) && !logState.flashFault && (logState.page < MAX_LOG_PAGES)) {
    if(!pageIsErased(logState.page) && !recoverLogState()) setLogState(logState.page, true);
  }
  
  // Check if wakeup from STANDBY via RESET_WAKEUP
  if(resetCause == RESET_WAKEUP) // then no need to reinitialize the sensors, etc
  {
  uint32_t wakeReason = STM32WB.wakeupReason(); 
  
  if(wakeReason == WAKEUP_PIN_3) {                     // this is the LIS2DW12 interrupt

   if(!LIS2DW12.getStatus(&LIS2DW12_status)) {LIS2DW12_status = 0; sensorOK = false;} // Read LIS2DW12 status.

   if(!LIS2DW12.getWakeSource(&wakeSource)) {wakeSource = 0; sensorOK = false;} // Read the acceleration wake source.
   
   if(SerialDebug) {
    if(wakeSource & 0x20) Serial.println("Free fall detected!");
    if(wakeSource & 0x10) Serial.println("Sleep event detected!");
    if(wakeSource & 0x08) Serial.println("Wake-up event detected!");
    if(wakeSource & 0x04) Serial.println("Wake-up on x-axis detected!");
    if(wakeSource & 0x02) Serial.println("Wake-up on y-axis detected!");
    if(wakeSource & 0x01) Serial.println("Wake-up on z-axis detected!");
   }
   
   if(LIS2DW12_status & 0x40) {       // is this a wakeup event?
   if(!LIS2DW12.deactivateWakeOnMotionInterrupt()) sensorOK = false; // Disable repeated threshold interrupts.
   if(SerialDebug) Serial.println("** LIS2DW12 is awake! **");

    // collect and store the accel data around the wakeup event
   if(!LIS2DW12.FIFOsamples(&FIFOstatus)) {FIFOstatus = 0; sensorOK = false;} // Read FIFO count and flags.
   if(FIFOstatus & 0x80) {                            // if the FIFO threshold is reached
      numFIFOSamples =  FIFOstatus & 0x3F;             // should be 32 sample

      memset(flashPage, 0xFF, sizeof(flashPage));      // Erased bytes remain available for future fields.
      flashPage[SAMPLE_COUNT_BYTE] = numFIFOSamples;  // Save the actual number of FIFO samples.
      flashPage[VERSION_BYTE] = RECORD_VERSION;       // Let future readers recognize this layout.
      bool accelReadOK = true;                        // Require every FIFO sample before committing the event.

      for(uint8_t i = 0; i < numFIFOSamples; i++) {    // Read the FIFO data.
         if(!LIS2DW12.readAccelData(accelCount)) {     // Stop if any three-axis sample cannot be read.
           accelReadOK = false;
           sensorOK = false;                          // Request sensor recovery before returning to STANDBY.
           break;
         }

         ax = (float)accelCount[0]*aRes - offset[0];   // get actual g value, this depends on scale being set
         ay = (float)accelCount[1]*aRes - offset[1];   // could just store the two raw bytes for each axis
         az = (float)accelCount[2]*aRes - offset[2];   // but here we will scale and store half floats

         //  convert to milligs to make it easier to plot changes due to motion
         iax = LIS2DW12.FloattoHalf(1000.0f*ax);       // convert float data to half-float data for efficient storage
         iay = LIS2DW12.FloattoHalf(1000.0f*ay);
         iaz = LIS2DW12.FloattoHalf(1000.0f*az);

         // store data in QSPI flash memory
         // First page number is 0
         // Page 0xFFFF is the full-flash sentinel for this 128-Mbit device.
         if(!logState.flashFault && (logState.page < MAX_LOG_PAGES)) {
         flashPage[6*i + 0] = (iax & 0xFF00) >> 8;  // write six half-float bytes for each of 0 - 31 FIFO samples
         flashPage[6*i + 1] = (iax & 0x00FF);      
         flashPage[6*i + 2] = (iay & 0xFF00) >> 8;
         flashPage[6*i + 3] = (iay & 0x00FF);      
         flashPage[6*i + 4] = (iaz & 0xFF00) >> 8;
         flashPage[6*i + 5] = (iaz & 0x00FF); }     
         
         if(SerialDebug) {  // print out FIFO data on the serial monitor
          Serial.print("ax = ");  Serial.print((int)(1000.0f*ax));         
          Serial.print(" ay = "); Serial.print((int)(1000.0f*ay)); 
          Serial.print(" az = "); Serial.print((int)(1000.0f*az)); Serial.println(" mg"); Serial.println(" ");
         }
      }

      if(!accelReadOK && SerialDebug) Serial.println("LIS2DW12 FIFO read failed; event discarded.");

 // Check RTC time, RTC kept alive in STANDBT mode
  RTC.getDateTime(Day, Month, Year, Hours, Minutes, Seconds);
  if(SerialDebug) {
    Serial.println(" ");  Serial.print("RTC: ");
    if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
    Serial.print(":"); 
    if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
    Serial.print(":"); 
    if(Seconds < 10) {Serial.print("0"); Serial.println(Seconds);} else Serial.println(Seconds);  
    Serial.println(" ");
    Serial.print(Month); Serial.print("/"); Serial.print(Day); Serial.print("/"); Serial.println(Year);
    Serial.println(" ");
  }
  
  // Check analog and battery voltages
  VDDA = STM32WB.readVREF();
  Temperature = STM32WB.readTemperature();
  VBAT = STM32WB.readBattery();

  if(SerialDebug)  {
    Serial.print("VDDA = "); Serial.println(VDDA, 2); 
    Serial.print("STM32WB MCU Temperature = "); Serial.println(Temperature, 2);
    Serial.print("VBAT = "); Serial.println(VBAT, 2); 
  }
      iTemperature = LIS2DW12.FloattoHalf(Temperature);  // convert float system data to half-float data for efficient storage
      iVBAT = LIS2DW12.FloattoHalf(VBAT);
      
    // Add identifying, event, configuration, RTC, and system metadata to the record.
      if(sensorOK && accelReadOK && !logState.flashFault && (logState.page < MAX_LOG_PAGES)) { // Commit only a complete event.
      flashPage[PAGE_MSB_BYTE] = (logState.page & 0xFF00) >> 8;  // Embed the physical page for continuity checks.
      flashPage[PAGE_LSB_BYTE] =  logState.page & 0x00FF;
      flashPage[WAKE_SOURCE_BYTE] = wakeSource;                   // Preserve the LIS2DW12 event-source bits.
      flashPage[FULL_SCALE_BYTE] = (uint8_t)fs;                   // Save the LIS2DW12 full-scale enum code.
      flashPage[DATA_RATE_BYTE] = (uint8_t)odr;                   // Save the LIS2DW12 output-data-rate enum code.
      flashPage[RTC_SECONDS_BYTE] = Seconds;                      // Store RTC time and date as individual bytes.
      flashPage[RTC_MINUTES_BYTE] = Minutes;
      flashPage[RTC_HOURS_BYTE] = Hours;
      flashPage[RTC_DAY_BYTE] = Day;
      flashPage[RTC_MONTH_BYTE] = Month;
      flashPage[RTC_YEAR_BYTE] = Year;
      flashPage[VBAT_MSB_BYTE] = (iVBAT & 0xFF00) >> 8;           // Store battery voltage as a half-float.
      flashPage[VBAT_LSB_BYTE] =  iVBAT & 0x00FF;
      flashPage[TEMPERATURE_MSB_BYTE] = (iTemperature & 0xFF00) >> 8;  // Store MCU temperature as a half-float.
      flashPage[TEMPERATURE_LSB_BYTE] =  iTemperature & 0x00FF;

      for(uint8_t word = 0; word < (UID_LENGTH / 4); word++) {     // Store the 96-bit MCU UID most-significant byte first.
        for(uint8_t byte = 0; byte < 4; byte++) {
          flashPage[UID_START_BYTE + (4 * word) + byte] = (UID[word] >> (24 - (8 * byte))) & 0xFF;
        }
      }

      uint16_t crc = calculateCRC16(flashPage, CRC_MSB_BYTE);     // Protect all data and metadata before the CRC.
      flashPage[CRC_MSB_BYTE] = (crc & 0xFF00) >> 8;
      flashPage[CRC_LSB_BYTE] =  crc & 0x00FF;
      flashPage[MARKER_BYTE] = RECORD_MARKER;                     // Mark the page as a complete record.
 
   bool flashOK = SFLASH.begin();

   if(flashOK) {
     flashOK = SFLASH.program(logState.page * 256, flashPage, sizeof(flashPage));

     if(flashOK) {
       flashOK = waitForFlash();
     }

     SFLASH.end();
   }

   if(flashOK) {
     if(SerialDebug) {
       Serial.print("Wrote flash page: "); Serial.println(logState.page);

     }

      ledBlink(green, 1);
      setLogState(logState.page + 1, false);
    }
    
    else {
      if(SerialDebug) Serial.println("QSPI flash write failed!");
      ledBlink(red, 10);
      setLogState(logState.page, true);
    }
       }  
       else if(logState.page == MAX_LOG_PAGES) 
      {
       if(SerialDebug) {
        Serial.println("Reached last page of SPI flash!"); Serial.println("Data logging stopped!");
       }
      }
    }

    // Send some data via the BLE UART service
/*    SerialBLE.print("Temp = ");
    SerialBLE.print(Temperature);
    SerialBLE.println(" *C");

    SerialBLE.print("VBAT = ");
    SerialBLE.print(VBAT);
    SerialBLE.println(" V");

    SerialBLE.print("No Samples = ");
    SerialBLE.println(numFIFOSamples);

    SerialBLE.println();
    */

    if(!LIS2DW12.configureFIFO(BYPASS, 0x1F)) sensorOK = false; // Clear the FIFO and detect a bus failure.
    ledBlink(blue, 1);                                 // blink blue led when LIS2DW12 wakes from sleep
   }  // end of LIS2DW12 wake detect

   if(LIS2DW12_status & 0x20) { 
   if(!LIS2DW12.activateWakeOnMotionInterrupt()) sensorOK = false; // Arm the next motion threshold crossing.
   if(SerialDebug) Serial.println("** LIS2DW12 is asleep! **");     
   if(!LIS2DW12.configureFIFO(CONT_TO_FIFO, 0x1F)) sensorOK = false; // Restart continuous-to-FIFO capture.
   ledBlink(green, 1);                                // blink green led when LIS2DW12 goes back to sleep
   } // end of LIS2DW12 sleep detect // 
   
   } // end of wakeup pin 3 event handling
  

  if(wakeReason == WAKEUP_TIMEOUT) {
    if(SerialDebug)Serial.println("Woke up from timeout!"); 
    ledBlink(red, 1);

    // watchdog timer, log time or take some other action here
  
    // Send some data via the BLE UART service
/*    SerialBLE.print("Temp = ");
    SerialBLE.print(Temperature);
    SerialBLE.println(" *C");

    SerialBLE.print("VBAT = ");
    SerialBLE.print(VBAT);
    SerialBLE.println(" V");
    */
  } // end wakeup timeout handling
  
  }
  else // if waking from hardware or watchdog reset, need to configure sensors, etc ****************************************
  {
  // should only enter this section once at the beginning
  digitalWrite(greenLed, LOW);  // turn on green led while initializing

  // test and configure sesors and flash on hardware reset
  // scan internal I2C port for devices...
  if(SerialDebug) {Serial.println("Scan internal I2C1 port for devices: ");}
  i2c_1.I2Cscan(); 

  // Read the LIS2DW12 identity and perform the complete commissioning sequence.
  byte LIS2DW12_ChipID = 0;                                      // Default to an invalid identity.
  sensorOK = LIS2DW12.getChipID(&LIS2DW12_ChipID);                // Verify I2C communication.
  if(LIS2DW12_ChipID != 0x44) sensorOK = false;                   // Verify the expected device.

  if(SerialDebug) {
    Serial.print("LIS2DW12 "); Serial.print("I AM "); Serial.print(LIS2DW12_ChipID, HEX); Serial.print(" I should be "); Serial.println(0x44, HEX);
    Serial.println(" ");
  }

  if(sensorOK && SerialDebug) {Serial.println("LIS2DW12 is online..."); Serial.println(" ");}

  if(sensorOK) sensorOK = LIS2DW12.reset();                       // Reset sensor registers.
  if(sensorOK) sensorOK = LIS2DW12.selfTest(stress);              // Run the electrical self-test.

  if(sensorOK && SerialDebug) {
     Serial.print("x-axis self test = "); Serial.print(stress[0], 1); Serial.println("mg, should be between 70 and 1500 mg");
     Serial.print("y-axis self test = "); Serial.print(stress[1], 1); Serial.println("mg, should be between 70 and 1500 mg");
     Serial.print("z-axis self test = "); Serial.print(stress[2], 1); Serial.println("mg, should be between 70 and 1500 mg");
  }

  if(sensorOK) {
    sensorOK = (stress[0] >= 70.0f && stress[0] <= 1500.0f) &&
               (stress[1] >= 70.0f && stress[1] <= 1500.0f) &&
               (stress[2] >= 70.0f && stress[2] <= 1500.0f);      // Enforce ST self-test limits.
  }

  if(sensorOK) sensorOK = LIS2DW12.reset();                       // Restore default registers.

  if(sensorOK && 1) {                                             // Change 1 to 0 to use stored offsets.
    if(SerialDebug) Serial.println("hold flat and motionless for bias calibration");
    delay(5000);                                                   // Allow time to place the board motionless.
    sensorOK = LIS2DW12.Compensation(fs, odr, Mode, lpMode, bw, lowNoise, offset); // Calculate acceleration offsets.

  if(sensorOK && SerialDebug) { // print out LIS2DW12 accel offset bias
     Serial.print("x-axis offset = "); Serial.print(offset[0]*1000.0f, 1); Serial.println(" mg");
     Serial.print("y-axis offset = "); Serial.print(offset[1]*1000.0f, 1); Serial.println(" mg");
     Serial.print("z-axis offset = "); Serial.print(offset[2]*1000.0f, 1); Serial.println(" mg"); Serial.println(" ");
    }
  }

  if(sensorOK) sensorOK = LIS2DW12.init(fs, odr, Mode, lpMode, bw, lowNoise, stMode); // Configure motion detection.
  if(sensorOK) sensorOK = LIS2DW12.configureFIFO(fifoMode, 0x1F);    // Start FIFO history capture.

  if(sensorOK) {
    sensorFault = false;                                            // Clear any retained runtime fault.
    delay(100);                                                      // Allow the configured sensor to settle.
  }

  if(!sensorOK)
  {
    if(SerialDebug) Serial.println("LIS2DW12 initialization failed; device is not operational.");
    digitalWrite(greenLed, HIGH);                                   // Turn off the initialization indicator.

    while(1) {
      ledBlink(red, 100);                                           // Flash red briefly once per second.
      delay(900);
    }
  }

  // Test QSPI flash memory
  if(SerialDebug) Serial.println("QSPI Flash Check");
  bool flashStarted = SFLASH.begin();                           // Track whether the flash interface was powered.
  bool flashReady = flashStarted && SFLASH.identify(mid, did);  // Verify the fitted flash device.

  if(flashReady && SerialDebug) {
  Serial.print("MID = ");       Serial.println(mid, HEX); 
  Serial.print("DID = ");       Serial.println(did, HEX); 
  Serial.print("CAPACITY = ");  Serial.println(SFLASH.capacity());
  Serial.print("BLOCKSIZE = "); Serial.println(SFLASH.blockSize());
  Serial.print("PAGESIZE = ");  Serial.println(SFLASH.pageSize());
  Serial.print("LENGTH = ");    Serial.println(SFLASH.length()); Serial.println(" ");
  }

  if(flashStarted) SFLASH.end();                               // Always release flash power and pins after begin().
  if(flashReady && logState.flashFault) flashReady = recoverLogState(); // Revalidate a retained flash fault before deployment.

  if(!flashReady)
  {
    setLogState(logState.page, true);                          // Prevent all attempted flash writes.
    if(SerialDebug) Serial.println("QSPI flash initialization failed; device is not operational.");
    digitalWrite(greenLed, HIGH);                              // Turn off the initialization indicator.

    while(1) {
      ledBlink(red, 100);                                      // Two red flashes identify a flash fault.
      delay(100);
      ledBlink(red, 100);
      delay(700);
    }
  }

  digitalWrite(greenLed, HIGH);  // turn off green led after initialization

  }  // end of reset cause check and initial configuration ****************************************
  
  
    if(!LIS2DW12.getStatus(&LIS2DW12_status)) sensorOK = false; // Clear pending status before STANDBY.

    if(sensorFault || !sensorOK) {
      if(SerialDebug) Serial.println("LIS2DW12 runtime failure; attempting recovery.");
      sensorFault = !recoverMotionSensor();             // Retain failure so the next timeout retries recovery.
    }
//  LIS2DW12.powerDown();

  STM32WB.standby(LIS2DW12_intPin, RISING, 60); // stay in STANDBY mode until timeout (1 min) occurs or wakeup pin goes HIGH  

} /* End of Setup */


void loop() {

// should never get here...

} /* end of main loop */


// LED indication helpers

  void ledBlink(uint8_t color, uint32_t duration)
  {

  uint8_t _out1;
  uint8_t _out2;
  uint8_t _out3;
  
   if(color == green) {
   _out1 = 0x00;
   _out2 = 0x01;
   _out3 = 0x01;
   }

   if(color == blue) {
   _out1 = 0x01;
   _out2 = 0x00;
   _out3 = 0x01;
  }

   if(color == red) {
   _out1 = 0x01;
   _out2 = 0x01;
   _out3 = 0x00;
   }

   if(color == cyan) {
   _out1 = 0x00;
   _out2 = 0x00;
   _out3 = 0x01;
   }

   if(color == magenta) {
   _out1 = 0x01;
   _out2 = 0x00;
   _out3 = 0x00;
   }

   if(color == yellow) {
   _out1 = 0x00;
   _out2 = 0x01;
   _out3 = 0x00;
   }

   if(color == white) {
   _out1 = 0x00;
   _out2 = 0x00;
   _out3 = 0x00;
   }

   // set rgb led current
  digitalWrite(redLed, _out3); 
  digitalWrite(greenLed, _out1);  
  digitalWrite(blueLed, _out2);  
  delay(duration);
  digitalWrite(redLed, HIGH); 
  digitalWrite(greenLed, HIGH);  
  digitalWrite(blueLed, HIGH);  
  }


// LIS2DW12 recovery helpers

bool recoverMotionSensor()
{
  if(!LIS2DW12.reset()) return false;                                      // Reset sensor registers.
  if(!LIS2DW12.init(fs, odr, Mode, lpMode, bw, lowNoise, stMode)) return false; // Restore motion detection.
  if(!LIS2DW12.configureFIFO(fifoMode, 0x1F)) return false;                 // Restart FIFO capture.
  if(!LIS2DW12.getStatus(&LIS2DW12_status)) return false;                  // Clear and verify status.
  return true;
}


// CRC and record-validation helpers

uint16_t calculateCRC16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFF;

  while(length--) {
    crc ^= (uint16_t)(*data++) << 8;

    for(uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
  }

  return crc;
}


// QSPI flash operation and recovery helpers

bool waitForFlash()
{
  uint32_t start = millis();

  while(SFLASH.busy()) {
    if((millis() - start) >= FLASH_TIMEOUT_MS) return false;
  }

  return (SFLASH.status() == SFLASH_STATUS_SUCCESS);
}


bool pageBufferIsErased()
{
  for(uint16_t i = 0; i < sizeof(flashPage); i++) {
    if(flashPage[i] != 0xFF) return false;
  }

  return true;
}


bool recordBufferIsValid(uint16_t expectedPage)
{
  uint8_t version = flashPage[VERSION_BYTE];
  uint16_t storedPage = ((uint16_t)flashPage[PAGE_MSB_BYTE] << 8) | flashPage[PAGE_LSB_BYTE];
  uint16_t storedCRC = ((uint16_t)flashPage[CRC_MSB_BYTE] << 8) | flashPage[CRC_LSB_BYTE];

  return ((version == 1) || (version == RECORD_VERSION)) &&  // Accept existing v1 records while appending v2.
         ((version == 1) || (storedPage == expectedPage)) &&
         (flashPage[MARKER_BYTE] == RECORD_MARKER) &&
         (storedCRC == calculateCRC16(flashPage, CRC_MSB_BYTE));
}


bool pageIsErased(uint16_t page)
{
  bool success = SFLASH.begin();

  if(success) {
    uint32_t pageSize = SFLASH.pageSize();
    uint32_t pageCount = pageSize ? (SFLASH.length() / pageSize) : 0;

    success = (pageSize == sizeof(flashPage)) &&
              (page < pageCount) &&
              SFLASH.read((uint32_t)page * pageSize, flashPage, sizeof(flashPage));

    if(success) success = pageBufferIsErased();

    SFLASH.end();
  }

  return success;
}


bool recoverLogState()
{
  bool success = SFLASH.begin();
  uint32_t page = 0;

  if(success) {
    uint32_t pageSize = SFLASH.pageSize();
    uint32_t pageCount = pageSize ? (SFLASH.length() / pageSize) : 0;

    if(pageCount > MAX_LOG_PAGES) pageCount = MAX_LOG_PAGES;
    success = (pageSize == sizeof(flashPage)) && pageCount;

    uint32_t first = 0;
    uint32_t last = pageCount;

    // Written pages are contiguous, so at most 16 marker reads locate the boundary.
    while(success && (first < last)) {
      uint32_t middle = first + ((last - first) / 2);
      uint8_t marker;

      success = SFLASH.read((middle * pageSize) + MARKER_BYTE, &marker, 1);

      if(success) {
        if(marker == RECORD_MARKER) first = middle + 1;
        else                        last = middle;
      }
    }

    page = first;

    // Validate the final written page and ensure the next page is truly unused.
    if(success && page) {
      success = SFLASH.read((page - 1) * pageSize, flashPage, sizeof(flashPage));
      if(success) success = recordBufferIsValid((uint16_t)(page - 1));
    }

    if(success && (page < pageCount)) {
      success = SFLASH.read(page * pageSize, flashPage, sizeof(flashPage));
      if(success) success = pageBufferIsErased();
    }

    SFLASH.end();
  }

  if(success) setLogState((uint16_t)page, false);

  return success;
}
