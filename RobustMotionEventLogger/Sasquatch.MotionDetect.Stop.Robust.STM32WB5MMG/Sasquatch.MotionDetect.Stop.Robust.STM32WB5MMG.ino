#include "Arduino.h"
#include "STM32WB.h"
#include "I2Cdev.h"
#include "LIS2DW12.h"
#include "SFLASH.h"
#include "RTC.h"
#include <string.h>

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

struct RetainedLogState {
  uint32_t magic;
  uint16_t page;
  uint16_t pageInverse;
  uint8_t  flashFault;
  uint8_t  flashFaultInverse;
};

volatile RetainedLogState __SECTION_BBRAM_NOINIT logState; // Preserve the next flash page across resets.
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
uint8_t  flashPage[256];                      // Hold one complete flash record.

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
//FIFOMODE fifoMode = BYPASS;              // capture 32 samples of data before wakeup event, about 2.5 secs at 12.5 Hz, 20 sec at 1.6 Hz
bool lowNoise = false;                     // low noise or lowest power
// when in stationary more, sample rate odr is constant in wake and sleep states,
// when not in stationary mode, sample rate is 12.5 Hz in sleep state and whatever odr is set in wake state
bool stMode = false;                       // lowest power usage when in stationary mode and odr is set to 1.6 Hz
                      
float aRes = 0.000244f * (1 << fs);        // scale resolutions per LSB for the sensor at 14-bit data 
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t tempCount;      // 8-bit signed temperature output
uint8_t rawTempCount;   // raw temperature output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float ax, ay, az;       // variables to hold latest sensor data values 
uint16_t iax, iay, iaz; // variables to hold latest sensor data values as half-floats 
float offset[3] = {-0.0361f, -0.0774f, 0.0145f};        // holds accel bias offsets in mg
float stress[3];        // holds results of the self test
uint8_t LIS2DW12_status = 0, wakeSource = 0, eventWakeSource = 0;
uint8_t FIFOstatus = 0, numFIFOSamples = 0;

// Logic flags to keep track of device states
volatile bool LIS2DW12_int_flag = false;
volatile bool InMotion = false;
bool LIS2DW12_fifo_capture = false;

LIS2DW12 LIS2DW12(&i2c_1); // instantiate LIS2DW12 class

uint16_t FloattoHalf(float f);      // Convert a 32-bit float to compact 16-bit storage
float HalftoFloat(uint16_t h);      // Convert compact 16-bit storage back to a 32-bit float
uint16_t calculateCRC16(const uint8_t *data, uint16_t length); // Calculate record CRC.
bool waitForFlash();                // Wait for flash completion or timeout.
bool pageBufferIsErased();          // Test whether flashPage contains erased bytes.
bool recordBufferIsValid(uint16_t expectedPage); // Validate a buffered record.
bool pageIsErased(uint16_t page);   // Test one physical flash page.
bool recoverLogState();             // Find the first erased page.
void captureSystemSnapshot();       // Capture RTC and MCU operating values.
void reportSystemSnapshot();        // Print the captured values when debugging.
void scanI2CBus();                  // Scan normal seven-bit addresses and report from the sketch.


// Start of setup
void setup() {
  
  if(SerialDebug) {
    Serial.begin(115200);
    uint32_t serialWaitStarted = millis();
    while(!Serial && (millis() - serialWaitStarted < 5000)) { } // Bound the commissioning-only USB wait.
    }

  // Configure MCU GPIOs
  pinMode(greenLed, OUTPUT); digitalWrite(greenLed, HIGH);  // set rgb leds as output, active LOW
  pinMode(  redLed, OUTPUT); digitalWrite(redLed,   HIGH);
  pinMode( blueLed, OUTPUT); digitalWrite(blueLed,  HIGH);
  pinMode(LIS2DW12_intPin, INPUT);  // define LIS2DW12 wake/sleep interrupt pin as WB5MMG input

  digitalWrite(greenLed, LOW);  // turn on green led while initializing

  STM32WB.getUID(UID);
  if(SerialDebug) {Serial.print("STM32WB5 MCU UID = 0x"); Serial.print(UID[0], HEX); Serial.print(UID[1], HEX); Serial.println(UID[2], HEX);} 

  // instantiate internal wire port
  I2C_BUS_1.begin(); // set master mode 
  I2C_BUS_1.setClock(400000); // I2C frequency at 400 kHz  
  delay(100);

  // test and configure sesors and flash on hardware reset
  // scan internal I2C port for devices...
  if(SerialDebug) {Serial.println("Scan internal I2C1 port for devices: ");}
  if(SerialDebug) scanI2CBus();

  // Read the LIS2DW12 identity and perform the complete commissioning sequence.
  bool sensorOK = true;                                               // Track LIS2DW12 initialization success.
  byte LIS2DW12_ChipID = 0;                                           // Hold the LIS2DW12 identity byte.
  sensorOK = LIS2DW12.getChipID(&LIS2DW12_ChipID);                    // Verify I2C communication.
  if(LIS2DW12_ChipID != 0x44) sensorOK = false;                       // Verify the expected device.
  if(SerialDebug) {
    Serial.print("LIS2DW12 "); Serial.print("I AM "); Serial.print(LIS2DW12_ChipID, HEX); Serial.print(" I should be "); Serial.println(0x44, HEX);
    Serial.println(" ");
  }

  if(sensorOK && SerialDebug) {Serial.println("LIS2DW12 is online..."); Serial.println(" ");}

  if(sensorOK) sensorOK = LIS2DW12.reset();                           // Reset sensor registers.

  if(sensorOK) sensorOK = LIS2DW12.selfTest(stress);                  // Run the electrical self-test.
  if(sensorOK && SerialDebug) {
    Serial.print("x-axis self test = "); Serial.print(stress[0], 1); Serial.println("mg, should be between 70 and 1500 mg");
    Serial.print("y-axis self test = "); Serial.print(stress[1], 1); Serial.println("mg, should be between 70 and 1500 mg");
    Serial.print("z-axis self test = "); Serial.print(stress[2], 1); Serial.println("mg, should be between 70 and 1500 mg");
  }
   
  if(sensorOK) {
    sensorOK = (stress[0] >= 70.0f && stress[0] <= 1500.0f) &&
               (stress[1] >= 70.0f && stress[1] <= 1500.0f) &&
               (stress[2] >= 70.0f && stress[2] <= 1500.0f);          // Enforce ST self-test limits.
  }

  if(sensorOK) sensorOK = LIS2DW12.reset();                           // Restore default registers.

  if(sensorOK && 1) {  // 1 to calculate accel bias offset, 0 to use those stored in the array
   if(SerialDebug) Serial.println("hold flat and motionless for bias calibration");
   delay(5000);                                                       // Allow time to place the board motionless.
   sensorOK = LIS2DW12.Compensation(fs, odr, Mode, lpMode, bw, lowNoise, offset); // Calculate acceleration offsets.

  if(sensorOK && SerialDebug) { // print out LIS2DW12 accel offset bias
    Serial.print("x-axis offset = "); Serial.print(offset[0]*1000.0f, 1); Serial.println(" mg");
    Serial.print("y-axis offset = "); Serial.print(offset[1]*1000.0f, 1); Serial.println(" mg");
    Serial.print("z-axis offset = "); Serial.print(offset[2]*1000.0f, 1); Serial.println(" mg"); Serial.println(" ");
   }
  }
  
  if(sensorOK) sensorOK = LIS2DW12.init(fs, odr, Mode, lpMode, bw, lowNoise, stMode); // Configure motion detection.
  if(sensorOK) sensorOK = LIS2DW12.configureFIFO(BYPASS, 0x00); // Keep FIFO empty until motion is detected.
  if(sensorOK) delay(100);                                           // Allow the configured sensor to settle.

  if(!sensorOK)
  {
    if(SerialDebug) Serial.println("LIS2DW12 initialization failed; device is not operational.");
    digitalWrite(greenLed, HIGH);                                    // Turn off the initialization indicator.

    while(1) {
      ledBlink(red, 100);                                            // Flash red briefly once per second.
      delay(900);
    }
  }

  // Test QSPI flash memory
  if(SerialDebug) Serial.println("QSPI Flash Check");
  bool flashStarted = SFLASH.begin();                           // Track whether the flash interface started.
  bool flashReady = flashStarted && SFLASH.identify(mid, did);  // Verify the fitted flash device.
  if(flashReady && SerialDebug) {
  Serial.print("MID = ");       Serial.println(mid, HEX); 
  Serial.print("DID = ");       Serial.println(did, HEX); 
  Serial.print("CAPACITY = ");  Serial.println(SFLASH.capacity());
  Serial.print("BLOCKSIZE = "); Serial.println(SFLASH.blockSize());
  Serial.print("PAGESIZE = ");  Serial.println(SFLASH.pageSize());
  Serial.print("LENGTH = ");    Serial.println(SFLASH.length()); Serial.println(" ");
  }

  if(flashStarted) SFLASH.end();                                // Release flash power and pins after begin().
  if(flashReady && logState.flashFault) flashReady = recoverLogState(); // Revalidate a retained flash fault before deployment.

  if(!logStateValid()) {                                        // Recover if retained state was lost.
    if(!recoverLogState()) setLogState(0, true);
  }
  else if(!logState.flashFault && (logState.page < MAX_LOG_PAGES)) {
    if(!pageIsErased(logState.page) && !recoverLogState()) setLogState(logState.page, true);
  }

  if(!flashReady) {
    setLogState(logState.page, true);                           // Prevent attempted writes on flash failure.
    if(SerialDebug) Serial.println("QSPI flash initialization failed; device is not operational.");
    digitalWrite(greenLed, HIGH);                               // Turn off the initialization indicator.

    while(1) {
      ledBlink(red, 100);                                       // Two red flashes identify a flash fault.
      delay(100);
      ledBlink(red, 100);
      delay(700);
    }
  }

  digitalWrite(greenLed, HIGH);  // turn off green led after initialization

  /* Set up the RTC alarm interrupt */
//  RTC.enableAlarm(RTC_MATCH_ANY);     // alarm once a second
  RTC.enableAlarm(RTC_MATCH_SS);      // alarm once a minute
//  RTC.enableAlarm(RTC_MATCH_MMSS);      // alarm once an hour
//  RTC.enableAlarm(RTC_MATCH_HHMMSS);  // alarm once a day
  RTC.attachInterrupt(alarmMatch);      // interrupt every time the alarm sounds

  attachInterrupt(LIS2DW12_intPin, LIS2DW12_inthandler, RISING);  // attach sleep/wake interrupt for INT1 pin output of LIS2DW12
  if(!LIS2DW12.getWakeSource(&wakeSource)) {                     // Clear any stale latched wake/sleep source.
    if(SerialDebug) Serial.println("LIS2DW12 interrupt-source clear failed; device is not operational.");

    while(1) {
      ledBlink(red, 100);                                        // Flash red briefly once per second.
      delay(900);
    }
  }
//  LIS2DW12.powerDown();

} /* End of Setup */


void loop() {

  /* LIS2DW12 interrupt handling */
  if(LIS2DW12_int_flag)
  {
   LIS2DW12_int_flag = false;    // clear the interrupt flag
   bool sensorOK = true;                                      // Track runtime LIS2DW12 operation results.
   if(!LIS2DW12.getStatus(&LIS2DW12_status)) {LIS2DW12_status = 0; sensorOK = false;} // Check wake or sleep status.

   if(!LIS2DW12.getWakeSource(&wakeSource)) {wakeSource = 0; sensorOK = false;} // Read the acceleration wake source.
   if(SerialDebug) {
    if(wakeSource & 0x20) Serial.println("Free fall detected!");
    if(wakeSource & 0x10) Serial.println("Sleep event detected!");
    if(wakeSource & 0x08) Serial.println("Wake-up event detected!");
    if(wakeSource & 0x04) Serial.println("Wake-up on x-axis detected!");
    if(wakeSource & 0x02) Serial.println("Wake-up on y-axis detected!");
    if(wakeSource & 0x01) Serial.println("Wake-up on z-axis detected!");
   }
   
   // A motion event starts a fresh post-trigger FIFO acquisition.
   if((LIS2DW12_status & 0x40) && !LIS2DW12_fifo_capture) {
   if(!LIS2DW12.deactivateWakeOnMotionInterrupt()) sensorOK = false; // Disable repeated motion interrupts.
   if(!LIS2DW12.deactivateSleepChangeInterrupt()) sensorOK = false; // Reserve the combined pin for FIFO threshold.
   eventWakeSource = wakeSource;   // Preserve the initiating event for the later flash record.
   InMotion = true;                // set motion state latch
   if(SerialDebug) Serial.println("** LIS2DW12 is awake; starting post-trigger FIFO capture! **");

   if(sensorOK) sensorOK = LIS2DW12.configureFIFO(BYPASS, 0x00); // Clear all pre-trigger FIFO data.
   if(sensorOK) sensorOK = LIS2DW12.configureFIFO(FIFO, 0x1F);   // Collect until the 31-sample threshold.
   if(sensorOK) sensorOK = LIS2DW12.activateFIFOThresholdInterrupt();

   if(sensorOK) LIS2DW12_fifo_capture = true;
   else {
     LIS2DW12.activateSleepChangeInterrupt();
     LIS2DW12.activateWakeOnMotionInterrupt();
   }
   }

    // The threshold interrupt marks 31 raw post-trigger samples.
    if(LIS2DW12_fifo_capture && (LIS2DW12_status & 0x80)) {
    if(!LIS2DW12.deactivateFIFOThresholdInterrupt()) sensorOK = false; // Quiesce the interrupt before reading.
    LIS2DW12_fifo_capture = false;

    if(!LIS2DW12.FIFOsamples(&FIFOstatus)) {FIFOstatus = 0; sensorOK = false;} // Read FIFO count and flags.
    if(FIFOstatus & 0x80) {                            // if the FIFO threshold is reached
      numFIFOSamples =  FIFOstatus & 0x3F;             // normally 31 samples at threshold
      const uint8_t rawFIFOSamples =
          numFIFOSamples >= 31 ? 31 : numFIFOSamples;
      const uint8_t validFIFOSamples =
          rawFIFOSamples ? rawFIFOSamples - 1 : 0;     // Discard transitional FIFO[0].
      memset(flashPage, 0xFF, sizeof(flashPage));      // Erased bytes remain available for future fields.
      flashPage[SAMPLE_COUNT_BYTE] = validFIFOSamples; // Save the number of trusted samples.
      flashPage[VERSION_BYTE] = RECORD_VERSION;        // Let future readers recognize this layout.
      int16_t fifoAccel[32][3] = {{0}};                // Hold one complete 192-byte FIFO.
      bool accelReadOK = rawFIFOSamples == 31;         // Require the programmed threshold count.
      uint32_t fifoReadDuration_us = 0;

      if(accelReadOK) {
        uint32_t fifoReadStarted_us = micros();
        accelReadOK =
            LIS2DW12.readFIFOAccelData(fifoAccel, rawFIFOSamples);
        fifoReadDuration_us = micros() - fifoReadStarted_us;
      }

      if(!accelReadOK) sensorOK = false;

      if(SerialDebug) {
        Serial.print("LIS2DW12 FIFO samples = ");
        Serial.println(numFIFOSamples);
        Serial.print("LIS2DW12 valid samples logged = ");
        Serial.println(validFIFOSamples);
        Serial.print("LIS2DW12 FIFO Wire.transfer time = ");
        Serial.print(fifoReadDuration_us);
        Serial.println(" us");
      }

      if(SerialDebug && accelReadOK)
        Serial.println("Discarded transitional FIFO[0].");

      for(uint8_t i = 1; accelReadOK && i < rawFIFOSamples; i++) {
         const uint8_t logIndex = i - 1;
         accelCount[0] = fifoAccel[i][0];              // Use the burst-read 14-bit data.
         accelCount[1] = fifoAccel[i][1];
         accelCount[2] = fifoAccel[i][2];
     
         ax = (float)accelCount[0]*aRes - offset[0];   // get actual g value, this depends on scale being set
         ay = (float)accelCount[1]*aRes - offset[1];   // could just store the two raw bytes for each axis
         az = (float)accelCount[2]*aRes - offset[2];   // but here we will scale and store half floats

         //  convert to milligs to make it easier to plot changes due to motion
         iax = FloattoHalf(1000.0f*ax);                // convert float data to half-float data for efficient storage
         iay = FloattoHalf(1000.0f*ay);
         iaz = FloattoHalf(1000.0f*az);

         // store data in QSPI flash memory
         // First page number is 0
         // Page 0xFFFF is the full-flash sentinel for this 128-Mbit device.
         if(!logState.flashFault && (logState.page < MAX_LOG_PAGES)) {
         flashPage[6*logIndex + 0] = (iax & 0xFF00) >> 8; // write 30 trusted post-trigger samples
         flashPage[6*logIndex + 1] = (iax & 0x00FF);      
         flashPage[6*logIndex + 2] = (iay & 0xFF00) >> 8;
         flashPage[6*logIndex + 3] = (iay & 0x00FF);      
         flashPage[6*logIndex + 4] = (iaz & 0xFF00) >> 8;
         flashPage[6*logIndex + 5] = (iaz & 0x00FF); }     
         
         if(SerialDebug) {  // print out FIFO data on the serial monitor
          Serial.print("ax = ");  Serial.print((int)(1000.0f*ax));         
          Serial.print(" ay = "); Serial.print((int)(1000.0f*ay)); 
          Serial.print(" az = "); Serial.print((int)(1000.0f*az)); Serial.println(" mg"); 
         }
      }

      if(!accelReadOK && SerialDebug) Serial.println("LIS2DW12 FIFO read failed; event discarded.");
      
      captureSystemSnapshot();                         // Timestamp the event and capture system values.
      reportSystemSnapshot();                          // Print only when SerialDebug is enabled.
      
    // add system info to the flash page
      if(sensorOK && accelReadOK && !logState.flashFault && (logState.page < MAX_LOG_PAGES)) {
      flashPage[PAGE_MSB_BYTE] = (logState.page & 0xFF00) >> 8;   // Embed the physical page for continuity checks.
      flashPage[PAGE_LSB_BYTE] =  logState.page & 0x00FF;
      flashPage[WAKE_SOURCE_BYTE] = eventWakeSource;              // Preserve the initiating motion-source bits.
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

      uint16_t crc = calculateCRC16(flashPage, CRC_MSB_BYTE);      // Protect all data and metadata before the CRC.
      flashPage[CRC_MSB_BYTE] = (crc & 0xFF00) >> 8;
      flashPage[CRC_LSB_BYTE] =  crc & 0x00FF;
      flashPage[MARKER_BYTE] = RECORD_MARKER;                      // Mark the page as a complete record.

      bool flashOK = SFLASH.begin();                               // Start flash only for the write.

      if(flashOK) {
        flashOK = SFLASH.program((uint32_t)logState.page * 256, flashPage, sizeof(flashPage));
        if(flashOK) flashOK = waitForFlash();
        SFLASH.end();
      }

      if(flashOK) {
        if(SerialDebug) {Serial.print("Wrote flash page: "); Serial.println(logState.page);}
        ledBlink(green, 1);
        setLogState(logState.page + 1, false);                     // Advance only after a verified write.
      }
      else {
        if(SerialDebug) Serial.println("QSPI flash write failed!");
        ledBlink(red, 10);
        setLogState(logState.page, true);                          // Stop writes until recovery.
      }
      }  
      else if(logState.page == MAX_LOG_PAGES) 
      {
       if(SerialDebug) {Serial.println("Reached last page of SPI flash!"); Serial.println("Data logging stopped!");}
      }
    }
    if(!LIS2DW12.configureFIFO(BYPASS, 0x00)) sensorOK = false; // Clear the FIFO after the event.
    if(!LIS2DW12.activateSleepChangeInterrupt()) sensorOK = false; // Restore inactivity notification.
    ledBlink(blue, 1); // toggle blue led when motion detected 
   }     

    // if sleep event
   if((LIS2DW12_status & 0x20) && !LIS2DW12_fifo_capture) {
   if(!LIS2DW12.activateWakeOnMotionInterrupt()) sensorOK = false; // Allow next threshold crossing to interrupt.
   InMotion = false;               // set motion state latch
   if(SerialDebug) Serial.println("** LIS2DW12 is asleep! **");

   if(!LIS2DW12.configureFIFO(BYPASS, 0x00)) sensorOK = false; // Keep FIFO empty while stationary.
   ledBlink(red, 1); // Red marks return to inactivity.
   }

   if(!sensorOK && SerialDebug) Serial.println("LIS2DW12 runtime operation failed.");
 
  } /* end of LIS2DW12 interrupt detect */ 


  /*RTC Timer*/
  if (alarmFlag) { // update serial output whenever there is a timer alarm
      alarmFlag = false;

    if(SerialDebug) {
      captureSystemSnapshot();                       // Diagnostic snapshot only while USB serial is in use.
      reportSystemSnapshot();
    }
  
    ledBlink(yellow, 1); // Yellow is the periodic operational heartbeat.
  }  /* End of RTC Timer handling */

  
  if(!LIS2DW12_int_flag && !alarmFlag)
    STM32WB.stop();  // wait in an ultra-low power state until an interrupt occurs
} /* end of main loop */


void LIS2DW12_inthandler()
{
  LIS2DW12_int_flag = true; 
  STM32WB.wakeup();
}


void alarmMatch()
{
  alarmFlag = true;
  STM32WB.wakeup();
}


void captureSystemSnapshot()
{
  RTC.getDateTime(Day, Month, Year, Hours, Minutes, Seconds);
  VDDA = STM32WB.readVREF();
  Temperature = STM32WB.readTemperature();
  VBAT = STM32WB.readBattery();
  iTemperature = FloattoHalf(Temperature);
  iVBAT = FloattoHalf(VBAT);
}


void reportSystemSnapshot()
{
  if(!SerialDebug) return;

  Serial.println(" ");
  Serial.print("RTC: ");
  if(Hours < 10) Serial.print("0");
  Serial.print(Hours);
  Serial.print(":");
  if(Minutes < 10) Serial.print("0");
  Serial.print(Minutes);
  Serial.print(":");
  if(Seconds < 10) Serial.print("0");
  Serial.print(Seconds);
  Serial.print(" ");
  Serial.print(Month);
  Serial.print("/");
  Serial.print(Day);
  Serial.print("/");
  Serial.println(Year);
  Serial.println(" ");

  Serial.print("VDDA = ");
  Serial.println(VDDA, 2);
  Serial.print("STM32WB MCU Temperature = ");
  Serial.println(Temperature, 2);
  Serial.print("VBAT = ");
  Serial.println(VBAT, 2);
  Serial.println(" ");
}


void scanI2CBus()
{
  uint8_t devices = 0;
  Serial.println("Scanning...");

  for(uint8_t address = 0x08; address <= 0x77; address++) {
    uint8_t error = i2c_1.probeAddress(address);

    if(error == 0) {
      Serial.print("I2C device found at address 0x");
      if(address < 16) Serial.print("0");
      Serial.println(address, HEX);
      devices++;
    } else if(error == 4) {
      Serial.print("Unknown error at address 0x");
      if(address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if(devices == 0) Serial.println("No I2C devices found\n");
  else Serial.println("I2C scan complete\n");
}


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


uint16_t calculateCRC16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFF;                                      // Start with the standard CRC-CCITT seed.

  while(length--) {
    crc ^= (uint16_t)(*data++) << 8;                          // Mix the next byte into the high CRC byte.

    for(uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;   // Apply the CRC-CCITT polynomial.
    }
  }

  return crc;                                                  // Return the final 16-bit CRC.
}


bool waitForFlash()
{
  uint32_t start = millis();                                   // Bound the flash busy wait.

  while(SFLASH.busy()) {
    if((millis() - start) >= FLASH_TIMEOUT_MS) return false;   // Report a timeout instead of hanging.
  }

  return (SFLASH.status() == SFLASH_STATUS_SUCCESS);           // Confirm the flash operation completed cleanly.
}


bool pageBufferIsErased()
{
  for(uint16_t i = 0; i < sizeof(flashPage); i++) {
    if(flashPage[i] != 0xFF) return false;                     // Any programmed byte means the page is not erased.
  }

  return true;                                                  // All bytes still match the erased state.
}


bool recordBufferIsValid(uint16_t expectedPage)
{
  uint8_t version = flashPage[VERSION_BYTE];                   // Read the stored record format.
  uint16_t storedPage = ((uint16_t)flashPage[PAGE_MSB_BYTE] << 8) | flashPage[PAGE_LSB_BYTE];
  uint16_t storedCRC = ((uint16_t)flashPage[CRC_MSB_BYTE] << 8) | flashPage[CRC_LSB_BYTE];

  return ((version == 1) || (version == RECORD_VERSION)) &&    // Accept existing v1 records while appending v2.
         ((version == 1) || (storedPage == expectedPage)) &&
         (flashPage[MARKER_BYTE] == RECORD_MARKER) &&
         (storedCRC == calculateCRC16(flashPage, CRC_MSB_BYTE));
}


bool pageIsErased(uint16_t page)
{
  bool success = SFLASH.begin();                               // Start the flash interface for one page read.

  if(success) {
    uint32_t pageSize = SFLASH.pageSize();                     // Query the fitted flash page size.
    uint32_t pageCount = pageSize ? (SFLASH.length() / pageSize) : 0;

    success = (pageSize == sizeof(flashPage)) &&
              (page < pageCount) &&
              SFLASH.read((uint32_t)page * pageSize, flashPage, sizeof(flashPage));

    if(success) success = pageBufferIsErased();                // Accept only an all-erased page.

    SFLASH.end();                                              // Release flash power and pins.
  }

  return success;
}


bool recoverLogState()
{
  bool success = SFLASH.begin();                               // Start flash before searching records.
  uint32_t page = 0;                                           // Will become the next writable page.

  if(success) {
    uint32_t pageSize = SFLASH.pageSize();                     // Use the actual flash geometry.
    uint32_t pageCount = pageSize ? (SFLASH.length() / pageSize) : 0;

    if(pageCount > MAX_LOG_PAGES) pageCount = MAX_LOG_PAGES;   // Reserve 0xFFFF as the full-flash sentinel.
    success = (pageSize == sizeof(flashPage)) && pageCount;

    uint32_t first = 0;                                        // Lower bound of the first erased page.
    uint32_t last = pageCount;                                 // Upper bound of the first erased page.

    while(success && (first < last)) {                         // Binary search the contiguous log.
      uint32_t middle = first + ((last - first) / 2);
      uint8_t marker;

      success = SFLASH.read((middle * pageSize) + MARKER_BYTE, &marker, 1);

      if(success) {
        if(marker == RECORD_MARKER) first = middle + 1;        // Marker means this page is occupied.
        else                        last = middle;             // Missing marker means this page may be free.
      }
    }

    page = first;

    if(success && page) {
      success = SFLASH.read((page - 1) * pageSize, flashPage, sizeof(flashPage));
      if(success) success = recordBufferIsValid((uint16_t)(page - 1)); // Reject a corrupt final record.
    }

    if(success && (page < pageCount)) {
      success = SFLASH.read(page * pageSize, flashPage, sizeof(flashPage));
      if(success) success = pageBufferIsErased();              // Confirm the resume page is unused.
    }

    SFLASH.end();                                              // Release flash power and pins.
  }

  if(success) setLogState((uint16_t)page, false);              // Store recovered append state.

  return success;
}


uint16_t FloattoHalf(float f)
{
  uint32_t x;                                                   // Holds the IEEE-754 float bits
  memcpy(&x, &f, sizeof(x));                                    // Copy without changing the bit pattern

  uint16_t sign = (x >> 16) & 0x8000;                           // Move the sign bit to half-float position
  uint32_t rawExponent = (x >> 23) & 0xFF;                      // Extract the float exponent
  uint32_t mantissa = x & 0x007FFFFF;                           // Extract the float mantissa

  if(rawExponent == 0xFF) return sign | (mantissa ? 0x7E00 : 0x7C00); // Encode NaN or infinity

  int32_t exponent = (int32_t)rawExponent - 127 + 15;           // Convert float exponent bias to half bias

  if(exponent >= 31) return sign | 0x7C00;                      // Saturate finite overflow to infinity

  if(exponent <= 0) {
    if(exponent < -10) return sign;                             // Underflow too small for half subnormal

    mantissa |= 0x00800000;                                     // Restore the hidden leading one
    uint32_t shift = 14 - exponent;                             // Calculate subnormal right shift
    uint32_t halfMantissa = mantissa >> shift;                  // Shift into half-float mantissa field
    uint32_t remainder = mantissa & ((1UL << shift) - 1);        // Keep bits lost by shifting
    uint32_t halfway = 1UL << (shift - 1);                      // Locate the half-way rounding bit

    if(remainder > halfway || (remainder == halfway && (halfMantissa & 1))) halfMantissa++; // Round to nearest even

    return sign | halfMantissa;                                 // Return signed half subnormal
  }

  uint32_t halfMantissa = mantissa >> 13;                       // Shift float mantissa into half field
  uint32_t remainder = mantissa & 0x1FFF;                       // Keep bits lost by shifting

  if(remainder > 0x1000 || (remainder == 0x1000 && (halfMantissa & 1))) {
    halfMantissa++;                                             // Round to nearest even

    if(halfMantissa == 0x0400) {
      halfMantissa = 0;                                         // Mantissa rounded into the exponent
      if(++exponent >= 31) return sign | 0x7C00;                // Saturate rounded overflow to infinity
    }
  }

  return sign | (exponent << 10) | halfMantissa;                // Return signed normal half-float
}


float HalftoFloat(uint16_t h)
{
  uint32_t sign = (uint32_t)(h & 0x8000) << 16;                 // Move the half sign bit to float position
  uint32_t exponent = (h >> 10) & 0x1F;                         // Extract the half exponent
  uint32_t mantissa = h & 0x03FF;                               // Extract the half mantissa
  uint32_t x;                                                   // Holds the IEEE-754 float bits

  if(exponent == 0) {
    if(mantissa == 0) {
      x = sign;                                                 // Preserve signed zero
    } else {
      int32_t e = -14;                                          // Start at the half subnormal exponent
      while(!(mantissa & 0x0400)) {
        mantissa <<= 1;                                         // Normalize the subnormal mantissa
        e--;
      }
      mantissa &= 0x03FF;                                       // Remove the hidden leading one
      x = sign | ((uint32_t)(e + 127) << 23) | (mantissa << 13); // Build normalized float
    }
  } else if(exponent == 0x1F) {
    x = sign | 0x7F800000 | (mantissa << 13);                   // Preserve infinity or NaN
  } else {
    x = sign | ((exponent + 112) << 23) | (mantissa << 13);     // Convert normal half to float
  }

  float f;                                                      // Holds the reconstructed float
  memcpy(&f, &x, sizeof(f));                                    // Copy bits back into a float
  return f;                                                     // Return the decoded value
}
