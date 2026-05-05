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
bool SerialDebug = true;

// QSPI flash variable
uint8_t  mid;
uint16_t did;
uint16_t page_number = 0;     // set the page number for flash page write
// create a pointer to the page_number variabe to maintain value during STANDBY for use after wakeup
//uint16_t _SECTION_BBRAM_DATA page_number = 0; // stored in flash
uint8_t  flashPage[256];      // array to hold the data for flash page write
uint32_t block_address, start, end;

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
#define WAKEUP_TIMEOUT         0x00000100   // wake from STANDBY on timeout
#define WAKEUP_WATCHDOG        0x00000200
#define WAKEUP_RESET           0x00000400


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
// create a pointer to the offset array to maintain values during STANDBY for use after wakeup
//float _SECTION_BBRAM_DATA offset[3] = {0.0f, 0.0f, 0.0f}; // stored in flash
float stress[3];        // holds results of the self test
uint8_t LIS2DW12_status = 0, wakeSource = 0, FIFOstatus = 0, numFIFOSamples = 0;

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
  
  // Check if wakeup from STANDBY wakeup
  if(STM32WB.resetCause() == RESET_WAKEUP) // then no need to reinitialize the sensors, etc
  {
  uint32_t wakeReason = STM32WB.wakeupReason(); 
  
  if(wakeReason == WAKEUP_PIN_3) {                     // this is the LIS2DW12 interrupt

   LIS2DW12_status = LIS2DW12.getStatus(); // read status of interrupts to clear

   if(LIS2DW12_status & 0x40) {       // is this a wakeup event?
   LIS2DW12.deactivateNoMotionInterrupt(); // only want the first  detection of the crossing of the motion threshold
   if(SerialDebug) Serial.println("** LIS2DW12 is awake! **");

   wakeSource = LIS2DW12.getWakeSource(); // find wakeup source
   if(wakeSource & 0x20 && SerialDebug) Serial.println("Free fall detected!");
   if(wakeSource & 0x10 && SerialDebug) Serial.println("Sleep event detected!");
   if(wakeSource & 0x08 && SerialDebug) Serial.println("Wake-up event detected!");
   if(wakeSource & 0x04 && SerialDebug) Serial.println("Wake-up on x-axis detected!");
   if(wakeSource & 0x02 && SerialDebug) Serial.println("Wake-up on y-axis detected!");
   if(wakeSource & 0x01 && SerialDebug) Serial.println("Wake-up on z-axis detected!");
    

    // collect and store the accel data around the wakeup event
    FIFOstatus = LIS2DW12.FIFOsamples();               // get acceleration history prior to and during wakeup event
//    if(1) {                                            // do even if the FIFO threshold is not reached
    if(FIFOstatus & 0x80) {                            // if the FIFO threshold is reached
      numFIFOSamples =  FIFOstatus & 0x3F;             // should be 32 sample
      for(uint8_t i; i < numFIFOSamples; i++) {        // read the FIFO data
         LIS2DW12.readAccelData(accelCount);           // get 14-bit signed accel data

         ax = (float)accelCount[0]*aRes - offset[0];   // get actual g value, this depends on scale being set
         ay = (float)accelCount[1]*aRes - offset[1];   // could just store the two raw bytes for each axis
         az = (float)accelCount[2]*aRes - offset[2];   // but here we will scale and store half floats

         //  convert to milligs to make it easier to plot changes due to motion
         iax = LIS2DW12.FloattoHalf(1000.0f*ax);       // convert float data to half-float data for efficient storage
         iay = LIS2DW12.FloattoHalf(1000.0f*ay);
         iaz = LIS2DW12.FloattoHalf(1000.0f*az);

         // store data in QSPI flash memory
         // First page number is 0
         // Last page number is 0xFFFF = 65,535 for 128 Mbit MX25L12835FZNI flash
         if(page_number < 0xFFFF) {
         flashPage[6*i + 0] = (iax & 0xFF00) >> 8;  // write six half-float bytes for each of 0 - 31 FIFO samples
         flashPage[6*i + 1] = (iax & 0x00FF);      
         flashPage[6*i + 2] = (iay & 0xFF00) >> 8;
         flashPage[6*i + 3] = (iay & 0x00FF);      
         flashPage[6*i + 4] = (iaz & 0xFF00) >> 8;
         flashPage[6*i + 5] = (iaz & 0x00FF); }     
         
         if(SerialDebug) {  // print out FIFO data on the serial monitor
          Serial.print("ax = ");  Serial.print((int)1000*ax);         
          Serial.print(" ay = "); Serial.print((int)1000*ay); 
          Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg"); Serial.println(" ");
         }
      }

 // Check RTC time, RTC kept alive in STANDBT mode
  RTC.getDateTime(Day, Month, Year, Hours, Minutes, Seconds); 
  if(SerialDebug) {
    Serial.println("RTC:");
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
      
    // add system info to the flash page
      if(page_number < 0xFFFF) {
      flashPage[197] = Seconds;                       // RTC time and date
      flashPage[198] = Minutes;
      flashPage[199] = Hours;
      flashPage[200] = Day;
      flashPage[201] = Month;
      flashPage[202] = Year;
      flashPage[203] = (iVBAT & 0xFF00) >> 8;         // VBAT from fuel gaugeVBAT monitor
      flashPage[204] =  iVBAT & 0x00FF;
      flashPage[205] = (iTemperature & 0xFF00) >> 8;  // STM32WB internal temperature  
      flashPage[206] =  iTemperature & 0x00FF;
 
      SFLASH.begin();
      SFLASH.program(page_number * 256, flashPage, 256);  // write next 256-byte page
      while (SFLASH.busy()) { }
      SFLASH.end();
      if(SerialDebug) {
        Serial.print("Wrote flash page: "); Serial.println(page_number);
      } 
      ledBlink(green, 1);
      page_number++;
      }  
      else if(page_number == 0xFFFF) 
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

   LIS2DW12.configureFIFO(BYPASS, 0x1F);              // clear the FIFO
   ledBlink(blue, 1);                                 // blink blue led when LIS2DW12 wakes from sleep
   }  // end of LIS2DW12 wake detect

   if(LIS2DW12_status & 0x20) { 
   LIS2DW12.activateNoMotionInterrupt(); // allow next detection of crossing of the motion threshold
   if(SerialDebug) Serial.println("** LIS2DW12 is asleep! **");     
   LIS2DW12.configureFIFO(CONT_TO_FIFO, 0x1F);        // restart the FIFO
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
  else // if waking from initial reset, need to configure sensors, etc ****************************************
  {
  // should only enter this section once at the beginning
  digitalWrite(greenLed, LOW);  // turn on green led while initializing

  // test and configure sesors and flash on hardware reset
  // scan internal I2C port for devices...
  if(SerialDebug) {Serial.println("Scan internal I2C1 port for devices: ");}
  i2c_1.I2Cscan(); 

  //   Read the LIS2DW12 Chip ID register, this is a good test of communication
  byte LIS2DW12_ChipID = LIS2DW12.getChipID();  // Read CHIP_ID register for LIS2DW12
  if(SerialDebug) {
    Serial.print("LIS2DW12 "); Serial.print("I AM "); Serial.print(LIS2DW12_ChipID, HEX); Serial.print(" I should be "); Serial.println(0x44, HEX);
    Serial.println(" ");
  }

  if(LIS2DW12_ChipID == 0x44) // check if all I2C sensors with WHO_AM_I have acknowledged
  {
   if(SerialDebug) {Serial.println("LIS2DW12 is online..."); Serial.println(" ");}

   LIS2DW12.reset();                                                // software reset before initialization
   delay(100);      

   LIS2DW12.selfTest(stress);                                       // perform sensor self test
   if(SerialDebug) {
    Serial.print("x-axis self test = "); Serial.print(stress[0], 1); Serial.println("mg, should be between 70 and 1500 mg");
    Serial.print("y-axis self test = "); Serial.print(stress[1], 1); Serial.println("mg, should be between 70 and 1500 mg");
    Serial.print("z-axis self test = "); Serial.print(stress[2], 1); Serial.println("mg, should be between 70 and 1500 mg");
   }
   
   LIS2DW12.reset();                                                // software reset before initialization
   delay(100);                                                     

   if(1) {  // 1 to calculate accel bias offset, 0 to use those stored in the array
    if(SerialDebug) Serial.println("hold flat and motionless for bias calibration");
   delay(5000);
   LIS2DW12.Compensation(fs, odr, Mode, lpMode, bw, lowNoise, offset); // quickly estimate offset bias in normal mode     

  if(SerialDebug) { // print out LIS2DW12 accel offset bias
    Serial.print("x-axis offset = "); Serial.print(offset[0]*1000.0f, 1); Serial.println(" mg");
    Serial.print("y-axis offset = "); Serial.print(offset[1]*1000.0f, 1); Serial.println(" mg");
    Serial.print("z-axis offset = "); Serial.print(offset[2]*1000.0f, 1); Serial.println(" mg"); Serial.println(" ");
   }
  }
   
   LIS2DW12.init(fs, odr, Mode, lpMode, bw, lowNoise, stMode);        // Initialize sensor in desired mode for application                     
   LIS2DW12.configureFIFO(fifoMode, 0x1F); // 32 levels of data
   delay(100); // let sensor settle

  }
  else 
  {
  if(LIS2DW12_ChipID != 0x44 && SerialDebug) Serial.println(" LIS2DW12 is not functioning!");  
  while(1);  // wait here indefinitely   
  }

  // Test QSPI flash memory
  if(SerialDebug) Serial.println("QSPI Flash Check");
  SFLASH.begin();
  SFLASH.identify(mid, did);
  if(SerialDebug) {
  Serial.print("MID = ");       Serial.println(mid, HEX); 
  Serial.print("DID = ");       Serial.println(did, HEX); 
  Serial.print("CAPACITY = ");  Serial.println(SFLASH.capacity());
  Serial.print("BLOCKSIZE = "); Serial.println(SFLASH.blockSize());
  Serial.print("PAGESIZE = ");  Serial.println(SFLASH.pageSize());
  Serial.print("LENGTH = ");    Serial.println(SFLASH.length()); Serial.println(" ");
  }
  
  if(1) { // 1 for flash erase, 0 to skip
    // erase QSPI flash memory in preparation for data logging
    if(SerialDebug) Serial.println("Wait while flash memory is erased!");
    start = millis();
    for (block_address = 0; block_address < 4096 * 1024; block_address += SFLASH.blockSize()) { 
      SFLASH.erase(block_address);
    }
    end = millis();
    if(SerialDebug) { 
    Serial.println("Flash erased!");
    Serial.print(" Erase time = "); Serial.print(((end - start) / 1000.0)); Serial.println(" sec");
    Serial.print((4096.0 * 1024.0) / ((end - start) / 1000.0));
    Serial.println(" bytes/second"); Serial.println(" ");
    }
  }  
  while (SFLASH.busy()) { }
  SFLASH.end();

  digitalWrite(greenLed, HIGH);  // turn off green led after initialization

  }  // end of reset cause check and initial configuration ****************************************
  
  
    LIS2DW12.getStatus();                        // read status of interrupt to clear
//  LIS2DW12.powerDown();

  STM32WB.standby(LIS2DW12_intPin, RISING, 60000); // stay in STANDBY mode until timeout (1 min) occurs or wakeup pin goes HIGH  

} /* End of Setup */


void loop() {

// should never get here...

} /* end of main loop */


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
