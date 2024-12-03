#include "Arduino.h"
#include "STM32WB.h"
#include "I2Cdev.h"
#include "LIS2DW12.h"
#include "SFLASH.h"
#include "RTC.h"
#include "TimerMillis.h"

#define I2C_BUS_1    Wire1              // Define the internal I2C bus (Wire1 instance)  

I2Cdev             i2c_1(&I2C_BUS_1);   // Instantiate the I2Cdev object and point to the desired I2C bus //

TimerMillis SensorTimer;  // set up sensor polling timer
volatile bool Sensor_flag = false;

float VBAT, VDDA, Temperature;
uint32_t UID[3] = {0, 0, 0};
volatile bool USBConnected = false; 
volatile bool SerialDebug = true;

uint8_t mid;
uint16_t did;
uint16_t page_number = 0;     // set the page mumber for flash page write
uint8_t  sector_number = 0;   // set the sector number for sector write
uint8_t  flashPage[256];      // array to hold the data for flash page write

uint8_t seconds, minutes, hours, day, month, year;
uint8_t Seconds, Minutes, Hours, Day, Month, Year;
volatile bool alarmFlag = false; // for RTC alarm interrupt

// define rgb led pins and colors
#define greenLed 22 // green led active LOW
#define redLed   23 // red led active LOW
#define blueLed  24 // blue led active LOW

// allowed colors
#define red     0
#define green   1
#define blue    2
#define yellow  3
#define magenta 4
#define cyan    5

//LIS2DW12 definitions
#define LIS2DW12_intPin   33    // wake and sleep interrupts on same interrupt

// Specify sensor parameters //
LPMODE   lpMode = LIS2DW12_LP_MODE_1;      // choices are low power modes 1, 2, 3, or 4
MODE     mode   = LIS2DW12_MODE_LOW_POWER; // choices are low power, high performance, and one shot modes
ODR      odr    = LIS2DW12_ODR_12_5_1_6HZ; //  1.6 Hz in lpMode, max is 200 Hz in LpMode
FS       fs     = LIS2DW12_FS_2G;          // choices are 2, 4, 8, or 16 g
BW_FILT  bw     = LIS2DW12_BW_FILT_ODR2;   // choices are ODR divided by 2, 4, 10, or 20
FIFOMODE fifoMode = BYPASS;                // capture 32 samples of data before wakeup event, about 2 secs at 25 Hz
bool lowNoise = false;                     // low noise or lowest power

float aRes = 0;         // Sensor data scale in mg/LSB
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t tempCount;      // 8-bit signed temperature output
uint8_t rawTempCount;   // raw temperature output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float ax, ay, az;       // variables to hold latest sensor data values 
float offset[3];        // holds accel bias offsets
float stress[3];        // holds results of the self test
uint8_t LIS2DW12_status = 0, wakeSource = 0, FIFOstatus = 0, numFIFOSamples = 0;

// Logic flags to keep track of device states
volatile bool LIS2DW12_int_flag = false;
volatile bool InMotion = false;

LIS2DW12 LIS2DW12(&i2c_1); // instantiate LIS2DW12 class


void setup() {
  Serial.begin(115200);
//  while (!Serial) { }
  delay(4000);

  pinMode(greenLed, OUTPUT); digitalWrite(greenLed, HIGH);  // set rgb leds as output, active LOW
  pinMode(  redLed, OUTPUT); digitalWrite(redLed, HIGH);
  pinMode( blueLed, OUTPUT); digitalWrite(blueLed, HIGH);
  
  // toggle rgb indicator leds
  digitalWrite(redLed, LOW); delay(1000); digitalWrite(redLed, HIGH); delay(1000);
  digitalWrite(greenLed, LOW); delay(1000); digitalWrite(greenLed, HIGH); delay(1000);
  digitalWrite(blueLed, LOW); delay(1000); digitalWrite(blueLed, HIGH); delay(1000);

  for(uint8_t ii = 0; ii < 6; ii++) {
    ledBlink(ii, 1000);   // run through the six defined rgb led colors
  }

  STM32WB.getUID(UID);
  if(SerialDebug) {Serial.print("STM32L4 MCU UID = 0x"); Serial.print(UID[0], HEX); Serial.print(UID[1], HEX); Serial.println(UID[2], HEX);} 

  pinMode(LIS2DW12_intPin, INPUT);  // define LIS2DW12 wake/sleep interrupt pin as WB5MMG input

  // instantiate internal wire port
  I2C_BUS_1.begin(); // set master mode 
  I2C_BUS_1.setClock(400000); // I2C frequency at 400 kHz  
  delay(1000);

// scan internal I2C port for devices...
  Serial.println("Scan internal I2C1 port for devices: ");
  i2c_1.I2Cscan(); 

  // Check device IDs
  // Read the LIS2DW12 Chip ID register, this is a good test of communication
  byte LIS2DW12_ChipID = LIS2DW12.getChipID();  // Read CHIP_ID register for LIS2DW12
  if(SerialDebug) {
    Serial.print("LIS2DW12 "); Serial.print("I AM "); Serial.print(LIS2DW12_ChipID, HEX); Serial.print(" I should be "); Serial.println(0x44, HEX);
    Serial.println(" ");
  }

  if(LIS2DW12_ChipID == 0x44) // check if all I2C sensors with WHO_AM_I have acknowledged
  {
  Serial.println("LIS2DW12 is online..."); Serial.println(" ");

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

   aRes = 0.000244f * (1 << fs);                                    // scale resolutions per LSB for the sensor at 14-bit data 

   if(SerialDebug) Serial.println("hold flat and motionless for bias calibration");
   delay(5000);
   LIS2DW12.Compensation(fs, odr, mode, lpMode, bw, lowNoise, offset); // quickly estimate offset bias in normal mode
   if(SerialDebug) {
    Serial.print("x-axis offset = "); Serial.print(offset[0]*1000.0f, 1); Serial.println(" mg");
    Serial.print("y-axis offset = "); Serial.print(offset[1]*1000.0f, 1); Serial.println(" mg");
    Serial.print("z-axis offset = "); Serial.print(offset[2]*1000.0f, 1); Serial.println(" mg");
   }

   LIS2DW12.init(fs, odr, mode, lpMode, bw, lowNoise);               // Initialize sensor in desired mode for application                     
   LIS2DW12.configureFIFO(fifoMode, 0x1F); // 32 levels of data
   delay(100); // let sensor settle
  }
  else 
  {
  if(LIS2DW12_ChipID != 0x44) Serial.println(" LIS2DW12 is not functioning!");  
  while(1);  // wait here indefinitely   
  }

  // Test QSPI flash memory
  Serial.println("QSPI Flash Check");
  SFLASH.begin();
  SFLASH.identify(mid, did);
  Serial.print("MID = ");       Serial.println(mid, HEX); 
  Serial.print("DID = ");       Serial.println(did, HEX); 
  Serial.print("CAPACITY = ");  Serial.println(SFLASH.capacity());
  Serial.print("BLOCKSIZE = "); Serial.println(SFLASH.blockSize());
  Serial.print("PAGESIZE = ");  Serial.println(SFLASH.pageSize());
  Serial.print("LENGTH = ");    Serial.println(SFLASH.length()); Serial.println(" ");
  SFLASH.end();

  SensorTimer.start(callbackSensor, 10000, 2000);       // 2 second period, delayed 10 seconds 

  /* Set up the RTC alarm interrupt */
  RTC.enableAlarm(RTC_MATCH_ANY); // alarm once a second
  
  RTC.attachInterrupt(alarmMatch); // interrupt every time the alarm sounds
  attachInterrupt(LIS2DW12_intPin, myinthandler, RISING);  // attachsleep/wake interrupt for INT1 pin output of LIS2DW12

  LIS2DW12.getStatus(); // read status of interrupts to clear
}


void loop() {

  /* LIS2DW12 interrupt handling */
  if(LIS2DW12_int_flag)
  {
   LIS2DW12_int_flag = false;    // clear the interrupt flag

   LIS2DW12_status = LIS2DW12.getStatus(); // check whether interrupt event is wakeup or sleep

   // if wake event
   if(LIS2DW12_status & 0x40) {
   InMotion = true;               // set motion state latch
   if(SerialDebug) Serial.println("** LIS2DW12 is awake! **");

   ledBlink(magenta, 10); // toggle magenta led when motion detected 
  }     

   // if sleep event
   if(LIS2DW12_status & 0x20) {
   InMotion = false;               // set motion state latch
   if(SerialDebug) Serial.println("** LIS2DW12 is asleep! **");
      
   ledBlink(yellow, 10); // toggle magenta led when motion detected 
   }
 
  } /* end of LIS2DW12 interrupt detect */ 


  /* Sensor timer */
  if(Sensor_flag) {  // read accel if device is awake
    Sensor_flag = false;

    if(InMotion) { // read accel data only if device is awake and in motion
      
    LIS2DW12.readAccelData(accelCount); // get 14-bit signed accel data

     // Now we'll calculate the accleration value into actual g's
      ax = (float)accelCount[0]*aRes - offset[0];  // get actual g value, this depends on scale being set
      ay = (float)accelCount[1]*aRes - offset[1];   
      az = (float)accelCount[2]*aRes - offset[2]; 
     
      if(SerialDebug) {
        Serial.println(" ");
        Serial.print("ax = ");  Serial.print((int)1000*ax);  
        Serial.print(" ay = "); Serial.print((int)1000*ay); 
        Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
        Serial.println(" ");
      }
    }
    
  } /* end of sensor timer handling */
  

  /*RTC Timer*/
  if (alarmFlag) { // update serial output whenever there is a timer alarm
      alarmFlag = false;
      
  Serial.println("RTC:");
  Day = RTC.getDay();
  Month = RTC.getMonth();
  Year = RTC.getYear();
  Seconds = RTC.getSeconds();
  Minutes = RTC.getMinutes();
  Hours   = RTC.getHours();     
  if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
  Serial.print(":"); 
  if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
  Serial.print(":"); 
  if(Seconds < 10) {Serial.print("0"); Serial.println(Seconds);} else Serial.println(Seconds);  

  Serial.print(Month); Serial.print("/"); Serial.print(Day); Serial.print("/"); Serial.println(Year);
  Serial.println(" ");
  
  VDDA = STM32WB.readVREF();
  Temperature = STM32WB.readTemperature();
  USBConnected = USBDevice.attached();
  VBAT = STM32WB.readBattery();

  if(SerialDebug)   Serial.print("VDDA = "); Serial.println(VDDA, 2); 
  if(SerialDebug)   Serial.print("STM32WB MCU Temperature = "); Serial.println(Temperature, 2);
  if(USBConnected && SerialDebug) Serial.println("USB connected!");
  if(SerialDebug)   Serial.print("VBAT = "); Serial.println(VBAT, 2); 
  
  // toggle indicator led
  ledBlink(cyan, 1);
  
  } /* End of RTC Timer Handling */

  STM32WB.stop(); // stay in lowest power mode until interrupt occurs
  
} /* end of main loop */


/* interrupt handlers */
void callbackSensor()
{
  Sensor_flag = true; 
  STM32WB.wakeup();
}


void alarmMatch()
{
  alarmFlag = true;
  STM32WB.wakeup();
}


void myinthandler()
{
  LIS2DW12_int_flag = true; 
  STM32WB.wakeup();
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

   // set rgb led current
  digitalWrite(redLed, _out3); 
  digitalWrite(greenLed, _out1);  
  digitalWrite(blueLed, _out2);  
  delay(duration);
  digitalWrite(redLed, HIGH); 
  digitalWrite(greenLed, HIGH);  
  digitalWrite(blueLed, HIGH);  
  }
