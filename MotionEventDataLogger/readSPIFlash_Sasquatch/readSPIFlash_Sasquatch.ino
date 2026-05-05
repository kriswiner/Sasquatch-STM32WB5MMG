/* readSPIFlash_Sasquatch
 *  
2026 Copyright Tlera Corporation 

May 4, 2026

Sketch to read the QSPI Flash on the Sasquatch.v01, reconstruct the sensor data, and output CMS-compatible data for plotting.
 */

#include "SFLASH.h"
#include "STM32WB.h"

// Highest page number is 0xFFFF = 65535 for 128 Mbit flash
uint16_t max_page_number = 0xFFFF;
unsigned char flashPage[256];
uint16_t page_number = 0;
uint8_t mid;
uint16_t did;

float ax, ay, az, VBAT, Temperature;               // float values reconstructed from flash data
uint8_t Seconds, Minutes, Hours, Day, Month, Year; // time/date values stored on flash
uint16_t iax, iay, iaz, iVBAT, iTemperature;       // half float values stored on flash

void setup(void)
{ 
  Serial.begin(115200);
  while (!Serial) { }
  Serial.println("Serial enabled!");

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
 
  // read the Sasquatch QSPI flash
  for(page_number = 0; page_number < 6; page_number++)  { // change the page number limit to correspond to number of pages logged

  SFLASH.read(page_number * 256, flashPage, sizeof(flashPage));
           
    Seconds = flashPage[197]; // extract RTC time/date
    Minutes = flashPage[198];
    Hours =   flashPage[199];
    Day =     flashPage[200];
    Month =   flashPage[201];
    Year =    flashPage[202];

    iTemperature = uint16_t ( (uint16_t)flashPage[205] << 8 | flashPage[206]); // reconstruct temperature
    Temperature = HalftoFloat(iTemperature);

    iVBAT = uint16_t ( (uint16_t)flashPage[203] << 8 | flashPage[204]); // reconstruct battery voltage
    VBAT = HalftoFloat(iVBAT);

    // Comma delimited output for spreadsheet analysis
    if(Month < 10) {Serial.print("0"); Serial.print(Month);} else Serial.print(Month);
    Serial.print("/");Serial.print(Day); Serial.print("/");Serial.print(Year); Serial.print(" ");
    if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
    Serial.print(":"); 
    if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
    Serial.print(":"); 
    if(Seconds < 10) {Serial.print("0"); Serial.print(Seconds);} else Serial.print(Seconds); Serial.print(",");      

    Serial.print(Temperature, 2); Serial.print(","); Serial.println(VBAT, 2);  

    for( uint8_t i = 0; i < 32; i++) { // maximum 32 entries in FIFO per wake event
      iax = uint16_t ( (uint16_t)flashPage[6*i + 0] << 8 | flashPage[6*i + 1]); // reconstruct accel data
      iay = uint16_t ( (uint16_t)flashPage[6*i + 2] << 8 | flashPage[6*i + 3]);
      iaz = uint16_t ( (uint16_t)flashPage[6*i + 4] << 8 | flashPage[6*i + 5]);

//      ax = HalftoFloat(iax); if(ax > 2.0f) ax = 0.0f;
//      ay = HalftoFloat(iay); if(ay > 2.0f) ay = 0.0f;
//      az = HalftoFloat(iaz); if(az > 2.0f) az = 0.0f;

      Serial.print(i); Serial.print(",");   // should be inertial acceleration in milligs
      Serial.print(ax, 1); Serial.print(","); 
      Serial.print(ay, 1); Serial.print(","); 
      Serial.println(az, 1);
    }    
  }

  while (SFLASH.busy()) { }
  SFLASH.end();
}

void loop(void)
{
}


float HalftoFloat(uint16_t n)
{
    uint16_t frac = (n & 0x03FF) | 0x0400;
    int  exp = ((n & 0x7C00) >> 10) - 25;
    float m;

    if(frac == 0 && exp == 0x1F)
       m = INFINITY;
    else if (frac || exp)
        m = frac * pow(2, exp);
    else
        m = 0;

    return (n & 0x8000) ? -m : m;
}
