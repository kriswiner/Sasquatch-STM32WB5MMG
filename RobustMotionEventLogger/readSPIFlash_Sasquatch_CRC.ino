/*
  readSPIFlash_Sasquatch_CRC

  Reads sequential Sasquatch motion records until the first erased page. Each
  page is checked for a marker, supported format version, sample count, embedded
  page number (version 2), and CRC before any values are decoded. Lines beginning
  with '#' describe an event; the remaining comma-delimited lines are suitable
  for plotting or spreadsheet analysis.
*/

#include "SFLASH.h"
#include "STM32WB.h"

const uint8_t RECORD_MARKER = 0x73;
const uint8_t RECORD_VERSION_1 = 1;
const uint8_t RECORD_VERSION_2 = 2;
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
const uint16_t CRC_MSB_BYTE = 253;
const uint16_t CRC_LSB_BYTE = 254;
const uint16_t MARKER_BYTE = 255;
uint8_t flashPage[256];
uint32_t max_page_number;
uint32_t page_number;
uint8_t mid;
uint16_t did;

float ax, ay, az, VBAT, Temperature;               // float values reconstructed from flash data
uint8_t Seconds, Minutes, Hours, Day, Month, Year; // time/date values stored on flash
uint8_t recordVersion, sampleCount, wakeSource, fullScaleCode, dataRateCode;
uint16_t storedPage;
uint16_t iax, iay, iaz, iVBAT, iTemperature;       // half float values stored on flash

void setup(void)
{ 
  Serial.begin(115200);
  while (!Serial) { }
  Serial.println("Serial enabled!");

  // Test QSPI flash memory
  Serial.println("QSPI Flash Check");
  if(!SFLASH.begin() || !SFLASH.identify(mid, did)) {
    Serial.println("QSPI flash initialization failed!");
    return;
  }

  Serial.print("MID = ");       Serial.println(mid, HEX); 
  Serial.print("DID = ");       Serial.println(did, HEX); 
  Serial.print("CAPACITY = ");  Serial.println(SFLASH.capacity());
  Serial.print("BLOCKSIZE = "); Serial.println(SFLASH.blockSize());
  Serial.print("PAGESIZE = ");  Serial.println(SFLASH.pageSize());
  Serial.print("LENGTH = ");    Serial.println(SFLASH.length()); Serial.println(" ");

  if(SFLASH.pageSize() != sizeof(flashPage)) {
    Serial.println("Unsupported QSPI flash page size!");
    SFLASH.end();
    return;
  }

  max_page_number = SFLASH.length() / SFLASH.pageSize();
  if(max_page_number > 0xFFFF) max_page_number = 0xFFFF;

  // read the Sasquatch QSPI flash
  for(page_number = 0; page_number < max_page_number; page_number++) {

    if(!SFLASH.read(page_number * sizeof(flashPage), flashPage, sizeof(flashPage))) {
      Serial.print("Flash read failed at page "); Serial.println(page_number);
      break;
    }

    if(flashPage[MARKER_BYTE] == 0xFF) {
      if(pageBufferIsErased()) {
        Serial.print("Reached first unwritten page: "); Serial.println(page_number);
      }
      else {
        Serial.print("Incomplete record at page "); Serial.println(page_number);
      }
      break;
    }

    if(flashPage[MARKER_BYTE] != RECORD_MARKER) {
      Serial.print("Invalid marker at page "); Serial.println(page_number);
      break;
    }

    recordVersion = flashPage[VERSION_BYTE];
    if((recordVersion != RECORD_VERSION_1) && (recordVersion != RECORD_VERSION_2)) {
      Serial.print("Unsupported record version at page "); Serial.println(page_number);
      break;
    }

    sampleCount = flashPage[SAMPLE_COUNT_BYTE];       // Version 1 and 2 use the same sample-count byte.
    if((sampleCount == 0) || (sampleCount > 32)) {
      Serial.print("Invalid sample count at page "); Serial.println(page_number);
      break;
    }

    uint16_t storedCRC = ((uint16_t)flashPage[CRC_MSB_BYTE] << 8) | flashPage[CRC_LSB_BYTE];
    uint16_t calculatedCRC = calculateCRC16(flashPage, CRC_MSB_BYTE);

    if(storedCRC != calculatedCRC) {
      Serial.print("CRC failure at page "); Serial.println(page_number);
      break;
    }

    if(recordVersion == RECORD_VERSION_2) {
      storedPage = ((uint16_t)flashPage[PAGE_MSB_BYTE] << 8) | flashPage[PAGE_LSB_BYTE];
      if(storedPage != page_number) {
        Serial.print("Embedded page mismatch at page "); Serial.println(page_number);
        break;
      }

      wakeSource = flashPage[WAKE_SOURCE_BYTE];       // Raw LIS2DW12 wake-source register bits.
      fullScaleCode = flashPage[FULL_SCALE_BYTE];     // 0, 1, 2, 3 represent +/-2, 4, 8, 16 g.
      dataRateCode = flashPage[DATA_RATE_BYTE];       // LIS2DW12 ODR enum code from the logger.
      Seconds = flashPage[RTC_SECONDS_BYTE];           // Extract version 2 RTC time and date.
      Minutes = flashPage[RTC_MINUTES_BYTE];
      Hours =   flashPage[RTC_HOURS_BYTE];
      Day =     flashPage[RTC_DAY_BYTE];
      Month =   flashPage[RTC_MONTH_BYTE];
      Year =    flashPage[RTC_YEAR_BYTE];
      iVBAT = uint16_t((uint16_t)flashPage[VBAT_MSB_BYTE] << 8 | flashPage[VBAT_LSB_BYTE]);
      iTemperature = uint16_t((uint16_t)flashPage[TEMPERATURE_MSB_BYTE] << 8 | flashPage[TEMPERATURE_LSB_BYTE]);
    }
    else {
      storedPage = page_number;                       // Version 1 did not embed page or configuration.
      wakeSource = fullScaleCode = dataRateCode = 0xFF;
      Seconds = flashPage[197];                       // Extract legacy version 1 RTC time and date.
      Minutes = flashPage[198];
      Hours =   flashPage[199];
      Day =     flashPage[200];
      Month =   flashPage[201];
      Year =    flashPage[202];
      iVBAT = uint16_t((uint16_t)flashPage[203] << 8 | flashPage[204]);
      iTemperature = uint16_t((uint16_t)flashPage[205] << 8 | flashPage[206]);
    }

    VBAT = HalftoFloat(iVBAT);                        // Convert stored half-floats back to normal floats.
    Temperature = HalftoFloat(iTemperature);

    Serial.print("# page="); Serial.print(storedPage);  // Print compact event context before its data rows.
    Serial.print(",version="); Serial.print(recordVersion);
    Serial.print(",samples="); Serial.print(sampleCount);
    if(recordVersion == RECORD_VERSION_2) {
      Serial.print(",wake=0x"); Serial.print(wakeSource, HEX);
      Serial.print(",full-scale=+/-"); Serial.print(fullScaleG(fullScaleCode)); Serial.print("g");
      Serial.print(",ODR="); Serial.print(dataRateName(dataRateCode));
    }
    Serial.println();

    // Comma delimited output for spreadsheet analysis
    if(Month < 10) {Serial.print("0"); Serial.print(Month);} else Serial.print(Month);
    Serial.print("/");Serial.print(Day); Serial.print("/");Serial.print(Year); Serial.print(" ");
    if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
    Serial.print(":"); 
    if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
    Serial.print(":"); 
    if(Seconds < 10) {Serial.print("0"); Serial.print(Seconds);} else Serial.print(Seconds); Serial.print(",");      

    Serial.print(Temperature, 2); Serial.print(","); Serial.println(VBAT, 2);  

  // In the LIS2DW12 accelerometer's FIFO mode, the oldest data is at the beginning of the FIFO buffer
  // and the newest data is added at the end.
  for(int16_t i = sampleCount - 1; i >= 0; i--) {
      iax = uint16_t ( (uint16_t)flashPage[6*i + 0] << 8 | flashPage[6*i + 1]); // reconstruct accel data
      iay = uint16_t ( (uint16_t)flashPage[6*i + 2] << 8 | flashPage[6*i + 3]);
      iaz = uint16_t ( (uint16_t)flashPage[6*i + 4] << 8 | flashPage[6*i + 5]);

      ax = HalftoFloat(iax);   
      ay = HalftoFloat(iay);  
      az = HalftoFloat(iaz);  

      Serial.print(sampleCount - i); Serial.print(",");   // should be acceleration in milligs
      Serial.print((int)ax); Serial.print(",");  // 12-bit data resolution is ~1 millig
      Serial.print((int)ay); Serial.print(","); 
      Serial.println((int)az); // can subtract 1000 here to remove gravity
    }    
  }

  SFLASH.end();
}

void loop(void)
{
}


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


bool pageBufferIsErased()
{
  for(uint16_t i = 0; i < sizeof(flashPage); i++) {
    if(flashPage[i] != 0xFF) return false;
  }

  return true;
}


uint8_t fullScaleG(uint8_t code)
{
  return (code <= 3) ? (2 << code) : 0;  // LIS2DW12 FS codes map to 2, 4, 8, and 16 g.
}


const char *dataRateName(uint8_t code)
{
  switch(code) {
    case 0x00: return "power-down";
    case 0x01: return "12.5/1.6Hz";
    case 0x02: return "12.5Hz";
    case 0x03: return "25Hz";
    case 0x04: return "50Hz";
    case 0x05: return "100Hz";
    case 0x06: return "200Hz";
    case 0x07: return "400/200Hz";
    case 0x08: return "800/200Hz";
    case 0x09: return "1600/200Hz";
    default:   return "unknown";
  }
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
