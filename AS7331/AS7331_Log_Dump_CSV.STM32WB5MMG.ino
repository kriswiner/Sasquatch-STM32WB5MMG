/*
 * AS7331 QSPI log dump
 *
 * Reads checkpoint-4 pages and prints CSV suitable for saving from the Serial
 * monitor. It never writes or erases flash.
 */

#include <Arduino.h>
#include <SFLASH.h>

const uint8_t LOG_MAGIC_0 = 'U';
const uint8_t LOG_MAGIC_1 = 'V';
const uint8_t LOG_VERSION = 1;
const uint8_t PAGE_TYPE_SESSION = 1;
const uint8_t PAGE_TYPE_SAMPLE = 2;
const uint8_t LOG_MARKER = 0xA7;
const uint16_t LOG_PAGE_SIZE = 256;
const uint16_t CRC_MSB_BYTE = 253;
const uint16_t CRC_LSB_BYTE = 254;
const uint16_t MARKER_BYTE = 255;

uint8_t flashPage[LOG_PAGE_SIZE];


uint16_t getU16(uint16_t offset)
{
  return (uint16_t)(((uint16_t)flashPage[offset] << 8) |
                    flashPage[offset + 1]);
}


int16_t getI16(uint16_t offset)
{
  return (int16_t)getU16(offset);
}


uint32_t getU32(uint16_t offset)
{
  return ((uint32_t)flashPage[offset] << 24) |
         ((uint32_t)flashPage[offset + 1] << 16) |
         ((uint32_t)flashPage[offset + 2] << 8) |
         flashPage[offset + 3];
}


float getFloat(uint16_t offset)
{
  union {
    float floatingPoint;
    uint32_t bits;
  } conversion;

  conversion.bits = getU32(offset);
  return conversion.floatingPoint;
}


uint16_t calculateCRC16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFF;
  while(length--) {
    crc ^= (uint16_t)(*data++) << 8;
    for(uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) :
                             (uint16_t)(crc << 1);
    }
  }
  return crc;
}


bool pageIsErased()
{
  for(uint16_t index = 0; index < LOG_PAGE_SIZE; index++) {
    if(flashPage[index] != 0xFF) return false;
  }
  return true;
}


bool pageIsValid(uint32_t physicalPage)
{
  return flashPage[0] == LOG_MAGIC_0 &&
         flashPage[1] == LOG_MAGIC_1 &&
         flashPage[2] == LOG_VERSION &&
         getU32(4) == physicalPage &&
         flashPage[MARKER_BYTE] == LOG_MARKER &&
         getU16(CRC_MSB_BYTE) ==
           calculateCRC16(flashPage, CRC_MSB_BYTE);
}


void printSessionPage()
{
  Serial.print("# session_page="); Serial.println(getU32(4));
  Serial.print("# session_sequence="); Serial.println(getU32(8));
  Serial.print("# session_id="); Serial.println(getU32(12));
  Serial.print("# format_uuid=");
  for(uint8_t index = 0; index < 16; index++) {
    Serial.write(flashPage[24 + index]);
  }
  Serial.println();
  Serial.print("# start=");
  Serial.print(getU16(40)); Serial.print("-");
  Serial.print(flashPage[42]); Serial.print("-");
  Serial.print(flashPage[43]); Serial.print(" ");
  Serial.print(flashPage[44]); Serial.print(":");
  Serial.print(flashPage[45]); Serial.print(":");
  Serial.println(flashPage[46]);
  Serial.print("# sample_interval_ms="); Serial.println(getU32(47));
  Serial.print("# thermal_recheck_ms="); Serial.println(getU32(51));
  Serial.print("# gain="); Serial.println(flashPage[55]);
  Serial.print("# time="); Serial.println(flashPage[56]);
  Serial.print("# clock="); Serial.println(flashPage[57]);
  Serial.print("# divider_enabled="); Serial.println(flashPage[58]);
  Serial.print("# divider="); Serial.println(flashPage[59]);
  Serial.print("# high_temperature_c=");
  Serial.println((float)getI16(60) / 100.0f, 2);
  Serial.print("# resume_temperature_c=");
  Serial.println((float)getI16(62) / 100.0f, 2);
}


void printSamplePage()
{
  Serial.print(getU32(12)); Serial.print(",");
  Serial.print(getU32(8)); Serial.print(",");
  Serial.print(getU16(24)); Serial.print("-");
  Serial.print(flashPage[26]); Serial.print("-");
  Serial.print(flashPage[27]); Serial.print(" ");
  Serial.print(flashPage[28]); Serial.print(":");
  Serial.print(flashPage[29]); Serial.print(":");
  Serial.print(flashPage[30]); Serial.print(".");
  Serial.print(getU16(31)); Serial.print(",");
  Serial.print(getU32(33)); Serial.print(",");
  Serial.print(getU16(37)); Serial.print(",");
  Serial.print(getFloat(47), 2); Serial.print(",");
  Serial.print(getU16(39)); Serial.print(",");
  Serial.print(getFloat(51), 5); Serial.print(",");
  Serial.print(getU16(41)); Serial.print(",");
  Serial.print(getFloat(55), 5); Serial.print(",");
  Serial.print(getU16(43)); Serial.print(",");
  Serial.print(getFloat(59), 5); Serial.print(",");
  Serial.print("0x"); Serial.print(flashPage[45], HEX); Serial.print(",");
  Serial.print("0x"); Serial.print(flashPage[46], HEX); Serial.print(",");
  Serial.print("0x"); Serial.print(flashPage[63], HEX); Serial.print(",");
  Serial.print(flashPage[64]); Serial.print(",");
  Serial.print(flashPage[65]); Serial.print(",");
  Serial.print(flashPage[66]); Serial.print(",");
  Serial.println(flashPage[67]);
}


void setup()
{
  Serial.begin(115200);
  while(!Serial) {};

  Serial.println("AS7331 QSPI CSV dump");
  Serial.println("session_id,sequence,timestamp,sample,temp_raw,temp_c,"
                 "uva_raw,uva_uw_cm2,uvb_raw,uvb_uw_cm2,"
                 "uvc_raw,uvc_nominal_uw_cm2,osr,status,flags,"
                 "gain,time,clock,divider");

  uint8_t mid = 0;
  uint16_t did = 0;
  if(!SFLASH.begin() || !SFLASH.identify(mid, did)) {
    Serial.println("# ERROR: QSPI initialization failed");
    SFLASH.end();
    return;
  }

  if(SFLASH.pageSize() != LOG_PAGE_SIZE) {
    Serial.println("# ERROR: unsupported QSPI page size");
    SFLASH.end();
    return;
  }

  uint32_t pageCount = SFLASH.length() / LOG_PAGE_SIZE;
  for(uint32_t page = 0; page < pageCount; page++) {
    if(!SFLASH.read(page * LOG_PAGE_SIZE, flashPage, sizeof(flashPage))) {
      Serial.print("# ERROR: read failed at page "); Serial.println(page);
      break;
    }

    if(pageIsErased()) break;

    // Other logs may precede this one. Skip pages that do not use this format.
    if(flashPage[0] != LOG_MAGIC_0 || flashPage[1] != LOG_MAGIC_1) continue;

    if(!pageIsValid(page)) {
      Serial.print("# INVALID_PAGE,"); Serial.println(page);
      continue;
    }

    if(flashPage[3] == PAGE_TYPE_SESSION) {
      printSessionPage();
    } else if(flashPage[3] == PAGE_TYPE_SAMPLE) {
      printSamplePage();
    }
  }

  SFLASH.end();
  Serial.println("# dump_complete");
}


void loop()
{
}
