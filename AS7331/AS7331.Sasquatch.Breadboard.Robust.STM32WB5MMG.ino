/*
 * AS7331 Sasquatch breadboard test -- QSPI logger checkpoint 4
 *
 * A checked CMD-mode conversion is taken every 10 seconds and stored as one
 * append-only 256-byte QSPI page. READY, timeout, sample, and flash-poll
 * callbacks only set flags and wake the MCU. The main loop services those
 * flags and otherwise keeps the STM32WB in STOP.
 *
 * The logger never erases or wraps flash. Use a separate erase sketch before
 * starting a new empty-flash test when desired.
 */

#include <Arduino.h>
#include <RTC.h>
#include <SFLASH.h>
#include <STM32WB.h>
#include <TimerMillis.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "AS7331.h"

#define AS7331_READY_PIN       8
#define GREEN_LED             22 // Sasquatch RGB LED is active LOW
#define RED_LED               23
#define BLUE_LED              24
#define I2C_BUS               Wire // Sasquatch external I2C bus

const uint32_t AS7331_TIMEOUT_MS = 250;
const uint32_t SAMPLE_INTERVAL_MS = 10000;          // Development sampling interval
const uint32_t THERMAL_RECHECK_INTERVAL_MS = 60000; // Sparse sampling during lockout
const uint32_t FLASH_TIMEOUT_MS = 1000;
const uint32_t FLASH_POLL_INTERVAL_MS = 1;
const uint32_t HEARTBEAT_ON_MS = 1; // Brief indication after a verified log write
const float HIGH_TEMPERATURE_C = 50.0f;
const float RESUME_TEMPERATURE_C = 45.0f;

const uint8_t AS7331_GAIN = 8; // 8x gain; change only between test sessions
const uint8_t AS7331_TIME = 6; // 64 ms at 1.024 MHz; divider not required
const uint8_t AS7331_CLOCK = AS7331_1024_KHZ;
const bool AS7331_DIVIDER_ENABLED = false;
const uint8_t AS7331_DIVIDER = 0;

// Enable for USB Serial bring-up. Disable for battery-powered logging.
bool serialDebug = false;

// QSPI page format
const uint8_t LOG_MAGIC_0 = 'U';
const uint8_t LOG_MAGIC_1 = 'V';
const uint8_t LOG_VERSION = 1;
const uint8_t PAGE_TYPE_SESSION = 1;
const uint8_t PAGE_TYPE_SAMPLE = 2;
const uint8_t LOG_MARKER = 0xA7;
const uint16_t LOG_PAGE_SIZE = 256;
const uint16_t PAYLOAD_OFFSET = 24;
const uint16_t CRC_MSB_BYTE = 253;
const uint16_t CRC_LSB_BYTE = 254;
const uint16_t MARKER_BYTE = 255;

const uint8_t SAMPLE_FLAG_VALID = 0x01;
const uint8_t SAMPLE_FLAG_THERMAL_LOCKOUT = 0x02;
const uint8_t SAMPLE_FLAG_ENTERED_LOCKOUT = 0x04;
const uint8_t SAMPLE_FLAG_RECOVERED = 0x08;

const uint8_t LOG_FORMAT_UUID[16] = {
  'A', 'S', '7', '3', '3', '1', '-', 'U',
  'V', 'L', 'O', 'G', '-', 'V', '0', '1'
};

I2Cdev i2cBus(&I2C_BUS);
AS7331 as7331(&i2cBus);
TimerMillis measurementTimeout;
TimerMillis sampleTimer;
TimerMillis flashPollTimer;
TimerMillis heartbeatTimer;

enum MeasurementState : uint8_t {
  MEASUREMENT_IDLE,
  MEASUREMENT_WAITING,
  MEASUREMENT_FINISHED,
  MEASUREMENT_FAILED
};

enum FlashState : uint8_t {
  FLASH_IDLE,
  FLASH_BEGIN,
  FLASH_WAIT_BEFORE_PROGRAM,
  FLASH_WAIT_AFTER_PROGRAM,
  FLASH_FAULT
};

MeasurementState measurementState = MEASUREMENT_IDLE;
FlashState flashState = FLASH_IDLE;

volatile bool AS7331_readyFlag = false;
volatile bool AS7331_timeoutFlag = false;
volatile bool sampleDue = false;
volatile bool flashPollDue = false;
volatile bool heartbeatOffDue = false;

bool thermalLockout = false;
bool flashInterfaceOpen = false;
bool flashFull = false;
uint8_t flashPage[LOG_PAGE_SIZE];
uint8_t verifyPage[LOG_PAGE_SIZE];
uint8_t flashMID = 0;
uint16_t flashDID = 0;
uint32_t flashPageCount = 0;
uint32_t nextFlashPage = 0;
uint32_t logSequence = 0;
uint32_t sessionID = 0;
uint32_t flashDeadline = 0;
uint32_t measurementCount = 0;


void AS7331_inthandler()
{
  AS7331_readyFlag = true;
  STM32WB.wakeup();
}


void AS7331_timeoutHandler()
{
  AS7331_timeoutFlag = true;
  STM32WB.wakeup();
}


void sampleTimerHandler()
{
  sampleDue = true;
  STM32WB.wakeup();
}


void flashPollHandler()
{
  flashPollDue = true;
  STM32WB.wakeup();
}


void heartbeatTimerHandler()
{
  heartbeatOffDue = true;
  STM32WB.wakeup();
}


void serviceHeartbeat()
{
  if(!heartbeatOffDue) return;

  heartbeatOffDue = false;
  digitalWrite(GREEN_LED, HIGH); // Sasquatch RGB LED is active LOW
}


bool indicateSuccessfulLogWrite()
{
  heartbeatOffDue = false;
  heartbeatTimer.stop();
  digitalWrite(GREEN_LED, LOW);

  if(!heartbeatTimer.start(heartbeatTimerHandler, HEARTBEAT_ON_MS)) {
    digitalWrite(GREEN_LED, HIGH);
    return false;
  }
  return true;
}


void indicateFailure(const char *message)
{
  measurementState = MEASUREMENT_FAILED;
  measurementTimeout.stop();
  sampleTimer.stop();
  flashPollTimer.stop();
  heartbeatTimer.stop();
  detachInterrupt(digitalPinToInterrupt(AS7331_READY_PIN));
  as7331.powerDown();

  if(flashInterfaceOpen) {
    SFLASH.end();
    flashInterfaceOpen = false;
  }

  flashState = FLASH_FAULT;
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(RED_LED, LOW);
  if(serialDebug) Serial.println(message);
}


uint8_t buildMonth()
{
  const char *date = __DATE__;
  if(date[0] == 'J' && date[1] == 'a') return 1;
  if(date[0] == 'F') return 2;
  if(date[0] == 'M' && date[2] == 'r') return 3;
  if(date[0] == 'A' && date[1] == 'p') return 4;
  if(date[0] == 'M' && date[2] == 'y') return 5;
  if(date[0] == 'J' && date[2] == 'n') return 6;
  if(date[0] == 'J' && date[2] == 'l') return 7;
  if(date[0] == 'A' && date[1] == 'u') return 8;
  if(date[0] == 'S') return 9;
  if(date[0] == 'O') return 10;
  if(date[0] == 'N') return 11;
  return 12;
}


void setRTCFromBuildTime()
{
  const char *date = __DATE__;
  const char *time = __TIME__;
  uint8_t day = (date[4] == ' ') ? (date[5] - '0') :
                                   ((date[4] - '0') * 10 + date[5] - '0');
  uint8_t year = (uint8_t)((date[9] - '0') * 10 + date[10] - '0');
  uint8_t hours = (uint8_t)((time[0] - '0') * 10 + time[1] - '0');
  uint8_t minutes = (uint8_t)((time[3] - '0') * 10 + time[4] - '0');
  uint8_t seconds = (uint8_t)((time[6] - '0') * 10 + time[7] - '0');

  RTC.begin();
  RTC.setDateTime(day, buildMonth(), year, hours, minutes, seconds);
}


void clearFlashPage()
{
  memset(flashPage, 0xFF, sizeof(flashPage));
}


void putU16(uint16_t offset, uint16_t value)
{
  flashPage[offset] = (uint8_t)(value >> 8);
  flashPage[offset + 1] = (uint8_t)value;
}


void putI16(uint16_t offset, int16_t value)
{
  putU16(offset, (uint16_t)value);
}


void putU32(uint16_t offset, uint32_t value)
{
  flashPage[offset] = (uint8_t)(value >> 24);
  flashPage[offset + 1] = (uint8_t)(value >> 16);
  flashPage[offset + 2] = (uint8_t)(value >> 8);
  flashPage[offset + 3] = (uint8_t)value;
}


void putFloat(uint16_t offset, float value)
{
  union {
    float floatingPoint;
    uint32_t bits;
  } conversion;

  conversion.floatingPoint = value;
  putU32(offset, conversion.bits);
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


bool pageBufferIsErased()
{
  for(uint16_t index = 0; index < LOG_PAGE_SIZE; index++) {
    if(flashPage[index] != 0xFF) return false;
  }
  return true;
}


void fillPageHeader(uint8_t pageType, uint8_t flags)
{
  flashPage[0] = LOG_MAGIC_0;
  flashPage[1] = LOG_MAGIC_1;
  flashPage[2] = LOG_VERSION;
  flashPage[3] = pageType;
  putU32(4, nextFlashPage);
  putU32(8, logSequence);
  putU32(12, sessionID);
  flashPage[16] = flags;
  flashPage[17] = 0;
  flashPage[18] = 0;
  flashPage[19] = 0;
  flashPage[20] = 0;
  flashPage[21] = 0;
  flashPage[22] = 0;
  flashPage[23] = 0;
}


void finishPreparedPage()
{
  uint16_t crc = calculateCRC16(flashPage, CRC_MSB_BYTE);
  flashPage[CRC_MSB_BYTE] = (uint8_t)(crc >> 8);
  flashPage[CRC_LSB_BYTE] = (uint8_t)crc;
  flashPage[MARKER_BYTE] = LOG_MARKER;
}


bool waitForFlashDuringSetup()
{
  uint32_t start = millis();
  while(SFLASH.busy()) {
    if((millis() - start) >= FLASH_TIMEOUT_MS) return false;
  }
  return SFLASH.status() == SFLASH_STATUS_SUCCESS;
}


bool commitPreparedPageDuringSetup()
{
  if(nextFlashPage >= flashPageCount) return false;
  finishPreparedPage();

  if(SFLASH.busy() && !waitForFlashDuringSetup()) return false;
  uint32_t address = nextFlashPage * LOG_PAGE_SIZE;
  if(!SFLASH.program(address, flashPage, sizeof(flashPage))) return false;
  if(!waitForFlashDuringSetup()) return false;
  if(!SFLASH.read(address, verifyPage, sizeof(verifyPage))) return false;
  if(memcmp(flashPage, verifyPage, sizeof(flashPage)) != 0) return false;

  nextFlashPage++;
  logSequence++;
  return true;
}


void prepareSessionPage()
{
  clearFlashPage();
  fillPageHeader(PAGE_TYPE_SESSION, 0);

  for(uint8_t index = 0; index < sizeof(LOG_FORMAT_UUID); index++) {
    flashPage[PAYLOAD_OFFSET + index] = LOG_FORMAT_UUID[index];
  }

  uint8_t hours, minutes, seconds;
  uint16_t milliseconds;
  RTC.getTime(hours, minutes, seconds, milliseconds);

  putU16(40, (uint16_t)(2000 + RTC.getYear()));
  flashPage[42] = RTC.getMonth();
  flashPage[43] = RTC.getDay();
  flashPage[44] = hours;
  flashPage[45] = minutes;
  flashPage[46] = seconds;
  putU32(47, SAMPLE_INTERVAL_MS);
  putU32(51, THERMAL_RECHECK_INTERVAL_MS);
  flashPage[55] = AS7331_GAIN;
  flashPage[56] = AS7331_TIME;
  flashPage[57] = AS7331_CLOCK;
  flashPage[58] = AS7331_DIVIDER_ENABLED ? 1 : 0;
  flashPage[59] = AS7331_DIVIDER;
  putI16(60, (int16_t)(HIGH_TEMPERATURE_C * 100.0f));
  putI16(62, (int16_t)(RESUME_TEMPERATURE_C * 100.0f));
  flashPage[64] = flashMID;
  putU16(65, flashDID);
  putU16(67, LOG_PAGE_SIZE);
}


bool initializeFlashLog()
{
  if(!SFLASH.begin()) return false;
  flashInterfaceOpen = true;

  bool success = SFLASH.identify(flashMID, flashDID);
  success = success && (SFLASH.pageSize() == LOG_PAGE_SIZE);
  if(success) {
    flashPageCount = SFLASH.length() / LOG_PAGE_SIZE;
    nextFlashPage = 0;

    while(nextFlashPage < flashPageCount) {
      if(!SFLASH.read(nextFlashPage * LOG_PAGE_SIZE,
                      flashPage, sizeof(flashPage))) {
        success = false;
        break;
      }
      if(pageBufferIsErased()) break;
      nextFlashPage++;
    }

    if(nextFlashPage >= flashPageCount) {
      flashFull = true;
      success = false;
    }
  }

  if(success) {
    /*
     * The logger never wraps, so the first erased physical page is also a
     * simple monotonic sequence across resets. This prevents sequence numbers
     * from restarting at zero when a loose battery contact resets the MCU.
     */
    logSequence = nextFlashPage;

    /*
     * Mix the unique append position into the RTC value. RTC is initialized
     * from the build time in this standalone test, so time alone cannot
     * distinguish two closely spaced resets. The physical page can.
     */
    sessionID = RTC.getY2kEpoch() ^
                (nextFlashPage * 2654435761UL);
    prepareSessionPage();
    success = commitPreparedPageDuringSetup();
  }

  SFLASH.end();
  flashInterfaceOpen = false;

  if(serialDebug) {
    Serial.print("QSPI MID = 0x"); Serial.println(flashMID, HEX);
    Serial.print("QSPI DID = 0x"); Serial.println(flashDID, HEX);
    Serial.print("First sample page = "); Serial.println(nextFlashPage);
  }
  return success;
}


void prepareSamplePage(const AS7331Data &data, uint8_t osr, uint8_t status,
                       float temperatureC, float uva, float uvb, float uvc,
                       uint8_t sampleFlags)
{
  clearFlashPage();
  fillPageHeader(PAGE_TYPE_SAMPLE, sampleFlags);

  uint8_t day, month, year, hours, minutes, seconds;
  uint16_t milliseconds;
  RTC.getDateTime(day, month, year, hours, minutes, seconds, milliseconds);

  putU16(24, (uint16_t)(2000 + year));
  flashPage[26] = month;
  flashPage[27] = day;
  flashPage[28] = hours;
  flashPage[29] = minutes;
  flashPage[30] = seconds;
  putU16(31, milliseconds);
  putU32(33, measurementCount);
  putU16(37, data.temperature);
  putU16(39, data.uva);
  putU16(41, data.uvb);
  putU16(43, data.uvc);
  flashPage[45] = osr;
  flashPage[46] = status;
  putFloat(47, temperatureC);
  putFloat(51, uva);
  putFloat(55, uvb);
  putFloat(59, uvc);
  flashPage[63] = sampleFlags;
  flashPage[64] = AS7331_GAIN;
  flashPage[65] = AS7331_TIME;
  flashPage[66] = AS7331_CLOCK;
  flashPage[67] = AS7331_DIVIDER_ENABLED ? AS7331_DIVIDER : 0xFF;
}


bool queuePreparedPage()
{
  if(flashState != FLASH_IDLE || nextFlashPage >= flashPageCount) return false;
  finishPreparedPage();
  flashState = FLASH_BEGIN;
  return true;
}


bool startFlashPoll()
{
  flashPollDue = false;
  flashPollTimer.stop();
  return flashPollTimer.start(flashPollHandler, FLASH_POLL_INTERVAL_MS);
}


bool startFlashProgram()
{
  if(nextFlashPage >= flashPageCount) {
    flashFull = true;
    return false;
  }

  if(!SFLASH.program(nextFlashPage * LOG_PAGE_SIZE,
                     flashPage, sizeof(flashPage))) return false;

  flashDeadline = millis() + FLASH_TIMEOUT_MS;
  flashState = FLASH_WAIT_AFTER_PROGRAM;
  return startFlashPoll();
}


void serviceFlash()
{
  if(flashState == FLASH_IDLE || flashState == FLASH_FAULT) return;

  if(flashState == FLASH_BEGIN) {
    if(!SFLASH.begin()) {
      indicateFailure("ERROR: QSPI begin failed");
      return;
    }

    flashInterfaceOpen = true;
    flashDeadline = millis() + FLASH_TIMEOUT_MS;
    if(SFLASH.busy()) {
      flashState = FLASH_WAIT_BEFORE_PROGRAM;
      if(!startFlashPoll()) indicateFailure("ERROR: QSPI poll timer failed");
    } else if(!startFlashProgram()) {
      indicateFailure(flashFull ? "ERROR: QSPI log is full" :
                                  "ERROR: QSPI program start failed");
    }
    return;
  }

  if(!flashPollDue) return;
  flashPollDue = false;

  if((int32_t)(millis() - flashDeadline) >= 0) {
    indicateFailure("ERROR: QSPI operation timed out");
    return;
  }

  if(SFLASH.busy()) {
    if(!startFlashPoll()) indicateFailure("ERROR: QSPI poll timer failed");
    return;
  }

  if(flashState == FLASH_WAIT_BEFORE_PROGRAM) {
    if(!startFlashProgram()) {
      indicateFailure(flashFull ? "ERROR: QSPI log is full" :
                                  "ERROR: QSPI program start failed");
    }
    return;
  }

  bool success = (SFLASH.status() == SFLASH_STATUS_SUCCESS);
  uint32_t writtenPage = nextFlashPage;
  uint32_t address = writtenPage * LOG_PAGE_SIZE;
  success = success && SFLASH.read(address, verifyPage, sizeof(verifyPage));
  success = success && (memcmp(flashPage, verifyPage, sizeof(flashPage)) == 0);

  SFLASH.end();
  flashInterfaceOpen = false;
  flashPollTimer.stop();

  if(!success) {
    indicateFailure("ERROR: QSPI write verification failed");
    return;
  }

  nextFlashPage++;
  logSequence++;
  flashState = FLASH_IDLE;

  // Pulse green only after the complete page passes the QSPI readback check.
  if(!indicateSuccessfulLogWrite()) {
    indicateFailure("ERROR: heartbeat timer failed");
    return;
  }

  if(serialDebug) {
    Serial.print("Logged QSPI page "); Serial.println(writtenPage);
  }

  if(nextFlashPage >= flashPageCount) {
    flashFull = true;
    indicateFailure("ERROR: QSPI log is full; logging stopped");
  }
}


bool initializeAS7331()
{
  if(!i2cBus.probe(AS7331_ADDRESS)) return false;

  uint8_t chipID = 0;
  if(!as7331.powerUp() || !as7331.getChipID(&chipID)) return false;

  if(serialDebug) {
    Serial.print("AS7331 chip ID = 0x");
    Serial.println(chipID, HEX);
  }
  if(chipID != AS7331_CHIP_ID) return false;

  bool success = as7331.enterConfiguration();
  success = as7331.configure(AS7331_CMD_MODE,
                             (AS7331ConversionClock)AS7331_CLOCK,
                             false, 0, AS7331_GAIN, AS7331_TIME) && success;
  success = as7331.powerDown() && success;
  if(!success || !as7331.healthy()) return false;

  uint8_t operatingState = 0;
  if(!as7331.readOperatingState(&operatingState)) return false;

  if(serialDebug) {
    Serial.print("AS7331 OSR after configuration = 0x");
    Serial.println(operatingState, HEX);
  }
  return ((operatingState & AS7331_OSR_POWER_DOWN) != 0);
}


bool startAS7331Measurement()
{
  AS7331_readyFlag = false;
  AS7331_timeoutFlag = false;

  attachInterrupt(digitalPinToInterrupt(AS7331_READY_PIN),
                  AS7331_inthandler, RISING);

  if(!measurementTimeout.start(AS7331_timeoutHandler, AS7331_TIMEOUT_MS)) {
    detachInterrupt(digitalPinToInterrupt(AS7331_READY_PIN));
    return false;
  }

  if(!as7331.startOneShot()) {
    measurementTimeout.stop();
    detachInterrupt(digitalPinToInterrupt(AS7331_READY_PIN));
    return false;
  }

  measurementState = MEASUREMENT_WAITING;
  return true;
}


void reportStatusErrors(uint8_t status)
{
  if(!serialDebug) return;
  if(status & AS7331_STATUS_OUTCONVOF) Serial.println("AS7331 time-reference overflow");
  if(status & AS7331_STATUS_MRESOF)    Serial.println("AS7331 result-register overflow");
  if(status & AS7331_STATUS_ADCOF)     Serial.println("AS7331 analog-channel overflow");
  if(status & AS7331_STATUS_LDATA)     Serial.println("AS7331 previous result was overwritten");
  if(status & AS7331_STATUS_NOTREADY)  Serial.println("AS7331 still reports measurement in progress");
}


bool updateThermalState(float temperatureC, uint8_t *sampleFlags)
{
  if(!thermalLockout && temperatureC >= HIGH_TEMPERATURE_C) {
    thermalLockout = true;
    *sampleFlags |= SAMPLE_FLAG_THERMAL_LOCKOUT |
                    SAMPLE_FLAG_ENTERED_LOCKOUT;
    return sampleTimer.restart(THERMAL_RECHECK_INTERVAL_MS,
                               THERMAL_RECHECK_INTERVAL_MS);
  }

  if(thermalLockout && temperatureC <= RESUME_TEMPERATURE_C) {
    thermalLockout = false;
    *sampleFlags |= SAMPLE_FLAG_RECOVERED;
    return sampleTimer.restart(SAMPLE_INTERVAL_MS, SAMPLE_INTERVAL_MS);
  }

  if(thermalLockout) *sampleFlags |= SAMPLE_FLAG_THERMAL_LOCKOUT;
  return true;
}


void serviceAS7331Measurement()
{
  if(measurementState != MEASUREMENT_WAITING) return;

  if(AS7331_timeoutFlag) {
    AS7331_timeoutFlag = false;
    indicateFailure("ERROR: AS7331 READY timeout");
    return;
  }
  if(!AS7331_readyFlag) return;

  AS7331_readyFlag = false;
  measurementTimeout.stop();
  detachInterrupt(digitalPinToInterrupt(AS7331_READY_PIN));

  uint16_t combinedStatus = 0;
  if(!as7331.readStatus(&combinedStatus)) {
    indicateFailure("ERROR: AS7331 status read failed");
    return;
  }

  uint8_t osr = (uint8_t)(combinedStatus >> 8);
  uint8_t status = (uint8_t)combinedStatus;
  reportStatusErrors(status);

  if((status & AS7331_STATUS_NDATA) == 0 ||
     (status & (AS7331_STATUS_OUTCONVOF |
                AS7331_STATUS_MRESOF |
                AS7331_STATUS_ADCOF |
                AS7331_STATUS_NOTREADY)) != 0) {
    indicateFailure("ERROR: AS7331 result is not valid");
    return;
  }

  AS7331Data data = {0, 0, 0, 0};
  if(!as7331.readAllData(&data) || !as7331.powerDown()) {
    indicateFailure("ERROR: AS7331 data read or power-down failed");
    return;
  }

  measurementState = MEASUREMENT_FINISHED;
  measurementCount++;

  const float lsbA = 0.60938f;
  const float lsbB = 0.79688f;
  const float lsbC = 0.38282f;
  float temperatureC = (float)data.temperature * 0.05f - 66.9f;
  float uva = (float)data.uva * lsbA;
  float uvb = (float)data.uvb * lsbB;
  float uvc = (float)data.uvc * lsbC;
  uint8_t sampleFlags = SAMPLE_FLAG_VALID;

  if(!updateThermalState(temperatureC, &sampleFlags)) {
    indicateFailure("ERROR: thermal sample timer restart failed");
    return;
  }

  if(flashState != FLASH_IDLE) {
    indicateFailure("ERROR: previous QSPI write still pending");
    return;
  }

  prepareSamplePage(data, osr, status, temperatureC,
                    uva, uvb, uvc, sampleFlags);
  if(!queuePreparedPage()) {
    indicateFailure(flashFull ? "ERROR: QSPI log is full" :
                                "ERROR: could not queue QSPI page");
    return;
  }

  if(serialDebug) {
    Serial.print("Measurement "); Serial.println(measurementCount);
    Serial.print("OSR = 0x"); Serial.print(osr, HEX);
    Serial.print(", STATUS = 0x"); Serial.println(status, HEX);
    Serial.print("UVA = "); Serial.print(data.uva);
    Serial.print(" ("); Serial.print(uva, 5); Serial.println(" uW/cm^2)");
    Serial.print("UVB = "); Serial.print(data.uvb);
    Serial.print(" ("); Serial.print(uvb, 5); Serial.println(" uW/cm^2)");
    Serial.print("UVC channel = "); Serial.print(data.uvc);
    Serial.print(" ("); Serial.print(uvc, 5); Serial.println(" nominal uW/cm^2)");
    Serial.print("Sensor temperature = ");
    Serial.print(temperatureC, 2); Serial.println(" C");
    if(sampleFlags & SAMPLE_FLAG_ENTERED_LOCKOUT) Serial.println("THERMAL LOCKOUT ENTERED");
    if(sampleFlags & SAMPLE_FLAG_RECOVERED) Serial.println("THERMAL LOCKOUT CLEARED");
  }
}


void setup()
{
  if(serialDebug) {
    Serial.begin(115200);
    while(!Serial) {};
  }

  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(BLUE_LED, HIGH);
  pinMode(AS7331_READY_PIN, INPUT); // READY is push-pull; no MCU pull resistor

  I2C_BUS.begin();
  I2C_BUS.setClock(400000);
  setRTCFromBuildTime();

  if(serialDebug) {
    Serial.println("AS7331 Sasquatch QSPI logger");
    Serial.println("Checkpoint 4: 10-second append-only logging");
  }

  if(!initializeAS7331()) {
    indicateFailure("ERROR: AS7331 initialization failed");
    return;
  }

  if(!initializeFlashLog()) {
    indicateFailure(flashFull ? "ERROR: QSPI log is full" :
                                "ERROR: QSPI log initialization failed");
    return;
  }

  if(!sampleTimer.start(sampleTimerHandler,
                        SAMPLE_INTERVAL_MS, SAMPLE_INTERVAL_MS)) {
    indicateFailure("ERROR: could not start AS7331 sample timer");
    return;
  }

  if(!startAS7331Measurement()) {
    indicateFailure("ERROR: could not start AS7331 measurement");
  }
}


void loop()
{
  serviceAS7331Measurement();
  serviceFlash();
  serviceHeartbeat();

  if(sampleDue &&
     measurementState != MEASUREMENT_WAITING &&
     measurementState != MEASUREMENT_FAILED &&
     flashState == FLASH_IDLE) {
    sampleDue = false;
    if(!startAS7331Measurement()) {
      indicateFailure("ERROR: could not start periodic AS7331 measurement");
    }
  }

  // Timer and GPIO callbacks wake the MCU; all otherwise-idle time is STOP.
  STM32WB.stop();
}
