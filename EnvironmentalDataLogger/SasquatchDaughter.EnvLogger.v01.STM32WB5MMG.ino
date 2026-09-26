/*
 * Sasquatch Daughter v01 robust environmental logger
 *
 * One complete environmental record is collected each minute. Three sensors
 * are used in one-shot mode; the remaining low-power sensors are sampled or
 * cached as appropriate. Four 56-byte records share each 256-byte QSPI page.
 * Logging is append-only: it never erases or wraps when the flash becomes full.
 */

#include <Arduino.h>
#include <Wire.h>
#include <RTC.h>
#include <SFLASH.h>
#include <STM32WB.h>
#include <TimerMillis.h>
#include <BLE.h>
#include <EEPROM.h>

#include "I2Cdev.h"
#include "AEM13921.h"
#include "APDS9253.h"
#include "ENS161.h"
#include "HDC2010.h"
#include "LIS2DW12.h"
#include "LPS22DF.h"

// Set true only for tethered Serial Monitor tests. Untethered field logging
// must use false or setup() will wait forever for a USB serial connection.
const bool serialDebug = false;
const bool debugAEM13921RegisterDump = false;

/*
 * SENSOR SELECTION
 *
 * For normal holistic daughter-board tests, keep all sensors enabled. Disabled
 * sensors retain their log fields but are marked invalid.
 */
#define ENABLE_LPS22DF    1
#define ENABLE_HDC2010    1
#define ENABLE_APDS9253   1
#define ENABLE_ENS161     1
#define ENABLE_AEM13921   1
#define ENABLE_BLE_NUS    1
#define ENABLE_WATCHDOG   0
#define CLEAR_HEALTH_ON_NON_WATCHDOG_BOOT 1

#define AEM13921_IRQ_PIN   3
#define ENS161_IRQ_PIN     7
#define LPS22DF_IRQ_PIN    8
#define HDC2010_IRQ_PIN    9
#define APDS9253_IRQ_PIN   A0
#define AEM13921_RUN_PIN   A1

#define GREEN_LED 22
#define RED_LED   23
#define BLUE_LED  24

const uint32_t SAMPLE_INTERVAL_MS       = 60000UL;
const uint32_t ONE_SHOT_CONVERSION_MS   = 40UL;
const uint32_t HEARTBEAT_INTERVAL_MS    = 10000UL;
const uint32_t INDICATOR_INTERVAL_MS    = 5000UL;
const uint32_t FLASH_TIMEOUT_MS         = 1000UL;
const uint32_t FLASH_POLL_INTERVAL_MS   = 2UL;
const uint32_t SERIAL_WAIT_MS           = 5000UL;
const uint32_t AEM_COMMISSION_TIMEOUT_MS = 5000UL;
const uint32_t AEM_SOURCE_PROMPT_MS      = 1000UL;
const uint32_t AEM_PROBE_INTERVAL_MS     = 100UL;
const uint16_t AEM_STARTUP_PROBE_MS      = 250;
const uint16_t AEM_SHIP_SETTLE_MS        = 1000;
const uint32_t BLE_PERIOD_MS             = 120000UL;
const uint32_t BLE_ADVERTISE_MS          = 3000UL;
const uint16_t BLE_RECOVER_MS            = 1000;
const uint32_t BLE_ACTIVE_POLL_MS        = 100UL;
// The installed STM32WB core caps IWDG timeout requests at 32000 ms.
const uint32_t WATCHDOG_TIMEOUT_MS       = 30000UL;

/*
 * AEM13921 CHARGING POLICY
 *
 * When ENABLE_AEM13921 is set, the AEM is always managed by this policy. The
 * AEM costs roughly 300 uA when left active, so the logger treats it as a
 * two-state hardware resource: OFF/ship or ON/active. P0/P4/P5/P6/P7 are log
 * reasons for the same OFF hardware state; P1/P2/P3 are log reasons for the
 * same ON hardware state. State changes are separated by a dwell interval so
 * the analog harvester state can settle and threshold chatter is avoided.
 */
const uint16_t AEM_POLICY_START_VBAT_MV       = 4050;
const uint16_t AEM_POLICY_STOP_VBAT_MV        = 4140;
const uint16_t AEM_POLICY_STORAGE_HIGH_MV     = 4200;
const uint16_t AEM_POLICY_MIN_AMBIENT_LUX     = 50;
const uint16_t AEM_POLICY_KEEP_SRC1_K         = 3000;
const uint8_t AEM_POLICY_MAX_EVALUATE_SAMPLES = 3;
const uint8_t AEM_POLICY_BAD_HARVEST_SAMPLES  = 2;
const uint16_t AEM_POLICY_RETRY_SAMPLES       = 30;
const uint8_t AEM_POLICY_MIN_DWELL_SAMPLES    = 2;

const uint16_t LOW_BATTERY_MV           = 3400;
const float HIGH_TEMPERATURE_C          = 45.0f;
const float APDS_LUX_PER_COUNT          = 2.16f; // value for gain1, divide by any other gain, e.g., for gain6 change to 0.360f
const uint8_t SENSOR_READ_RETRIES        = 3;
const uint8_t MAX_CONSECUTIVE_SAMPLE_FAULTS = 3;

const uint16_t LOG_PAGE_SIZE            = 256;
const uint8_t LOG_HEADER_SIZE            = 32;
const uint8_t LOG_RECORD_SIZE            = 56;
const uint8_t LOG_RECORDS_PER_PAGE       = 4;
const uint8_t LOG_VERSION                = 4;
const uint8_t LOG_MAGIC[4]               = {'S', 'D', 'L', 'G'};
const uint32_t FAULT_MAGIC               = 0x53444654UL; // "SDFT"
const uint8_t FAULT_VERSION              = 1;
const uint16_t FAULT_EEPROM_ADDRESS      = 0;
const uint32_t HEALTH_MAGIC              = 0x5344484CUL; // "SDHL"
const uint8_t HEALTH_VERSION             = 9;
const uint16_t HEALTH_EEPROM_ADDRESS     = 64;
const uint8_t HEALTH_BREADCRUMB_COUNT    = 8;
const uint8_t LOG_UUID[16]               = {
  0x6E, 0x42, 0xA1, 0x79, 0xCE, 0x51, 0x4B, 0xE8,
  0x9B, 0x35, 0x71, 0xCC, 0x5D, 0x01, 0x00, 0x01
};

// The STM32WB linker stores the sketch's UTC build epoch in this eight-byte
// information block. Its layout is defined by the installed STM32WB core.
struct EmbeddedRTCInfo {
  uint32_t epoch;
  uint16_t zone;
  uint8_t dst;
  uint8_t leapSeconds;
};

extern "C" const EmbeddedRTCInfo __rtc_info__;

static_assert(LOG_HEADER_SIZE + LOG_RECORDS_PER_PAGE * LOG_RECORD_SIZE == LOG_PAGE_SIZE,
              "The header and records must fill one QSPI page");

enum SensorMask : uint8_t {
  SENSOR_HDC = 0x01, SENSOR_LPS = 0x02, SENSOR_ENS = 0x04,
  SENSOR_APDS = 0x08, SENSOR_AEM = 0x20,
  SENSOR_LIS = 0x40
};

enum PrimaryStatus : uint8_t {
  STATUS_LOW_BATTERY = 0x01, STATUS_CHARGING = 0x02,
  STATUS_HIGH_TEMPERATURE = 0x04, STATUS_MOTION = 0x08,
  STATUS_LOG_FULL = 0x10, STATUS_I2C_FAULT = 0x20,
  STATUS_FLASH_FAULT = 0x40, STATUS_ENS_VALID = 0x80
};

enum LogQuality : uint8_t {
  QUALITY_PRIMARY_ENV_PARTIAL = 0x01,
  QUALITY_ENS_NOT_VALID       = 0x02,
  QUALITY_BATTERY_INVALID     = 0x04,
  QUALITY_AEM_UNAVAILABLE     = 0x08,
  QUALITY_I2C_WARNING         = 0x10,
  QUALITY_FLASH_WARNING       = 0x20
};

enum FlashState : uint8_t {
  FLASH_IDLE, FLASH_BEGIN, FLASH_WAIT_BEFORE_PROGRAM,
  FLASH_WAIT_AFTER_PROGRAM, FLASH_FAILED
};

enum BLEWindowState : uint8_t {
  BLE_WINDOW_SLEEP, BLE_WINDOW_ADVERTISING, BLE_WINDOW_CONNECTED,
  BLE_WINDOW_RECOVERING
};

enum FaultCode : uint8_t {
  FAULT_NONE = 0,
  FAULT_RTC_INIT = 1,
  FAULT_SENSOR_INIT = 2,
  FAULT_FLASH_INIT = 3,
  FAULT_TIMER_INIT = 4,
  FAULT_SAMPLE_REPEATED = 5,
  FAULT_FLASH_PROGRAM = 6,
  FAULT_WATCHDOG_RESET = 9,
  FAULT_FAULT_RESET = 10,
  FAULT_ASSERT_RESET = 11,
  FAULT_PANIC_RESET = 12
};

enum AEMPolicyState : uint8_t {
  AEM_POLICY_OFF = 0,
  AEM_POLICY_WAKE_REQUESTED = 1,
  AEM_POLICY_EVALUATING = 2,
  AEM_POLICY_CHARGING = 3,
  AEM_POLICY_STOP_STORAGE_HIGH = 4,
  AEM_POLICY_SKIP_LIGHT_LOW = 5,
  AEM_POLICY_STOP_NO_HARVEST = 6,
  AEM_POLICY_UNAVAILABLE = 7
};

enum AEMInitMask : uint8_t {
  AEM_INIT_ADDRESS = 0x01,
  AEM_INIT_IDENTITY = 0x02,
  AEM_INIT_CONFIG = 0x04,
  AEM_INIT_FLAGS = 0x08
};

enum SetupPhase : uint8_t {
  SETUP_PHASE_START = 0,
  SETUP_PHASE_GPIO_READY = 1,
  SETUP_PHASE_I2C_READY = 2,
  SETUP_PHASE_RTC_READY = 3,
  SETUP_PHASE_WATCHDOG_RECOVERY = 4,
  SETUP_PHASE_SENSOR_INIT = 5,
  SETUP_PHASE_SENSOR_SKIPPED = 6,
  SETUP_PHASE_FLASH_INIT = 7,
  SETUP_PHASE_AEM_INIT = 8,
  SETUP_PHASE_TIMERS = 9,
  SETUP_PHASE_LOGGING = 10,
  SETUP_PHASE_INIT_FAILED = 11
};

enum RuntimePhase : uint8_t {
  RUNTIME_PHASE_BOOT = 0,
  RUNTIME_PHASE_IDLE = 1,
  RUNTIME_PHASE_BLE_AIRLOCK = 2,
  RUNTIME_PHASE_SENSOR_IRQ_SERVICE = 3,
  RUNTIME_PHASE_MEASURE_START = 4,
  RUNTIME_PHASE_MEASURE_FINISH = 5,
  RUNTIME_PHASE_AEM_POLICY = 8,
  RUNTIME_PHASE_FLASH_SERVICE = 9,
  RUNTIME_PHASE_WATCHDOG_CHECKPOINT = 10,
  RUNTIME_PHASE_STATUS_INDICATORS = 11,
  RUNTIME_PHASE_STOP = 12
};

enum RuntimeStep : uint8_t {
  RUNTIME_STEP_NONE = 0,
  RUNTIME_STEP_LOOP_TOP = 1,
  RUNTIME_STEP_BLE_SERVICE = 2,
  RUNTIME_STEP_IRQ_ENS = 3,
  RUNTIME_STEP_IRQ_AEM = 4,
  RUNTIME_STEP_MEASURE_START_LPS = 6,
  RUNTIME_STEP_MEASURE_START_HDC = 7,
  RUNTIME_STEP_MEASURE_START_APDS = 8,
  RUNTIME_STEP_HDC_STATUS = 9,
  RUNTIME_STEP_HDC_READ = 10,
  RUNTIME_STEP_ENS_COMP = 11,
  RUNTIME_STEP_LPS_STATUS = 13,
  RUNTIME_STEP_LPS_READ = 14,
  RUNTIME_STEP_APDS_STATUS = 15,
  RUNTIME_STEP_APDS_READ = 16,
  RUNTIME_STEP_APDS_DISABLE = 17,
  RUNTIME_STEP_ENS_STATUS = 18,
  RUNTIME_STEP_ENS_READ = 19,
  RUNTIME_STEP_AEM_CONFIG = 23,
  RUNTIME_STEP_AEM_POLICY = 24,
  RUNTIME_STEP_AEM_IRQ_FLAGS = 25,
  RUNTIME_STEP_AEM_IRQ_STATUS = 26,
  RUNTIME_STEP_AEM_IRQ_APM = 27,
  RUNTIME_STEP_LIS_STATUS = 28,
  RUNTIME_STEP_LIS_READ = 29,
  RUNTIME_STEP_FLASH_APPEND = 30,
  RUNTIME_STEP_FLASH_SERVICE = 31,
  RUNTIME_STEP_WATCHDOG_FEED = 32,
  RUNTIME_STEP_STOP_ENTER = 33,
  RUNTIME_STEP_I2C_RECOVER = 34
};

struct __attribute__((packed)) FaultRecord {
  uint32_t magic;
  uint8_t version;
  uint16_t sequence;
  uint32_t timestamp;
  uint8_t code;
  uint8_t detail0;
  uint8_t detail1;
  uint8_t detail2;
  uint16_t crc;
};

struct __attribute__((packed)) HealthBreadcrumb {
  uint32_t timestamp;
  uint16_t sampleSequence;
  uint8_t runtimeStep;
  uint8_t aemPolicy;
  uint8_t i2cBusState;
  uint8_t status0;
};

struct __attribute__((packed)) SystemHealthRecord {
  uint32_t magic;
  uint8_t version;
  uint32_t bootCount;
  uint16_t watchdogResetCount;
  uint16_t faultResetCount;
  uint16_t assertResetCount;
  uint16_t panicResetCount;
  uint32_t lastBootTimestamp;
  uint32_t lastCheckpointTimestamp;
  uint8_t lastResetCause;
  uint32_t lastWakeupReason;
  uint16_t lastSampleSequence;
  uint8_t lastAEMPolicy;
  uint8_t lastStatus0;
  uint8_t lastSetupPhase;
  uint8_t lastRuntimePhase;
  uint8_t lastRuntimeStep;
  uint8_t i2cBusBeforeRecover;
  uint8_t i2cBusAfterRecover;
  uint8_t i2cBusLastSeen;
  uint8_t breadcrumbWriteIndex;
  uint8_t breadcrumbCount;
  HealthBreadcrumb breadcrumbs[HEALTH_BREADCRUMB_COUNT];
  uint16_t crc;
};

struct SensorSnapshot {
  uint32_t timestamp;
  uint16_t sequence;
  uint8_t status0;
  uint8_t status1;
  uint8_t validMask;
  uint8_t freshMask;
  int16_t hdcTemperature;
  uint16_t hdcHumidity;
  int32_t pressureRaw;
  int16_t pressureTemperature;
  uint8_t ensAQI;
  uint8_t ensValidity;
  uint16_t ensTVOC;
  uint16_t ensECO2;
  uint16_t light[4];
  uint16_t batteryMillivolts;
  uint16_t batteryPercent;
  uint16_t storageMillivolts;
  int16_t aemTemperature;
  uint8_t aemStatus0;
  uint8_t aemStatus1;
  uint8_t source1Raw;
  uint8_t source2Raw;
  bool aemAPMValid;
  uint32_t aemAPMSource1;
  uint32_t aemAPMSource2;
  uint32_t aemAPMLoad;
  uint16_t aemAPM5V;
  uint8_t aemAPMError;
  uint8_t aemPolicyState;
  int16_t acceleration[3];
};

bool readMCUBatteryTelemetry(SensorSnapshot *sample);
uint8_t logQualityForSample(const SensorSnapshot &sample);
void requestAEMRun();
void requestAEMShip();
bool requestAEMShipAndCheck(uint16_t settleMs);
bool configureAEMForPolicy();
void attachAEM13921IRQ();
void detachAEM13921IRQ();
void enableAEMHardware();
void disableAEMHardware(AEMPolicyState reason);
bool aemPolicyShouldReadAEM(AEMPolicyState state);
bool aemPolicyBatteryHigh(const SensorSnapshot &sample);
bool aemPolicyBatteryWantsCharge(const SensorSnapshot &sample);
bool aemPolicyLightAdequate(const SensorSnapshot &sample);
bool aemPolicyUsefulHarvesting(const SensorSnapshot &sample);
void applyAEMPolicyForSample(SensorSnapshot *sample,
                             AEMPolicyState startingState,
                             bool aemConfiguredForSample);
void initializeBLE_NUS();
void settleBLECommand(uint16_t delayMs);
void serviceBLE_NUS();
bool loggerIdleForBLE();
void startBLEAdvertiseWindow();
void startBLEConnectedWindow();
void closeBLEWindow();
void finishBLERecovery();
void serviceBLE_RX();
void sendBLEReport();
bool loggerSafeToStop();
void clearFaultRecord();
void recordFault(uint8_t code, uint8_t detail0, uint8_t detail1, uint8_t detail2);
void updateSystemHealthOnBoot(uint32_t resetCause, uint32_t wakeupReason);
void updateSetupPhase(SetupPhase phase);
void updateRuntimePhase(RuntimePhase phase);
void updateRuntimeStep(RuntimeStep step);
void recordHealthBreadcrumb();
uint8_t readExternalI2CBusState();
void updateExternalI2CBusLastSeen();
void recoverExternalI2C(uint32_t clock = 400000);
void markWatchdogCheckpointDue();
void serviceWatchdogCheckpoint();
void serviceWatchdogDuringBLE();
void enableWatchdog();
bool watchdogSampleFreshEnough();
uint8_t aemInitializationMask(bool addressOK, bool identityOK, bool configOK, bool flagsOK);
void setAEMUnavailable();

I2Cdev externalI2C(&Wire);
I2Cdev internalI2C(&Wire1);
AEM13921 aem13921(&externalI2C);
APDS9253 apds9253(&externalI2C);
ENS161 ens161(&externalI2C);
HDC2010 hdc2010(&externalI2C);
LIS2DW12 lis2dw12(&internalI2C);
LPS22DF lps22df(&externalI2C);
BLEUart SerialBLE(BLE_UART_PROTOCOL_NORDIC);

TimerMillis sampleTimer;
TimerMillis conversionTimer;
TimerMillis heartbeatTimer;
TimerMillis indicatorTimer;
TimerMillis ledOffTimer;
TimerMillis flashPollTimer;
TimerMillis blePeriodTimer;
TimerMillis bleWindowTimer;

volatile bool sampleDue = false;
volatile bool conversionDue = false;
volatile bool heartbeatDue = false;
volatile bool indicatorDue = false;
volatile bool ledOffDue = false;
volatile bool ensInterrupt = false;
volatile bool aemInterrupt = false;
volatile bool flashPollDue = false;
volatile bool blePeriodDue = true;
volatile bool bleWindowExpired = false;

bool measurementPending = false;
bool systemFault = false;
bool flashFull = false;
bool lowBattery = false;
bool charging = false;
bool transientFault = false;
bool heartbeatRequest = false;
bool indicatorRequest = false;
bool flashInterfaceOpen = false;
bool latestSampleValid = false;
bool persistentFaultRecorded = false;
bool watchdogEnabled = false;
bool watchdogFeedDue = false;
bool watchdogRecoveryBoot = false;
uint32_t watchdogLastSampleTimestamp = 0;
uint8_t sensorInitFailureMask = 0;
uint8_t aemInitPassMask = 0;
uint8_t ledPriority = 0;
uint8_t consecutiveSampleFaults = 0;
BLEWindowState bleWindowState = BLE_WINDOW_SLEEP;
AEMPolicyState aemPolicyState = AEM_POLICY_OFF;
bool aemHardwareEnabled = false;
bool aemIRQAttached = false;
uint8_t aemPolicyDwellSamples = 0;
uint16_t aemPolicyRetrySamples = 0;
uint8_t aemPolicyEvaluateSamples = 0;
uint8_t aemPolicyBadHarvestSamples = 0;

ENS161Data cachedENS = {};
bool cachedENSValid = false;
bool cachedENSFresh = false;
AEM13921InterruptFlags cachedAEMFlags = {};
AEM13921Status cachedAEMStatus = {};
AEM13921Measurements cachedAEMMeasurements = {};
AEM13921APMData cachedAEMAPM = {};
bool cachedAEMStatusFresh = false;
bool cachedAEMAPMFresh = false;
SensorSnapshot latestSample = {};
SystemHealthRecord systemHealth = {};

uint8_t logPage[LOG_PAGE_SIZE];
uint8_t verifyPage[LOG_PAGE_SIZE];
uint8_t recordCount = 0;
uint32_t nextFlashPage = 0;
uint32_t flashPageCount = 0;
uint32_t pageSequence = 0;
uint16_t sampleSequence = 0;
uint32_t flashDeadline = 0;
FlashState flashState = FLASH_IDLE;


/* --------------------------------------------------------------------------
 * SETUP
 * -------------------------------------------------------------------------- */

void setup()
{
  /*
   * GPIO commissioning comes first. Several daughter-board interrupt traces
   * are high-impedance unless the MCU biases them, and we have already seen
   * that floating interrupt lines can masquerade as sensor-power problems.
  */
  pinMode(APDS9253_IRQ_PIN, INPUT);       // external 10K pullup
  pinMode(LPS22DF_IRQ_PIN, INPUT);        // sensor internal pulldown enabled/verified

  pinMode(ENS161_IRQ_PIN, INPUT_PULLUP);  // unless configured push-pull and always driven
  pinMode(HDC2010_IRQ_PIN, INPUT_PULLUP); // if interrupt output disabled/high-Z
  pinMode(AEM13921_IRQ_PIN, INPUT);       // logic output; clear flags instead of biasing

  pinMode(AEM13921_RUN_PIN, OUTPUT);
  // Default to AEM ship/off. The AEM commissioning and charge-policy paths
  // explicitly request RUN only when they need to talk to the harvester.
  digitalWrite(AEM13921_RUN_PIN, LOW);

  if(serialDebug) {
    Serial.begin(115200);
    uint32_t serialWaitStart = millis();
    while(!Serial && (uint32_t)(millis() - serialWaitStart) < SERIAL_WAIT_MS) {};
  }

  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  setRGB(true, true, false); // Yellow marks commissioning in progress.

  uint32_t resetCause = STM32WB.resetCause();
  uint32_t wakeupReason = STM32WB.wakeupReason();
  watchdogRecoveryBoot = resetCause == RESET_WATCHDOG;

  // Bring up both I2C buses: Wire is the daughter board, Wire1 is Sasquatch.
  Wire.begin();
  Wire.setClock(400000);
  Wire1.begin();
  Wire1.setClock(400000);
#if defined(WIRE_HAS_CLOCK_LOW_TIMEOUT)
  Wire.setClockLowTimeout(25000);
#endif

#if ENABLE_BLE_NUS
  initializeBLE_NUS();
#endif

  if(serialDebug) Serial.println("\nSasquatch Daughter robust QSPI logger");

  /*
   * Cold commissioning is intentionally strict. A watchdog reboot is treated as
   * a field recovery path: rebuild MCU plumbing, recover the append pointer, and
   * let normal runtime reads prove or recover retained peripheral state. This is
   * the same philosophy as the standby motion logger: do not make a watchdog
   * bite repeat every fragile first-power-up test.
   */
  bool rtcInitialized = initializeRTC();
  if(rtcInitialized) {
    updateSystemHealthOnBoot(resetCause, wakeupReason);
    updateSetupPhase(SETUP_PHASE_RTC_READY);
    if(watchdogRecoveryBoot) {
      updateSetupPhase(SETUP_PHASE_WATCHDOG_RECOVERY);
      enableWatchdog();
    }
  }

  bool sensorsInitialized = false;
  if(watchdogRecoveryBoot && rtcInitialized) {
    sensorsInitialized = true;
    sensorInitFailureMask = 0;
    if(serialDebug) {
      Serial.println("Watchdog recovery: strict sensor commissioning skipped");
    }
    if(readExternalI2CBusState() != 0x03 || !externalI2C.healthy()) {
      recoverExternalI2C(400000);
    }
    updateSetupPhase(SETUP_PHASE_SENSOR_SKIPPED);
  } else {
    updateSetupPhase(SETUP_PHASE_SENSOR_INIT);
    sensorsInitialized = initializeSensors();
  }

  // Do not touch the external log unless RTC and every required sensor have
  // commissioned successfully. A failed deployment must accrue no log writes.
  bool flashInitialized = false;
  if(rtcInitialized && sensorsInitialized) {
    updateSetupPhase(SETUP_PHASE_FLASH_INIT);
    flashInitialized = initializeFlashLog();
  }
  if(serialDebug) {
    Serial.print("  QSPI flash log: ");
    if(!rtcInitialized || !sensorsInitialized) Serial.println("SKIPPED - commissioning failed");
    else if(!flashInitialized) Serial.println("FAIL");
    else if(flashFull) Serial.println("FULL - logging disabled without wraparound");
    else Serial.println("PASS");
  }

  /*
   * AEM13921 commissioning is deliberately last. The dedicated AEM toggle
   * sketch showed that the harvester can be controlled when it is brought up,
   * configured, then immediately returned to ship/off and left alone. Recreate
   * that sequence here after all non-AEM logger setup that touches I2C/QSPI.
   */
  bool aemInitialized = true;
#if ENABLE_AEM13921
  bool aem13921Address_OK = false;
  bool aem13921Identity_OK = false;
  bool aem13921Config_OK = false;
  bool aem13921Flags_OK = false;

  if(rtcInitialized && sensorsInitialized && flashInitialized && !watchdogRecoveryBoot) {
    updateSetupPhase(SETUP_PHASE_AEM_INIT);
    aemInitialized = commissionAEM13921(&aem13921Address_OK,
                                        &aem13921Identity_OK,
                                        &aem13921Config_OK,
                                        &aem13921Flags_OK);
    aemInitPassMask = aemInitializationMask(aem13921Address_OK,
                                            aem13921Identity_OK,
                                            aem13921Config_OK,
                                            aem13921Flags_OK);

    aemPolicyState = AEM_POLICY_OFF;
    aemHardwareEnabled = false;
    aemPolicyDwellSamples = AEM_POLICY_MIN_DWELL_SAMPLES;
    aemPolicyRetrySamples = 0;
    aemPolicyEvaluateSamples = 0;
    aemPolicyBadHarvestSamples = 0;
    requestAEMShip();

    if(serialDebug) {
      Serial.println("Late AEM13921 commissioning:");
      Serial.print("  AEM13921 address ACK: "); Serial.println(aem13921Address_OK ? "PASS" : "FAIL");
      Serial.print("  AEM13921 identity:    "); Serial.println(aem13921Identity_OK ? "PASS" : "FAIL");
      Serial.print("  AEM13921 config:      "); Serial.println(aem13921Config_OK ? "PASS" : "FAIL");
      Serial.print("  AEM13921 clear IRQ:   "); Serial.println(aem13921Flags_OK ? "PASS" : "FAIL");
      Serial.println("  AEM13921 post-commission ship requested");
    }

    if(!aemInitialized) {
      if(aem13921Address_OK) recordFault(FAULT_SENSOR_INIT, SENSOR_AEM, aemInitPassMask, 0);
      setAEMUnavailable();
      if(serialDebug) {
        Serial.println("  AEM13921 unavailable now; logger will continue and retry by policy.");
      }
    }
  } else if(watchdogRecoveryBoot) {
    aemPolicyState = AEM_POLICY_OFF;
    aemHardwareEnabled = false;
    aemPolicyDwellSamples = AEM_POLICY_MIN_DWELL_SAMPLES;
    aemPolicyRetrySamples = 0;
    aemPolicyEvaluateSamples = 0;
    aemPolicyBadHarvestSamples = 0;
    detachAEM13921IRQ();
    digitalWrite(AEM13921_RUN_PIN, LOW);
    aemInterrupt = false;
    cachedAEMStatusFresh = false;
    cachedAEMAPMFresh = false;
    if(serialDebug) {
      Serial.println("Watchdog recovery: AEM recommissioning skipped; RUN held low");
    }
  }
#endif

  bool initialized = rtcInitialized && sensorsInitialized && flashInitialized;
  if(!initialized) {
    systemFault = true;
    if(rtcInitialized) updateSetupPhase(SETUP_PHASE_INIT_FAILED);
    if(!rtcInitialized) recordFault(FAULT_RTC_INIT, 0, 0, 0);
    else if(!sensorsInitialized) recordFault(FAULT_SENSOR_INIT, sensorInitFailureMask,
      aemInitPassMask,
                                             externalI2C.healthy() ? 1 : 0);
    else recordFault(FAULT_FLASH_INIT, flashFull ? 1 : 0, 0, 0);
    setRGB(true, false, false);
    if(serialDebug) Serial.println("INITIALIZATION FAILED - logging not started");
    return;
  }

#if !ENABLE_AEM13921
  /*
   * Disabled AEM means true isolation: do not probe or configure the harvester.
   * Just keep the RUN gate low before any logger timers or interrupts are
   * attached.
   */
  digitalWrite(AEM13921_RUN_PIN, LOW);
  aemInterrupt = false;
  cachedAEMStatusFresh = false;
  cachedAEMAPMFresh = false;
  aemPolicyState = AEM_POLICY_OFF;
  aemHardwareEnabled = false;
  charging = false;
  if(serialDebug) Serial.println("  AEM13921 final disabled ship requested");
#endif

#if ENABLE_ENS161
  attachInterrupt(digitalPinToInterrupt(ENS161_IRQ_PIN), ENS161_inthandler, FALLING);
#endif
  // The periodic timers are the only normal wake sources during logging.
  updateSetupPhase(SETUP_PHASE_TIMERS);
  bool timersOK = sampleTimer.start(sampleTimerHandler,
                                    SAMPLE_INTERVAL_MS, SAMPLE_INTERVAL_MS);
  timersOK &= heartbeatTimer.start(heartbeatTimerHandler,
                                   HEARTBEAT_INTERVAL_MS, HEARTBEAT_INTERVAL_MS);
  timersOK &= indicatorTimer.start(indicatorTimerHandler,
                                   INDICATOR_INTERVAL_MS, INDICATOR_INTERVAL_MS);
  if(!timersOK) {
    systemFault = true;
    recordFault(FAULT_TIMER_INIT, 0, 0, 0);
    setRGB(true, false, false);
    if(serialDebug) Serial.println("TIMER INITIALIZATION FAILED");
    return;
  }

  // A brief green pulse confirms that initialization and log recovery passed.
  if(!persistentFaultRecorded) clearFaultRecord();
  pulseLED(false, true, false, 100, 2);

  // Cold boots start the native independent watchdog only after successful
  // commissioning. Watchdog recovery boots enabled it immediately after the RTC
  // and health record were alive, so setup itself is covered on recovery.
  if(!watchdogRecoveryBoot) enableWatchdog();

  // Take the first sample immediately after setup. The hardware watchdog has a
  // 30 second maximum timeout, so a one-minute wait before sample 0 would cause
  // a watchdog reset before the first valid checkpoint exists.
  sampleDue = true;
  updateSetupPhase(SETUP_PHASE_LOGGING);
  if(serialDebug) Serial.println("ALL REQUIRED DEVICES INITIALIZED - logging started");
}


/* --------------------------------------------------------------------------
 * LOOP
 * -------------------------------------------------------------------------- */

void loop()
{
  /*
   * BLE is observational and opportunistic. It is serviced once per loop and
   * never enters STOP from inside the BLE path. While a BLE window is active,
   * new samples are deferred and the normal safe-stop gate keeps the MCU awake
   * with a short delay rather than rapid STOP/WAKE cycling the radio stack.
   */
#if ENABLE_BLE_NUS
  updateRuntimeStep(RUNTIME_STEP_BLE_SERVICE);
  serviceBLE_NUS();
#endif

  /*
   * First clear any latched sensor events. These handlers are intentionally
   * short; they only refresh cached status/data and clear interrupt sources.
   */
  if(ensInterrupt || aemInterrupt) {
    updateRuntimePhase(RUNTIME_PHASE_SENSOR_IRQ_SERVICE);
  }
#if ENABLE_ENS161
  serviceENS161Interrupt();
#endif
#if ENABLE_AEM13921
  serviceAEM13921Interrupt();
#endif

  /*
   * Start a new one-shot measurement group only when the previous sample has
   * been completed and the flash logger is idle. This keeps sensor conversion
   * and page programming from colliding.
   */
  if(sampleDue && !measurementPending && flashState == FLASH_IDLE
#if ENABLE_BLE_NUS
     && bleWindowState == BLE_WINDOW_SLEEP
#endif
     ) {
    sampleDue = false;
    updateRuntimePhase(RUNTIME_PHASE_MEASURE_START);
    startEnvironmentalMeasurement();
  }

  // Finish the one-shot group after the conversion timer says results are due.
  if(measurementPending && conversionDue) {
    updateRuntimePhase(RUNTIME_PHASE_MEASURE_FINISH);
  }
  finishEnvironmentalMeasurementIfReady();

  // Page programming is a small nonblocking state machine.
  if(flashState != FLASH_IDLE && flashState != FLASH_FAILED) {
    updateRuntimePhase(RUNTIME_PHASE_FLASH_SERVICE);
    updateRuntimeStep(RUNTIME_STEP_FLASH_SERVICE);
  }
  serviceFlashLogger();

  // Feed the hardware watchdog only after a complete, clean logger checkpoint.
  if(watchdogFeedDue || heartbeatDue) {
    updateRuntimePhase(RUNTIME_PHASE_WATCHDOG_CHECKPOINT);
    updateRuntimeStep(RUNTIME_STEP_WATCHDOG_FEED);
  }
  serviceWatchdogCheckpoint();

  // LED indications are also nonblocking, so the loop can always return to STOP.
  serviceStatusIndicators();

  /*
   * STOP is only entered from a quiet logger state. If the external I2C bus is
   * not idle high/high, try to clear it while the MCU is still awake; do not
   * freeze a wedged bus by entering STOP at the bottom of the loop.
   */
  uint8_t externalBusState = readExternalI2CBusState();
  if(externalBusState != 0x03) {
    updateExternalI2CBusLastSeen();
    recoverExternalI2C();
    return;
  }
  if(!loggerSafeToStop()) {
#if ENABLE_BLE_NUS
    if(bleWindowState != BLE_WINDOW_SLEEP) delay(BLE_ACTIVE_POLL_MS);
#endif
    return;
  }

  // GPIO and TimerMillis callbacks wake the MCU; all other quiet time is STOP.
  updateRuntimePhase(RUNTIME_PHASE_STOP);
  updateRuntimeStep(RUNTIME_STEP_STOP_ENTER);
  STM32WB.stop();
}


/* --------------------------------------------------------------------------
 * HELPER FUNCTIONS - interrupt and timer callbacks
 * -------------------------------------------------------------------------- */

void ENS161_inthandler()
{
  ensInterrupt = true;
  STM32WB.wakeup();
}


void AEM13921_inthandler()
{
  aemInterrupt = true;
  STM32WB.wakeup();
}


void sampleTimerHandler()
{
  sampleDue = true;
  STM32WB.wakeup();
}


void conversionTimerHandler()
{
  conversionDue = true;
  STM32WB.wakeup();
}


void heartbeatTimerHandler()
{
  heartbeatDue = true;
  STM32WB.wakeup();
}


void indicatorTimerHandler()
{
  indicatorDue = true;
  STM32WB.wakeup();
}


void ledOffTimerHandler()
{
  ledOffDue = true;
  STM32WB.wakeup();
}


void flashPollHandler()
{
  flashPollDue = true;
  STM32WB.wakeup();
}


void blePeriodTimerHandler()
{
  blePeriodDue = true;
  STM32WB.wakeup();
}


void bleWindowTimerHandler()
{
  bleWindowExpired = true;
  STM32WB.wakeup();
}


/* --------------------------------------------------------------------------
 * HELPER FUNCTIONS - RTC commissioning
 * -------------------------------------------------------------------------- */

bool initializeRTC()
{
  const uint32_t minimumUnixEpoch = 946684800UL; // 2000-01-01 00:00:00 UTC.
  uint32_t buildEpoch = __rtc_info__.epoch;
  uint32_t retainedEpoch = RTC.getEpoch();

  // Build time is a safe lower bound: real time cannot precede compilation.
  // Advance a stale retained RTC once, but never rewind a clock that has
  // legitimately continued beyond this sketch's build time.
  bool rtcOK = buildEpoch >= minimumUnixEpoch;
  bool advanced = false;
  if(rtcOK && retainedEpoch < buildEpoch) {
    rtcOK = RTC.setEpoch(buildEpoch);
    advanced = rtcOK;
  }

  if(serialDebug) {
    Serial.print("RTC retained Unix epoch: "); Serial.println(retainedEpoch);
    Serial.print("RTC sketch-build epoch: "); Serial.println(buildEpoch);
    if(!rtcOK) Serial.println("RTC commissioning: FAIL");
    else if(advanced) Serial.println("RTC commissioning: ADVANCED TO SKETCH BUILD TIME");
    else Serial.println("RTC commissioning: RETAINED EXISTING TIME");
  }

  return rtcOK;
}


/* --------------------------------------------------------------------------
 * HELPER FUNCTIONS - sensor initialization
 * -------------------------------------------------------------------------- */

bool commissionAEM13921(bool *addressOK,
                        bool *identityOK,
                        bool *configOK,
                        bool *flagsOK)
{
  if(!addressOK || !identityOK || !configOK || !flagsOK) return false;
  *addressOK = false;
  *identityOK = false;
  *configOK = false;
  *flagsOK = false;

  uint32_t startTime = millis();
  uint32_t nextProbeTime = startTime;
  bool sourcePromptPrinted = false;
  bool firstProbe = true;
  bool firstProbeACK = false;

  requestAEMRun();

  while((millis() - startTime) < AEM_COMMISSION_TIMEOUT_MS) {
    uint32_t now = millis();
    if((int32_t)(now - nextProbeTime) >= 0) {
      nextProbeTime = now + AEM_PROBE_INTERVAL_MS;

      *addressOK = externalI2C.probe(AEM13921_ADDRESS);
      if(firstProbe) {
        firstProbeACK = *addressOK;
        firstProbe = false;
      }

      *identityOK = *addressOK && aem13921.begin();

      *configOK = *identityOK && aem13921.configureStorageAPMMode();

      *flagsOK = *configOK && aem13921.readInterruptFlags(&cachedAEMFlags);

      if(*addressOK && *identityOK && *configOK && *flagsOK) {
        if(serialDebug) {
          Serial.print("  AEM13921 startup path: ");
          Serial.println(firstProbeACK ? "already responsive" :
                                            "became responsive after startup source applied");

          bool configuredByI2C = false;
          if(aem13921.configuredByI2C(&configuredByI2C)) {
            Serial.print("  AEM13921 configuration source: ");
            Serial.println(configuredByI2C ? "I2C registers" : "hardware pins");
          }

          AEM13921Status startupStatus = {};
          if(aem13921.readStatus(&startupStatus)) {
            Serial.print("  AEM13921 5 V source at commissioning: ");
            Serial.println((startupStatus.status0 & AEM13921_STATUS_5V_CONNECTED) ?
                           "PRESENT" : "ABSENT");
          }

          printAEM13921RegisterDump();
        }
        return true;
      }
    }

    if(!sourcePromptPrinted && (millis() - startTime) >= AEM_SOURCE_PROMPT_MS) {
      sourcePromptPrinted = true;
      if(serialDebug) {
        Serial.println("AEM13921 is not yet I2C-responsive.");
        Serial.println("  Logger will continue if startup source remains unavailable.");
      }
    }

    delay(10); // Setup-only pacing; loop() remains entirely nonblocking.
  }

  requestAEMShip();
  if(serialDebug) Serial.println("AEM13921 commissioning timed out; charging policy will retry later.");
  return false;
}


bool initializeSensors()
{
  sensorInitFailureMask = 0;
  aemInitPassMask = 0;

  if(serialDebug) Serial.println("Sensor initialization:");

  uint8_t id8 = 0;
  uint16_t id16 = 0;

  bool lis2dw12_OK = lis2dw12.getChipID(&id8) && id8 == 0x44;
  lis2dw12_OK = lis2dw12.reset() && lis2dw12_OK;
  // Run continuously at the LIS2DW12's lowest ODR; stationary mode prevents
  // the sleep state from reverting to the otherwise fixed 12.5 Hz ODR.
  lis2dw12_OK = lis2dw12.initMeasurement(LIS2DW12_FS_2G, LIS2DW12_ODR_12_5_1_6Hz,
                                         LIS2DW12_MODE_LOW_POWER, LIS2DW12_LP_MODE_2,
                                         LIS2DW12_BW_FILT_ODR2, false, true) && lis2dw12_OK;
  if(serialDebug) {
    Serial.print("  LIS2DW12:  ");
    Serial.println(lis2dw12_OK ? "PASS" : "FAIL");
  }

  if(serialDebug) {
    Serial.println("  Battery telemetry: STM32WB.readBattery()");
  }

  bool lps22df_OK = true;
#if ENABLE_LPS22DF
  lps22df_OK = lps22df.getChipID(&id8) && id8 == 0xB4;
  lps22df_OK = lps22df.reset() && lps22df_OK;
  lps22df_OK = lps22df.Init(P_1shot, avg_128, lpf_odr4, false) && lps22df_OK;
  lps22df_OK = lps22df.powerDown() && lps22df_OK;
#else
  lps22df_OK = lps22df.getChipID(&id8) && id8 == 0xB4 && lps22df.powerDown();
#endif
  if(serialDebug) {
    Serial.print("  LPS22DF:   ");
    Serial.println(!ENABLE_LPS22DF ? (lps22df_OK ? "DISABLED" : "FAIL") : (lps22df_OK ? "PASS" : "FAIL"));
  }

  bool hdc2010_OK = true;
#if ENABLE_HDC2010
  hdc2010_OK = hdc2010.getDevID(HDC2010_0_ADDRESS, &id16) && id16 == 0x07D0;
  hdc2010_OK = hdc2010.reset(HDC2010_0_ADDRESS) && hdc2010_OK;
  hdc2010_OK = hdc2010.init(HDC2010_0_ADDRESS, HRES_14bit, TRES_14bit, ForceMode) && hdc2010_OK;
#else
  hdc2010_OK = hdc2010.getDevID(HDC2010_0_ADDRESS, &id16) &&
               id16 == 0x07D0 &&
               hdc2010.idle(HDC2010_0_ADDRESS);
#endif
  if(serialDebug) {
    Serial.print("  HDC2010:   ");
    Serial.println(!ENABLE_HDC2010 ? (hdc2010_OK ? "DISABLED" : "FAIL") : (hdc2010_OK ? "PASS" : "FAIL"));
  }

  bool apds9253_OK = true;
#if ENABLE_APDS9253
  apds9253_OK = apds9253.getChipID(&id8) && ((id8 & 0xF0) == 0xC0);
  apds9253_OK = apds9253.reset() && apds9253_OK;
  if(apds9253_OK) delay(10); // Setup-only reset completion; loop remains nonblocking.
  apds9253_OK = apds9253.init(RGBiR, res16bit, rate40Hz, gain1) && apds9253_OK;
  apds9253_OK = apds9253.disable() && apds9253_OK;
#else
  apds9253_OK = apds9253.getChipID(&id8) && ((id8 & 0xF0) == 0xC0) &&
                apds9253.disable();
#endif
  if(serialDebug) {
    Serial.print("  APDS9253:  ");
    Serial.println(!ENABLE_APDS9253 ? (apds9253_OK ? "DISABLED" : "FAIL") : (apds9253_OK ? "PASS" : "FAIL"));
  }

  bool ens161_OK = true;
#if ENABLE_ENS161
  ens161_OK = ens161.startInitialization(ENS161_ULTRA_LOW_POWER_MODE);
#else
  // Use the complete state machine even when disabling the sensor. A bare
  // register write can acknowledge while leaving the ENS161 in its ~2 mA IDLE
  // state; completion below verifies that OPMODE actually becomes DEEP SLEEP.
  ens161_OK = ens161.startInitialization(ENS161_DEEP_SLEEP_MODE);
#endif
  uint32_t deadline = millis() + 10000UL;
  while(ens161_OK && !ens161.initializationComplete() && !ens161.initializationFailed() &&
         (int32_t)(millis() - deadline) < 0) ens161.serviceInitialization();
  ens161_OK = ens161_OK && ens161.initializationComplete();
  if(serialDebug) {
    Serial.print("  ENS161:    ");
    Serial.println(!ENABLE_ENS161 ? (ens161_OK ? "DEEP SLEEP" : "FAIL") : (ens161_OK ? "PASS" : "FAIL"));
  }

  // A true ship/reset start needs source energy; an MCU-only reset normally
  // finds the AEM13921 already responsive. Both paths are bounded and verified
  // only when the AEM is intentionally enabled.
  bool aem13921_OK = true;
#if ENABLE_AEM13921
  aem13921_OK = true;
#else
  // A disabled AEM is a no-touch isolation state: hold RUN low and do not probe
  // or configure the harvester. It is intentionally unlike a normal sensor.
  digitalWrite(AEM13921_RUN_PIN, LOW);
  aemInterrupt = false;
  cachedAEMStatusFresh = false;
  cachedAEMAPMFresh = false;
  aem13921_OK = true;
#endif
  if(serialDebug) {
    Serial.print("  AEM13921:  ");
    Serial.println(ENABLE_AEM13921 ? "DEFERRED TO END OF SETUP" : "DISABLED - RUN LOW");
  }

  bool allSensorsOK = lis2dw12_OK && lps22df_OK &&
                      hdc2010_OK && apds9253_OK && ens161_OK && aem13921_OK;

  if(!lis2dw12_OK) sensorInitFailureMask |= SENSOR_LIS;
  if(!lps22df_OK) sensorInitFailureMask |= SENSOR_LPS;
  if(!hdc2010_OK) sensorInitFailureMask |= SENSOR_HDC;
  if(!apds9253_OK) sensorInitFailureMask |= SENSOR_APDS;
  if(!ens161_OK) sensorInitFailureMask |= SENSOR_ENS;
  if(!aem13921_OK) sensorInitFailureMask |= SENSOR_AEM;

  return allSensorsOK;
}


/* --------------------------------------------------------------------------
 * HELPER FUNCTIONS - sensor event service and sample collection
 * -------------------------------------------------------------------------- */

void serviceENS161Interrupt()
{
  if(!ensInterrupt) return;
  ensInterrupt = false;
  updateRuntimeStep(RUNTIME_STEP_IRQ_ENS);
  bool valid = false;
  if(ens161.readData(&cachedENS, &valid)) {
    cachedENSValid = valid;
    cachedENSFresh = true;
  }
}


void serviceAEM13921Interrupt()
{
  if(!aemInterrupt) return;
  aemInterrupt = false;
  updateRuntimeStep(RUNTIME_STEP_IRQ_AEM);

  // Read flags first; this clears latched events and releases IRQ when no
  // enabled event remains. If APM finished, read APM before the next APM
  // window can overwrite the result.
  updateRuntimeStep(RUNTIME_STEP_AEM_IRQ_FLAGS);
  if(!aem13921.readInterruptFlags(&cachedAEMFlags)) return;

  updateRuntimeStep(RUNTIME_STEP_AEM_IRQ_STATUS);
  cachedAEMStatusFresh = aem13921.readStatus(&cachedAEMStatus) &&
                         aem13921.readMeasurements(&cachedAEMMeasurements);

  if(cachedAEMFlags.flags1 & AEM13921_IRQ_APM_DONE) {
    updateRuntimeStep(RUNTIME_STEP_AEM_IRQ_APM);
    cachedAEMAPMFresh = aem13921.readAPMData(&cachedAEMAPM);
  }
}


void startEnvironmentalMeasurement()
{
  bool started = true;
#if ENABLE_LPS22DF
  updateRuntimeStep(RUNTIME_STEP_MEASURE_START_LPS);
  started = lps22df.oneShot();
#endif
#if ENABLE_HDC2010
  updateRuntimeStep(RUNTIME_STEP_MEASURE_START_HDC);
  started = hdc2010.triggerMeasurement(HDC2010_0_ADDRESS) && started;
#endif
#if ENABLE_APDS9253
  updateRuntimeStep(RUNTIME_STEP_MEASURE_START_APDS);
  started = apds9253.enable() && started;
#endif

  if(!started) {
    systemFault = true;
#if ENABLE_APDS9253
    apds9253.disable();
#endif
    if(serialDebug) Serial.println("One-shot conversion start failed");
    return;
  }

  measurementPending = true;

#if ENABLE_LPS22DF || ENABLE_HDC2010 || ENABLE_APDS9253
  if(!conversionTimer.start(conversionTimerHandler, ONE_SHOT_CONVERSION_MS)) {
    measurementPending = false;
    systemFault = true;
#if ENABLE_APDS9253
    apds9253.disable();
#endif
    if(serialDebug) Serial.println("One-shot conversion timer failed");
  }
#else
  // No conversion is pending; collect the always-on LIS2DW12 and log now.
  conversionDue = true;
#endif
}


void finishEnvironmentalMeasurementIfReady()
{
  if(!measurementPending || !conversionDue) return;
  conversionDue = false;
  measurementPending = false;

  SensorSnapshot sample = {};
  sample.timestamp = RTC.getY2kEpoch();
  sample.sequence = sampleSequence++;

#if ENABLE_AEM13921
  AEMPolicyState startingAEMPolicyState = aemPolicyState;
  bool aemConfiguredForSample = false;
  bool readAEMThisSample = false;

  readAEMThisSample = aemPolicyShouldReadAEM(startingAEMPolicyState);
  if(startingAEMPolicyState == AEM_POLICY_WAKE_REQUESTED) {
    updateRuntimeStep(RUNTIME_STEP_AEM_CONFIG);
    aemConfiguredForSample = configureAEMForPolicy();
    readAEMThisSample = aemConfiguredForSample;
  }
#endif

#if ENABLE_HDC2010
  float temperature = 0.0f;
  float humidity = 0.0f;
  uint8_t hdcStatus = 0;
  updateRuntimeStep(RUNTIME_STEP_HDC_STATUS);
  bool hdcReady = hdc2010.getIntStatus(HDC2010_0_ADDRESS, &hdcStatus) &&
                  (hdcStatus & 0x80) != 0; // DRDY_STATUS: new T/RH result available.
  if(hdcReady) {
    updateRuntimeStep(RUNTIME_STEP_HDC_READ);
  }
  if(hdcReady && hdc2010.readData(HDC2010_0_ADDRESS, &temperature, &humidity)) {
    sample.validMask |= SENSOR_HDC;
    sample.freshMask |= SENSOR_HDC;
    sample.hdcTemperature = (int16_t)(temperature * 100.0f);
    sample.hdcHumidity = (uint16_t)(humidity * 100.0f);
#if ENABLE_ENS161
    updateRuntimeStep(RUNTIME_STEP_ENS_COMP);
    ens161.writeCompensation(temperature, humidity);
#endif
    if(temperature >= HIGH_TEMPERATURE_C) sample.status0 |= STATUS_HIGH_TEMPERATURE;
  }
#endif

#if ENABLE_LPS22DF
  uint8_t lpsStatus = 0;
  uint8_t lpsStatusAfterRead = 0;
  bool lpsIdleBeforeRead = false;
  bool lpsIdleAfterRead = false;

  // ST specifies that ONESHOT self-clears and the sensor returns to
  // power-down before the new data are reported ready.
  updateRuntimeStep(RUNTIME_STEP_LPS_STATUS);
  bool lpsOK = lps22df.isIdle(&lpsIdleBeforeRead) && lpsIdleBeforeRead;
  lpsOK = lps22df.status(&lpsStatus) && (lpsStatus & 0x03) == 0x03 && lpsOK;
  updateRuntimeStep(RUNTIME_STEP_LPS_READ);
  lpsOK = lps22df.readSample(&sample.pressureRaw,
                             &sample.pressureTemperature) && lpsOK;
  updateRuntimeStep(RUNTIME_STEP_LPS_STATUS);
  lpsOK = lps22df.status(&lpsStatusAfterRead) &&
          (lpsStatusAfterRead & 0x03) == 0 && lpsOK;
  lpsOK = lps22df.isIdle(&lpsIdleAfterRead) && lpsIdleAfterRead && lpsOK;

  if(lpsOK) {
    sample.validMask |= SENSOR_LPS;
    sample.freshMask |= SENSOR_LPS;
  } else {
    // Treat a single LPS22DF one-shot miss as a bad sample, not a fatal
    // deployment. Repeated misses are promoted to systemFault below.
    if(serialDebug) {
      Serial.print("LPS22DF one-shot state fault: beforeIdle=");
      Serial.print(lpsIdleBeforeRead);
      Serial.print(", status=0x"); Serial.print(lpsStatus, HEX);
      Serial.print(", statusAfter=0x"); Serial.print(lpsStatusAfterRead, HEX);
      Serial.print(", afterIdle="); Serial.println(lpsIdleAfterRead);
    }
  }
#endif

#if ENABLE_APDS9253
  uint32_t light[4] = {0, 0, 0, 0};
  uint8_t apdsStatus = 0;
  updateRuntimeStep(RUNTIME_STEP_APDS_STATUS);
  bool apdsReady = apds9253.getStatus(&apdsStatus) &&
                   (apdsStatus & 0x08) != 0; // LS_DATA_STATUS: unread conversion available.
  if(apdsReady) {
    updateRuntimeStep(RUNTIME_STEP_APDS_READ);
  }
  if(apdsReady && apds9253.getRGBiRdata(light)) {
    for(uint8_t channel = 0; channel < 4; channel++) {
      sample.light[channel] = light[channel] > 65535UL ? 65535 : (uint16_t)light[channel];
    }
    sample.validMask |= SENSOR_APDS;
    sample.freshMask |= SENSOR_APDS;
  }
  updateRuntimeStep(RUNTIME_STEP_APDS_DISABLE);
  apds9253.disable();
#endif

#if ENABLE_ENS161
  bool ensReady = false;
  updateRuntimeStep(RUNTIME_STEP_ENS_STATUS);
  if(!cachedENSFresh && ens161.dataReady(&ensReady) && ensReady) {
    bool valid = false;
    updateRuntimeStep(RUNTIME_STEP_ENS_READ);
    if(ens161.readData(&cachedENS, &valid)) {
      cachedENSValid = valid;
      cachedENSFresh = true;
    }
  }
  if(cachedENSFresh || cachedENS.status != 0) {
    sample.validMask |= SENSOR_ENS;
    if(cachedENSFresh) sample.freshMask |= SENSOR_ENS;
    sample.ensAQI = cachedENS.aqiUBA;
    sample.ensValidity = (uint8_t)cachedENS.validity;
    sample.ensTVOC = cachedENS.tvoc;
    sample.ensECO2 = cachedENS.eco2;
    if(cachedENSValid) sample.status0 |= STATUS_ENS_VALID;
  }
  cachedENSFresh = false;
#endif

  if(readMCUBatteryTelemetry(&sample)) {
    lowBattery = sample.batteryMillivolts <= LOW_BATTERY_MV;
    if(lowBattery) sample.status0 |= STATUS_LOW_BATTERY;
  } else {
    lowBattery = false;
  }
#if ENABLE_AEM13921
  sample.aemPolicyState = aemPolicyState;
  if(readAEMThisSample && cachedAEMStatusFresh) {
    sample.validMask |= SENSOR_AEM;
    sample.freshMask |= SENSOR_AEM;
    sample.storageMillivolts = (uint16_t)(cachedAEMMeasurements.storageVoltage * 1000.0f);
    sample.aemTemperature = cachedAEMMeasurements.temperatureValid ?
                            (int16_t)(cachedAEMMeasurements.temperatureC * 10.0f) : INT16_MIN;
    sample.aemStatus0 = cachedAEMStatus.status0;
    sample.aemStatus1 = cachedAEMStatus.status1;
    sample.source1Raw = cachedAEMMeasurements.source1Raw;
    sample.source2Raw = cachedAEMMeasurements.source2Raw;

    /*
     * APM is the first charging indicator that should mean "useful energy was
     * measured", not merely "a source voltage is present". Keep it diagnostic
     * for now: expose it in Serial/BLE output, log SRC1/1000, and use it
     * to set STATUS_CHARGING.
     */
    if(cachedAEMAPMFresh) {
      sample.aemAPMValid = true;
      sample.aemAPMSource1 = cachedAEMAPM.source1;
      sample.aemAPMSource2 = cachedAEMAPM.source2;
      sample.aemAPMLoad = cachedAEMAPM.load;
      sample.aemAPM5V = cachedAEMAPM.charge5V;
      sample.aemAPMError = cachedAEMAPM.error;
    }

    bool aemMeasuredCharge = sample.aemAPMValid &&
                             ((sample.aemAPMSource1 / 1000UL) >= AEM_POLICY_KEEP_SRC1_K ||
                              sample.aemAPM5V > 0);
    charging = (((cachedAEMStatus.status0 & AEM13921_STATUS_5V_CONNECTED) != 0) ||
                aemMeasuredCharge) &&
               sample.storageMillivolts < AEM_POLICY_STORAGE_HIGH_MV &&
               cachedAEMStatus.status1 == 0;
    if(charging) sample.status0 |= STATUS_CHARGING;
  }
  cachedAEMStatusFresh = false;
  cachedAEMAPMFresh = false;
  updateRuntimePhase(RUNTIME_PHASE_AEM_POLICY);
  updateRuntimeStep(RUNTIME_STEP_AEM_POLICY);
  applyAEMPolicyForSample(&sample, startingAEMPolicyState, aemConfiguredForSample);
#else
  charging = false;
  sample.aemPolicyState = AEM_POLICY_OFF;
#endif

  uint8_t lisStatus = 0;
  updateRuntimeStep(RUNTIME_STEP_LIS_STATUS);
  bool lisOK = lis2dw12.getStatus(&lisStatus);
  updateRuntimeStep(RUNTIME_STEP_LIS_READ);
  if(lisOK && lis2dw12.readAccelData(sample.acceleration)) {
    sample.validMask |= SENSOR_LIS;
    sample.freshMask |= SENSOR_LIS;
  }

  uint8_t required = SENSOR_LIS;
#if ENABLE_LPS22DF
  required |= SENSOR_LPS;
#endif
#if ENABLE_HDC2010
  required |= SENSOR_HDC;
#endif
#if ENABLE_APDS9253
  required |= SENSOR_APDS;
#endif
  bool sampleFault = (sample.validMask & required) != required;
  if(sampleFault) {
    sample.status0 |= STATUS_I2C_FAULT;
    transientFault = true;
    if(consecutiveSampleFaults < 255) consecutiveSampleFaults++;
    if(consecutiveSampleFaults >= MAX_CONSECUTIVE_SAMPLE_FAULTS) {
      systemFault = true;
      recordFault(FAULT_SAMPLE_REPEATED, sample.validMask, required, consecutiveSampleFaults);
    }
    if(!externalI2C.healthy()) recoverExternalI2C(400000);
    if(!internalI2C.healthy()) internalI2C.recover(400000);
  } else {
    consecutiveSampleFaults = 0;
  }
  if(flashFull) sample.status0 |= STATUS_LOG_FULL;
  if(flashState == FLASH_FAILED) sample.status0 |= STATUS_FLASH_FAULT;

  latestSample = sample;
  latestSampleValid = true;

  updateRuntimeStep(RUNTIME_STEP_FLASH_APPEND);
  appendLogRecord(sample);
  printSample(sample);
  markWatchdogCheckpointDue();
}


bool readMCUBatteryTelemetry(SensorSnapshot *sample)
{
  if(sample == nullptr) return false;

  float battery = STM32WB.readBattery();
  if(!(battery > 0.5f && battery < 6.5f)) return false;

  sample->batteryMillivolts = (uint16_t)(battery * 1000.0f + 0.5f);
  sample->batteryPercent = 0;
  return true;
}


uint8_t logQualityForSample(const SensorSnapshot &sample)
{
  uint8_t quality = 0;
  uint8_t primary = SENSOR_LIS;
#if ENABLE_HDC2010
  primary |= SENSOR_HDC;
#endif
#if ENABLE_LPS22DF
  primary |= SENSOR_LPS;
#endif
#if ENABLE_APDS9253
  primary |= SENSOR_APDS;
#endif
  if((sample.validMask & primary) != primary) quality |= QUALITY_PRIMARY_ENV_PARTIAL;

#if ENABLE_ENS161
  if((sample.validMask & SENSOR_ENS) == 0 || (sample.status0 & STATUS_ENS_VALID) == 0) {
    quality |= QUALITY_ENS_NOT_VALID;
  }
#endif

  if(sample.batteryMillivolts == 0) quality |= QUALITY_BATTERY_INVALID;

#if ENABLE_AEM13921
  bool aemExpected = aemPolicyShouldReadAEM((AEMPolicyState)sample.aemPolicyState);
  if(sample.aemPolicyState == AEM_POLICY_UNAVAILABLE ||
     (aemExpected && (sample.validMask & SENSOR_AEM) == 0)) {
    quality |= QUALITY_AEM_UNAVAILABLE;
  }
#endif

  if(sample.status0 & STATUS_I2C_FAULT) quality |= QUALITY_I2C_WARNING;
  if(sample.status0 & (STATUS_LOG_FULL | STATUS_FLASH_FAULT)) quality |= QUALITY_FLASH_WARNING;
  return quality;
}


void requestAEMRun()
{
  digitalWrite(AEM13921_RUN_PIN, HIGH);
}


void requestAEMShip()
{
  /*
   * If the AEM is still responsive, first apply the empirical force-disable
   * path. Then release RUN so the hardware ship circuit can remove the AEM
   * from the I2C bus.
   */
  detachAEM13921IRQ();

  if(externalI2C.probe(AEM13921_ADDRESS)) {
    aem13921.forceDisable();
  }

  digitalWrite(AEM13921_RUN_PIN, LOW);
  aemInterrupt = false;
  cachedAEMStatusFresh = false;
  cachedAEMAPMFresh = false;
}


bool requestAEMShipAndCheck(uint16_t settleMs)
{
  /*
   * Setup-only verification helper. Ship mode removes the AEM from I2C, so
   * "success" here is no ACK after the RUN gate has been released.
   */
  requestAEMShip();
  delay(settleMs);
  return !externalI2C.probe(AEM13921_ADDRESS);
}


bool configureAEMForPolicy()
{
  if(!externalI2C.probe(AEM13921_ADDRESS)) return false;
  if(!aem13921.begin()) return false;

  if(!aem13921.configureStorageAPMMode()) return false;

  // Clear stale flags after each wake/configure attempt so the next APMDONE
  // interrupt represents the current policy trial.
  return aem13921.readInterruptFlags(&cachedAEMFlags);
}


void attachAEM13921IRQ()
{
  if(aemIRQAttached) return;
  aemInterrupt = false;
  attachInterrupt(digitalPinToInterrupt(AEM13921_IRQ_PIN), AEM13921_inthandler, RISING);
  aemIRQAttached = true;
}


void detachAEM13921IRQ()
{
  if(!aemIRQAttached) return;
  detachInterrupt(digitalPinToInterrupt(AEM13921_IRQ_PIN));
  aemIRQAttached = false;
  aemInterrupt = false;
}


void enableAEMHardware()
{
  if(aemHardwareEnabled) return;

  requestAEMRun();
  attachAEM13921IRQ();
  aemHardwareEnabled = true;
  aemPolicyDwellSamples = AEM_POLICY_MIN_DWELL_SAMPLES;
  aemPolicyEvaluateSamples = 0;
  aemPolicyBadHarvestSamples = 0;
  aemPolicyState = AEM_POLICY_WAKE_REQUESTED;
}


void disableAEMHardware(AEMPolicyState reason)
{
  requestAEMShip();
  aemHardwareEnabled = false;
  aemPolicyDwellSamples = AEM_POLICY_MIN_DWELL_SAMPLES;
  aemPolicyEvaluateSamples = 0;
  aemPolicyBadHarvestSamples = 0;
  aemPolicyState = reason;
  charging = false;
}


void setAEMUnavailable()
{
  requestAEMShip();
  aemHardwareEnabled = false;
  aemPolicyState = AEM_POLICY_UNAVAILABLE;
  aemPolicyDwellSamples = AEM_POLICY_MIN_DWELL_SAMPLES;
  aemPolicyRetrySamples = AEM_POLICY_RETRY_SAMPLES;
  aemPolicyEvaluateSamples = 0;
  aemPolicyBadHarvestSamples = 0;
  charging = false;
}


bool aemPolicyShouldReadAEM(AEMPolicyState state)
{
  return state == AEM_POLICY_WAKE_REQUESTED ||
         state == AEM_POLICY_EVALUATING ||
         state == AEM_POLICY_CHARGING;
}


bool aemPolicyBatteryHigh(const SensorSnapshot &sample)
{
  bool storageHigh = sample.storageMillivolts >= AEM_POLICY_STORAGE_HIGH_MV;
  return sample.batteryMillivolts >= AEM_POLICY_STOP_VBAT_MV || storageHigh;
}


bool aemPolicyBatteryWantsCharge(const SensorSnapshot &sample)
{
  if(sample.batteryMillivolts == 0) return false;
  return sample.batteryMillivolts < AEM_POLICY_START_VBAT_MV;
}


bool aemPolicyLightAdequate(const SensorSnapshot &sample)
{
  if((sample.validMask & SENSOR_APDS) == 0) return false;
  float ambientLux = sample.light[1] * APDS_LUX_PER_COUNT;
  return ambientLux >= AEM_POLICY_MIN_AMBIENT_LUX;
}


bool aemPolicyUsefulHarvesting(const SensorSnapshot &sample)
{
  if((sample.validMask & SENSOR_AEM) == 0) return false;
  if(sample.aemStatus1 != 0) return false;
  if(sample.storageMillivolts >= AEM_POLICY_STORAGE_HIGH_MV) return false;

  bool source1Useful = sample.aemAPMValid &&
                       (sample.aemAPMSource1 / 1000UL) >= AEM_POLICY_KEEP_SRC1_K;
  bool fiveVUseful = sample.aemAPMValid && sample.aemAPM5V > 0;
  bool fiveVPresent = (sample.aemStatus0 & AEM13921_STATUS_5V_CONNECTED) != 0;

  return source1Useful || fiveVUseful || fiveVPresent;
}


void applyAEMPolicyForSample(SensorSnapshot *sample,
                             AEMPolicyState startingState,
                             bool aemConfiguredForSample)
{
  if(sample == nullptr) return;

  bool batteryHigh = aemPolicyBatteryHigh(*sample);
  bool batteryWantsCharge = aemPolicyBatteryWantsCharge(*sample);
  bool lightAdequate = aemPolicyLightAdequate(*sample);
  bool aemReadable = (sample->validMask & SENSOR_AEM) != 0;
  bool usefulHarvesting = aemPolicyUsefulHarvesting(*sample);

  if(aemPolicyDwellSamples > 0) aemPolicyDwellSamples--;
  if(aemPolicyRetrySamples > 0) aemPolicyRetrySamples--;

  /*
   * Hardware policy is deliberately two-state:
   *   OFF: P0/P4/P5/P6/P7 are only the logged reason for being off.
   *   ON:  P1/P2/P3 are only the logged reason/phase for being on.
   *
   * Every OFF transition uses the same force-disable + RUN-low path proven by
   * the AEMPolicyTest sketch. Every ON transition uses RUN-high and then
   * configure/read on the following sample.
   */
  if(aemHardwareEnabled) {
    if(batteryHigh) {
      disableAEMHardware(AEM_POLICY_STOP_STORAGE_HIGH);
    } else if(!batteryWantsCharge) {
      disableAEMHardware(AEM_POLICY_OFF);
    } else if(startingState == AEM_POLICY_WAKE_REQUESTED) {
      if(aemConfiguredForSample) {
        aemPolicyState = AEM_POLICY_EVALUATING;
        aemPolicyEvaluateSamples = 1;
        aemPolicyBadHarvestSamples = 0;
      } else {
        aemPolicyRetrySamples = AEM_POLICY_RETRY_SAMPLES;
        disableAEMHardware(AEM_POLICY_UNAVAILABLE);
      }
    } else if(!aemReadable) {
      aemPolicyRetrySamples = AEM_POLICY_RETRY_SAMPLES;
      disableAEMHardware(AEM_POLICY_UNAVAILABLE);
    } else if(aemPolicyDwellSamples > 0) {
      /*
       * Once the harvester is enabled, keep it enabled for a small,
       * deliberate evaluation window. This makes the power trace readable
       * and avoids twitching between ON and OFF due to momentary light or
       * APM variations.
       */
      if(usefulHarvesting) {
        aemPolicyState = AEM_POLICY_CHARGING;
        aemPolicyBadHarvestSamples = 0;
      } else {
        aemPolicyState = AEM_POLICY_EVALUATING;
      }

      if(aemPolicyEvaluateSamples < 255) aemPolicyEvaluateSamples++;
    } else if(!lightAdequate) {
      disableAEMHardware(AEM_POLICY_SKIP_LIGHT_LOW);
    } else if(usefulHarvesting) {
      aemPolicyState = AEM_POLICY_CHARGING;
      aemPolicyBadHarvestSamples = 0;
      if(aemPolicyEvaluateSamples < 255) aemPolicyEvaluateSamples++;
    } else {
      if(aemPolicyEvaluateSamples < 255) aemPolicyEvaluateSamples++;
      if(aemPolicyBadHarvestSamples < 255) aemPolicyBadHarvestSamples++;

      if(aemPolicyEvaluateSamples >= AEM_POLICY_MAX_EVALUATE_SAMPLES ||
         aemPolicyBadHarvestSamples >= AEM_POLICY_BAD_HARVEST_SAMPLES) {
        aemPolicyRetrySamples = AEM_POLICY_RETRY_SAMPLES;
        disableAEMHardware(AEM_POLICY_STOP_NO_HARVEST);
      } else {
        aemPolicyState = AEM_POLICY_EVALUATING;
      }
    }
  } else {
    AEMPolicyState offReason = AEM_POLICY_OFF;
    if(batteryHigh) offReason = AEM_POLICY_STOP_STORAGE_HIGH;
    else if(batteryWantsCharge && !lightAdequate) offReason = AEM_POLICY_SKIP_LIGHT_LOW;
    else if(startingState == AEM_POLICY_STOP_NO_HARVEST && aemPolicyRetrySamples > 0) {
      offReason = AEM_POLICY_STOP_NO_HARVEST;
    } else if(startingState == AEM_POLICY_UNAVAILABLE && aemPolicyRetrySamples > 0) {
      offReason = AEM_POLICY_UNAVAILABLE;
    }

    aemPolicyState = offReason;
    charging = false;

    if(batteryWantsCharge && lightAdequate &&
       aemPolicyRetrySamples == 0 &&
       aemPolicyDwellSamples == 0) {
      enableAEMHardware();
    }
  }

  sample->aemPolicyState = aemPolicyState;
  charging = ((sample->status0 & STATUS_CHARGING) != 0) &&
             aemPolicyState == AEM_POLICY_CHARGING;
}


/* --------------------------------------------------------------------------
 * HELPER FUNCTIONS - nonblocking RGB status indication
 * -------------------------------------------------------------------------- */

void setRGB(bool red, bool green, bool blue)
{
  digitalWrite(RED_LED, red ? LOW : HIGH);
  digitalWrite(GREEN_LED, green ? LOW : HIGH);
  digitalWrite(BLUE_LED, blue ? LOW : HIGH);
}


void pulseLED(bool red, bool green, bool blue, uint32_t duration, uint8_t priority)
{
  if(priority < ledPriority) return;
  ledPriority = priority;
  setRGB(red, green, blue);
  ledOffTimer.stop();
  if(!ledOffTimer.start(ledOffTimerHandler, duration)) {
    setRGB(false, false, false);
    ledPriority = 0;
  }
}


void serviceStatusIndicators()
{
  if(ledOffDue) {
    ledOffDue = false;
    setRGB(false, false, false);
    ledPriority = 0;
  }

  if(heartbeatDue) {
    heartbeatDue = false;
    heartbeatRequest = true;
  }
  if(indicatorDue) {
    indicatorDue = false;
    indicatorRequest = true;
  }

  // Red is fatal/low-battery, magenta is LC telemetry lost, green is AEM active,
  // and blue is the ordinary logger heartbeat.
  if(indicatorRequest) {
    indicatorRequest = false;
    if(systemFault || flashFull || lowBattery) pulseLED(true, false, false, 100, 3);
    else if(transientFault) {
      transientFault = false;
      pulseLED(true, false, false, 25, 2);
    }
    else if(charging || aemHardwareEnabled) pulseLED(false, true, false, 1, 2);
  }
  if(heartbeatRequest) {
    heartbeatRequest = false;
    pulseLED(false, false, true, 10, 1);
  }
}


/* --------------------------------------------------------------------------
 * HELPER FUNCTIONS - QSPI log construction and append recovery
 * -------------------------------------------------------------------------- */

void putU16(uint8_t *destination, uint16_t value)
{
  destination[0] = (uint8_t)value;
  destination[1] = (uint8_t)(value >> 8);
}


void putI16(uint8_t *destination, int16_t value)
{
  putU16(destination, (uint16_t)value);
}


void putU24(uint8_t *destination, int32_t value)
{
  destination[0] = (uint8_t)value;
  destination[1] = (uint8_t)(value >> 8);
  destination[2] = (uint8_t)(value >> 16);
}


void putU32(uint8_t *destination, uint32_t value)
{
  destination[0] = (uint8_t)value;
  destination[1] = (uint8_t)(value >> 8);
  destination[2] = (uint8_t)(value >> 16);
  destination[3] = (uint8_t)(value >> 24);
}


uint16_t getU16(const uint8_t *source)
{
  return (uint16_t)source[0] | ((uint16_t)source[1] << 8);
}


uint32_t getU32(const uint8_t *source)
{
  return (uint32_t)source[0] | ((uint32_t)source[1] << 8) |
         ((uint32_t)source[2] << 16) | ((uint32_t)source[3] << 24);
}


uint16_t calculateCRC16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFF;
  while(length--) {
    crc ^= (uint16_t)(*data++) << 8;
    for(uint8_t bit = 0; bit < 8; bit++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}


/* --------------------------------------------------------------------------
 * HELPER FUNCTIONS - persistent EEPROM fault breadcrumb
 * -------------------------------------------------------------------------- */

bool faultRecordValid(const FaultRecord &record)
{
  if(record.magic != FAULT_MAGIC || record.version != FAULT_VERSION) return false;
  return record.crc == calculateCRC16((const uint8_t *)&record,
                                      sizeof(FaultRecord) - sizeof(record.crc));
}


void clearFaultRecord()
{
  FaultRecord record = {};
  EEPROM.put(FAULT_EEPROM_ADDRESS, record);
  persistentFaultRecorded = false;
}


void recordFault(uint8_t code, uint8_t detail0, uint8_t detail1, uint8_t detail2)
{
  if(persistentFaultRecorded) return;
  persistentFaultRecorded = true;

  FaultRecord previous = {};
  EEPROM.get(FAULT_EEPROM_ADDRESS, previous);

  FaultRecord record = {};
  record.magic = FAULT_MAGIC;
  record.version = FAULT_VERSION;
  record.sequence = faultRecordValid(previous) ? (uint16_t)(previous.sequence + 1) : 1;
  record.timestamp = RTC.getY2kEpoch();
  record.code = code;
  record.detail0 = detail0;
  record.detail1 = detail1;
  record.detail2 = detail2;
  record.crc = calculateCRC16((const uint8_t *)&record,
                              sizeof(FaultRecord) - sizeof(record.crc));

  EEPROM.put(FAULT_EEPROM_ADDRESS, record);
}


bool systemHealthRecordValid(const SystemHealthRecord &record)
{
  if(record.magic != HEALTH_MAGIC || record.version != HEALTH_VERSION) return false;
  return record.crc == calculateCRC16((const uint8_t *)&record,
                                      sizeof(SystemHealthRecord) - sizeof(record.crc));
}


void writeSystemHealthRecord()
{
  systemHealth.magic = HEALTH_MAGIC;
  systemHealth.version = HEALTH_VERSION;
  systemHealth.crc = calculateCRC16((const uint8_t *)&systemHealth,
                                    sizeof(SystemHealthRecord) - sizeof(systemHealth.crc));
  EEPROM.put(HEALTH_EEPROM_ADDRESS, systemHealth);
}


void updateSystemHealthOnBoot(uint32_t resetCause, uint32_t wakeupReason)
{
  SystemHealthRecord previous = {};
  EEPROM.get(HEALTH_EEPROM_ADDRESS, previous);

  bool startFresh = true;
  if(systemHealthRecordValid(previous)) {
#if CLEAR_HEALTH_ON_NON_WATCHDOG_BOOT
    startFresh = resetCause != RESET_WATCHDOG;
#else
    startFresh = false;
#endif
  }

  if(!startFresh) {
    systemHealth = previous;
  } else {
    memset(&systemHealth, 0, sizeof(systemHealth));
    systemHealth.magic = HEALTH_MAGIC;
    systemHealth.version = HEALTH_VERSION;
  }

  systemHealth.bootCount++;
  systemHealth.lastBootTimestamp = RTC.getY2kEpoch();
  systemHealth.lastResetCause = (uint8_t)resetCause;
  systemHealth.lastWakeupReason = wakeupReason;
  systemHealth.lastSetupPhase = SETUP_PHASE_START;
  systemHealth.lastRuntimePhase = RUNTIME_PHASE_BOOT;
  systemHealth.lastRuntimeStep = RUNTIME_STEP_NONE;
  systemHealth.i2cBusBeforeRecover = 0xFF;
  systemHealth.i2cBusAfterRecover = 0xFF;
  systemHealth.i2cBusLastSeen = readExternalI2CBusState();

  if(resetCause == RESET_WATCHDOG) {
    systemHealth.watchdogResetCount++;
  } else if(resetCause == RESET_FAULT) {
    systemHealth.faultResetCount++;
  } else if(resetCause == RESET_ASSERT) {
    systemHealth.assertResetCount++;
  } else if(resetCause == RESET_PANIC) {
    systemHealth.panicResetCount++;
  }

  writeSystemHealthRecord();

  if(serialDebug) {
    Serial.print("Reset cause: ");
    Serial.print(resetCause);
    Serial.print(", wakeup reason: 0x");
    Serial.println(wakeupReason, HEX);
  }
}


void updateSetupPhase(SetupPhase phase)
{
  if(systemHealth.magic != HEALTH_MAGIC || systemHealth.version != HEALTH_VERSION) return;
  if(systemHealth.lastSetupPhase == (uint8_t)phase) return;
  systemHealth.lastSetupPhase = (uint8_t)phase;
  writeSystemHealthRecord();
}


void updateRuntimePhase(RuntimePhase phase)
{
  if(systemHealth.magic != HEALTH_MAGIC || systemHealth.version != HEALTH_VERSION) return;
  if(systemHealth.lastRuntimePhase == (uint8_t)phase) return;
  systemHealth.lastRuntimePhase = (uint8_t)phase;
  writeSystemHealthRecord();
}


void updateRuntimeStep(RuntimeStep step)
{
  if(systemHealth.magic != HEALTH_MAGIC || systemHealth.version != HEALTH_VERSION) return;
  if(systemHealth.lastRuntimeStep == (uint8_t)step) return;
  systemHealth.lastRuntimeStep = (uint8_t)step;
  recordHealthBreadcrumb();
  writeSystemHealthRecord();
}


void recordHealthBreadcrumb()
{
  if(systemHealth.magic != HEALTH_MAGIC || systemHealth.version != HEALTH_VERSION) return;

  uint8_t index = systemHealth.breadcrumbWriteIndex % HEALTH_BREADCRUMB_COUNT;
  HealthBreadcrumb &crumb = systemHealth.breadcrumbs[index];
  crumb.timestamp = RTC.getY2kEpoch();
  crumb.sampleSequence = latestSampleValid ? latestSample.sequence : sampleSequence;
  crumb.runtimeStep = systemHealth.lastRuntimeStep;
  crumb.aemPolicy = aemPolicyState;
  crumb.i2cBusState = readExternalI2CBusState();
  crumb.status0 = latestSampleValid ? latestSample.status0 : 0;

  systemHealth.breadcrumbWriteIndex = (index + 1) % HEALTH_BREADCRUMB_COUNT;
  if(systemHealth.breadcrumbCount < HEALTH_BREADCRUMB_COUNT) systemHealth.breadcrumbCount++;
}

uint8_t readExternalI2CBusState()
{
  /*
   * Compact external Wire snapshot: bit0=SDA high, bit1=SCL high.
   * 0 means both low/stuck-or-driven, 3 means normal idle high/high.
   */
  uint8_t state = 0;
  if(digitalRead(SDA)) state |= 0x01;
  if(digitalRead(SCL)) state |= 0x02;
  return state;
}


void updateExternalI2CBusLastSeen()
{
  if(systemHealth.magic != HEALTH_MAGIC || systemHealth.version != HEALTH_VERSION) return;
  uint8_t state = readExternalI2CBusState();
  if(systemHealth.i2cBusLastSeen == state) return;
  systemHealth.i2cBusLastSeen = state;
  writeSystemHealthRecord();
}


void recoverExternalI2C(uint32_t clock)
{
  updateRuntimeStep(RUNTIME_STEP_I2C_RECOVER);
  uint8_t beforeState = readExternalI2CBusState();
  if(systemHealth.magic == HEALTH_MAGIC && systemHealth.version == HEALTH_VERSION) {
    systemHealth.i2cBusBeforeRecover = beforeState;
    writeSystemHealthRecord();
  }

  /*
   * Be conservative. If SDA/SCL already read idle high/high, do not pulse or
   * reset the external bus. Clear only the software health latch and let the
   * next scheduled device transaction prove whether the fault was transient.
   */
  if(beforeState == 0x03) {
    externalI2C.clearHealthFault();
  } else {
    externalI2C.recover(clock);
  }

  if(systemHealth.magic == HEALTH_MAGIC && systemHealth.version == HEALTH_VERSION) {
    systemHealth.i2cBusAfterRecover = readExternalI2CBusState();
    systemHealth.i2cBusLastSeen = systemHealth.i2cBusAfterRecover;
    writeSystemHealthRecord();
  }
}


void markWatchdogCheckpointDue()
{
#if ENABLE_WATCHDOG
  watchdogFeedDue = true;
#endif
}


void serviceWatchdogCheckpoint()
{
#if ENABLE_WATCHDOG
  if(!watchdogEnabled || systemFault) return;

  /*
   * With a one-minute sample cadence and a 30 second maximum IWDG timeout, the
   * watchdog also needs heartbeat feeds between samples. Do not feed it while a
   * sample, conversion, flash write, or BLE window is pending; those are the
   * states we want the watchdog to recover from if they wedge. Also stop
   * feeding if sample progress gets stale, so a dead sample timer cannot be
   * disguised as healthy idle.
   */
  if(sampleDue || measurementPending || flashState != FLASH_IDLE) return;

  bool sampleCheckpoint = watchdogFeedDue && latestSampleValid;
  bool heartbeatCheckpoint = heartbeatDue && latestSampleValid && watchdogSampleFreshEnough();
  if(!sampleCheckpoint && !heartbeatCheckpoint) return;

  if(sampleCheckpoint) {
    systemHealth.lastSampleSequence = latestSample.sequence;
    systemHealth.lastCheckpointTimestamp = latestSample.timestamp;
    systemHealth.lastAEMPolicy = latestSample.aemPolicyState;
    systemHealth.lastStatus0 = latestSample.status0;
    writeSystemHealthRecord();
    watchdogLastSampleTimestamp = latestSample.timestamp;
    watchdogFeedDue = false;
  }

  STM32WB.wdtReset();
#endif
}


void serviceWatchdogDuringBLE()
{
#if ENABLE_WATCHDOG
  /*
   * BLE service windows are intentional pauses in normal logger work. Keep the
   * IWDG satisfied while the radio stack is advertising/connected so a slow
   * phone/app disconnect does not look like a firmware hang. True hard-locks
   * still bite because this helper only runs while the main loop is alive.
   */
  if(watchdogEnabled && !systemFault) STM32WB.wdtReset();
#endif
}


void enableWatchdog()
{
#if ENABLE_WATCHDOG
  watchdogLastSampleTimestamp = RTC.getY2kEpoch();
  STM32WB.wdtEnable(WATCHDOG_TIMEOUT_MS);
  STM32WB.wdtReset();
  watchdogEnabled = true;
  watchdogFeedDue = false;
  if(serialDebug) {
    Serial.print("Watchdog enabled: ");
    Serial.print(WATCHDOG_TIMEOUT_MS / 1000UL);
    Serial.println(" sec timeout");
  }
#endif
}


bool watchdogSampleFreshEnough()
{
  uint32_t maxSampleAgeSeconds = (SAMPLE_INTERVAL_MS / 1000UL) * 2UL;
  if(maxSampleAgeSeconds < 90UL) maxSampleAgeSeconds = 90UL;
  return (uint32_t)(RTC.getY2kEpoch() - watchdogLastSampleTimestamp) <= maxSampleAgeSeconds;
}


uint8_t aemInitializationMask(bool addressOK, bool identityOK, bool configOK, bool flagsOK)
{
  uint8_t mask = 0;
  if(addressOK) mask |= AEM_INIT_ADDRESS;
  if(identityOK) mask |= AEM_INIT_IDENTITY;
  if(configOK) mask |= AEM_INIT_CONFIG;
  if(flagsOK) mask |= AEM_INIT_FLAGS;
  return mask;
}


uint32_t calculateCRC32(const uint8_t *data, uint16_t length)
{
  uint32_t crc = 0xFFFFFFFFUL;
  while(length--) {
    crc ^= *data++;
    for(uint8_t bit = 0; bit < 8; bit++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320UL : crc >> 1;
  }
  return ~crc;
}


bool pageIsErased(const uint8_t *page)
{
  for(uint16_t index = 0; index < LOG_PAGE_SIZE; index++) if(page[index] != 0xFF) return false;
  return true;
}


bool pageIsValid(const uint8_t *page, uint32_t expectedPageSequence)
{
  if(memcmp(page, LOG_MAGIC, sizeof(LOG_MAGIC)) != 0 ||
     page[4] != LOG_VERSION ||
     page[5] != LOG_RECORDS_PER_PAGE ||
     page[6] != LOG_RECORD_SIZE ||
     page[7] != LOG_HEADER_SIZE ||
     getU32(&page[8]) != expectedPageSequence ||
     memcmp(&page[12], LOG_UUID, sizeof(LOG_UUID)) != 0) return false;

  uint32_t stored = getU32(&page[28]);
  uint8_t copy[LOG_PAGE_SIZE];
  memcpy(copy, page, sizeof(copy));
  memset(&copy[28], 0, 4);
  if(stored != calculateCRC32(copy, sizeof(copy))) return false;

  // The page CRC protects the complete page. Individual record CRCs and
  // sequence checks make append recovery reject a malformed terminal record.
  uint16_t previousSequence = 0;
  uint32_t previousTimestamp = 0;
  for(uint8_t index = 0; index < LOG_RECORDS_PER_PAGE; index++) {
    const uint8_t *record = &page[LOG_HEADER_SIZE + index * LOG_RECORD_SIZE];
    uint16_t sequence = getU16(&record[4]);
    uint32_t timestamp = getU32(&record[0]);
    if(getU16(&record[54]) != calculateCRC16(record, 54)) return false;
    if(index && sequence != (uint16_t)(previousSequence + 1)) return false;
    if(index && timestamp < previousTimestamp) return false;
    previousSequence = sequence;
    previousTimestamp = timestamp;
  }

  return true;
}


void initializeLogPage()
{
  memset(logPage, 0xFF, sizeof(logPage));
  memcpy(&logPage[0], LOG_MAGIC, sizeof(LOG_MAGIC));
  logPage[4] = LOG_VERSION;
  logPage[5] = 0;
  logPage[6] = LOG_RECORD_SIZE;
  logPage[7] = LOG_HEADER_SIZE;
  putU32(&logPage[8], pageSequence);
  memcpy(&logPage[12], LOG_UUID, sizeof(LOG_UUID));
  memset(&logPage[28], 0, 4);
  recordCount = 0;
}


bool initializeFlashLog()
{
  if(!SFLASH.begin()) return false;
  flashInterfaceOpen = true;
  uint8_t mid = 0;
  uint16_t did = 0;
  bool ok = SFLASH.identify(mid, did) && SFLASH.pageSize() == LOG_PAGE_SIZE;
  if(ok) {
    flashPageCount = SFLASH.length() / LOG_PAGE_SIZE;
    uint32_t low = 0;
    uint32_t high = flashPageCount;
    while(low < high) {
      uint32_t middle = low + (high - low) / 2;
      if(!SFLASH.read(middle * LOG_PAGE_SIZE, verifyPage, sizeof(verifyPage))) { ok = false; break; }
      if(pageIsErased(verifyPage)) high = middle;
      else low = middle + 1;
    }
    nextFlashPage = low;
    pageSequence = nextFlashPage;

    if(ok && nextFlashPage > 0) {
      ok = SFLASH.read((nextFlashPage - 1) * LOG_PAGE_SIZE, verifyPage, sizeof(verifyPage));
      ok = ok && pageIsValid(verifyPage, nextFlashPage - 1);

      if(ok) {
        const uint8_t *lastRecord = &verifyPage[LOG_HEADER_SIZE +
                                    (LOG_RECORDS_PER_PAGE - 1) * LOG_RECORD_SIZE];
        uint32_t lastTimestamp = getU32(&lastRecord[0]);
        sampleSequence = (uint16_t)(getU16(&lastRecord[4]) + 1);

        // The STM32WB backup-domain RTC is retained across MCU resets. Never
        // rewrite it from compile time; reject an unexpected time reversal.
        if(RTC.getY2kEpoch() < lastTimestamp) {
          if(serialDebug) Serial.println("RTC continuity check failed during log recovery");
          ok = false;
        }
        else if(serialDebug) {
          Serial.print("  Recovered next flash page: "); Serial.println(nextFlashPage);
          Serial.print("  Recovered next sample sequence: "); Serial.println(sampleSequence);
        }
      }
    }

    if(ok && nextFlashPage < flashPageCount) {
      ok = SFLASH.read(nextFlashPage * LOG_PAGE_SIZE, verifyPage, sizeof(verifyPage));
      ok = ok && pageIsErased(verifyPage);
    }
    else if(ok) {
      // A valid log occupying every page is a normal terminal condition, not
      // flash corruption. Keep sensing but prohibit all further flash writes.
      flashFull = true;
    }
  }
  SFLASH.end();
  flashInterfaceOpen = false;
  if(ok && !flashFull) initializeLogPage();
  return ok;
}


void appendLogRecord(const SensorSnapshot &sample)
{
  if(flashFull || flashState != FLASH_IDLE || recordCount >= LOG_RECORDS_PER_PAGE) return;
  uint8_t *record = &logPage[LOG_HEADER_SIZE + recordCount * LOG_RECORD_SIZE];
  memset(record, 0, LOG_RECORD_SIZE);
  putU32(&record[0], sample.timestamp);
  putU16(&record[4], sample.sequence);
  record[6] = logQualityForSample(sample);
  putI16(&record[7], sample.hdcTemperature);
  putU16(&record[9], sample.hdcHumidity);
  putU24(&record[11], sample.pressureRaw);
  putI16(&record[14], sample.pressureTemperature);
  record[16] = sample.ensAQI;
  putU16(&record[17], sample.ensTVOC);
  putU16(&record[19], sample.ensECO2);
  for(uint8_t channel = 0; channel < 4; channel++) putU16(&record[21 + channel * 2], sample.light[channel]);
  putU16(&record[29], sample.batteryMillivolts);
  putU16(&record[31], sample.storageMillivolts);
  putI16(&record[33], sample.aemTemperature);
  putU16(&record[35], sample.aemAPMValid ? (uint16_t)min(sample.aemAPMSource1 / 1000UL, 65535UL) : 0);
  for(uint8_t axis = 0; axis < 3; axis++) putI16(&record[37 + axis * 2], sample.acceleration[axis]);
  record[43] = sample.aemPolicyState;
  putU16(&record[54], calculateCRC16(record, 54));
  recordCount++;
  logPage[5] = recordCount;

  if(recordCount == LOG_RECORDS_PER_PAGE) {
    memset(&logPage[28], 0, 4);
    putU32(&logPage[28], calculateCRC32(logPage, sizeof(logPage)));
    flashState = FLASH_BEGIN;
  }
}


bool startFlashPoll()
{
  flashPollDue = false;
  flashPollTimer.stop();
  return flashPollTimer.start(flashPollHandler, FLASH_POLL_INTERVAL_MS);
}


bool startFlashProgram()
{
  if(nextFlashPage >= flashPageCount) { flashFull = true; return false; }
  if(!SFLASH.program(nextFlashPage * LOG_PAGE_SIZE, logPage, sizeof(logPage))) return false;
  flashDeadline = millis() + FLASH_TIMEOUT_MS;
  flashState = FLASH_WAIT_AFTER_PROGRAM;
  return startFlashPoll();
}


void failFlash()
{
  flashState = FLASH_FAILED;
  systemFault = true;
  recordFault(FAULT_FLASH_PROGRAM, recordCount, (uint8_t)(nextFlashPage & 0xFF), (uint8_t)((nextFlashPage >> 8) & 0xFF));
  flashPollTimer.stop();
  if(flashInterfaceOpen && !SFLASH.busy()) {
    SFLASH.end();
    flashInterfaceOpen = false;
  }
  if(serialDebug) Serial.println("QSPI LOGGER FAILED");
}


void serviceFlashLogger()
{
  if(flashState == FLASH_IDLE || flashState == FLASH_FAILED) return;

  if(flashState == FLASH_BEGIN) {
    if(!SFLASH.begin()) { failFlash(); return; }
    flashInterfaceOpen = true;
    flashDeadline = millis() + FLASH_TIMEOUT_MS;
    if(SFLASH.busy()) {
      flashState = FLASH_WAIT_BEFORE_PROGRAM;
      if(!startFlashPoll()) failFlash();
    }
    else if(!startFlashProgram()) failFlash();
    return;
  }

  if(!flashPollDue) return;
  flashPollDue = false;
  if((int32_t)(millis() - flashDeadline) >= 0) { failFlash(); return; }
  if(SFLASH.busy()) { if(!startFlashPoll()) failFlash(); return; }
  if(flashState == FLASH_WAIT_BEFORE_PROGRAM) {
    if(!startFlashProgram()) failFlash();
    return;
  }

  bool ok = SFLASH.status() == SFLASH_STATUS_SUCCESS;
  ok = ok && SFLASH.read(nextFlashPage * LOG_PAGE_SIZE, verifyPage, sizeof(verifyPage));
  ok = ok && memcmp(logPage, verifyPage, sizeof(logPage)) == 0;
  SFLASH.end();
  flashInterfaceOpen = false;
  flashPollTimer.stop();
  if(!ok) { failFlash(); return; }

  nextFlashPage++;
  pageSequence++;
  // Yellow pulse marks a completed, verified QSPI page commit.
  pulseLED(true, true, false, 25, 2);
  if(nextFlashPage >= flashPageCount) {
    flashFull = true;
    if(serialDebug) Serial.println("QSPI LOG FULL - logging stopped without wraparound");
  }
  initializeLogPage();
  flashState = FLASH_IDLE;
}


/* --------------------------------------------------------------------------
 * HELPER FUNCTIONS - optional serial diagnostics
 * -------------------------------------------------------------------------- */

void printAEM13921Register(const char *label, uint8_t reg)
{
  uint8_t value = 0;
  Serial.print("  ");
  Serial.print(label);
  Serial.print(" = ");

  if(aem13921.readRegister(reg, &value)) {
    Serial.print("0x");
    if(value < 0x10) Serial.print("0");
    Serial.println(value, HEX);
  }
  else {
    Serial.println("read failed");
  }
}


void printAEM13921RegisterDump()
{
  if(!serialDebug || !debugAEM13921RegisterDump) return;

  Serial.println("  AEM13921 post-config register dump:");
  printAEM13921Register("SRC1REGU0", AEM13921_SRC1REGU0);
  printAEM13921Register("SRC1REGU1", AEM13921_SRC1REGU1);
  printAEM13921Register("SRC2REGU0", AEM13921_SRC2REGU0);
  printAEM13921Register("SRC2REGU1", AEM13921_SRC2REGU1);
  printAEM13921Register("VOVDIS",    AEM13921_VOVDIS);
  printAEM13921Register("VCHRDY",    AEM13921_VCHRDY);
  printAEM13921Register("VOVCH",     AEM13921_VOVCH);
  printAEM13921Register("BST1CFG",   AEM13921_BST1CFG);
  printAEM13921Register("BST2CFG",   AEM13921_BST2CFG);
  printAEM13921Register("BUCKCFG",   AEM13921_BUCKCFG);
  printAEM13921Register("CHG5V",     AEM13921_CHG5V);
  printAEM13921Register("TEMPPROT",  AEM13921_TEMPPROTECT);
  printAEM13921Register("SRCLOW",    AEM13921_SRCLOW);
  printAEM13921Register("APM",       AEM13921_APM);
  printAEM13921Register("APMACC",    AEM13921_APMACC);
  printAEM13921Register("IRQEN0",    AEM13921_IRQEN0);
  printAEM13921Register("IRQEN1",    AEM13921_IRQEN1);
  printAEM13921Register("CTRL",      AEM13921_CTRL);
  printAEM13921Register("IRQFLG0",   AEM13921_IRQFLG0);
  printAEM13921Register("IRQFLG1",   AEM13921_IRQFLG1);
  printAEM13921Register("STATUS0",   AEM13921_STATUS0);
  printAEM13921Register("STATUS1",   AEM13921_STATUS1);
  printAEM13921Register("APMERR",    AEM13921_APMERR);
}


/* --------------------------------------------------------------------------
 * HELPER FUNCTIONS - BLE NUS query reporting
 * -------------------------------------------------------------------------- */

void initializeBLE_NUS()
{
  // Register the NUS service once and keep the BLE stack initialized.
  // Full BLE.end()/BLE.begin() cycling did not reliably reopen advertising
  // with this core, so BLE is controlled only by short advertising windows.
  bleWindowState = BLE_WINDOW_SLEEP;
  blePeriodDue = true;
  bleWindowExpired = false;

  settleBLECommand(25);
  if(!BLE.begin()) return;

  BLE.setLocalName("SasqEnvLog");
  BLE.setServiceUuid(SerialBLE.uuid());
  BLE.addService(SerialBLE);
  BLE.stopAdvertise();
}


void settleBLECommand(uint16_t delayMs)
{
  /*
   * STM32WB BLE calls synchronize with the wireless stack on the second core.
   * At 32 MHz with Serial disabled, back-to-back BLE setup calls can outrun
   * that background work. Yield once, then wait briefly, so BLE setup timing is
   * deliberate instead of accidentally supplied by Serial.print().
   */
  yield();
  delay(delayMs);
}


void serviceBLE_NUS()
{
  // If commissioning failed before the first sample, stay quiet and let the
  // red LED indicate the fault. Do not advertise a failed deployment.
  if(systemFault && !latestSampleValid) return;

  if(blePeriodDue && bleWindowState == BLE_WINDOW_SLEEP && loggerIdleForBLE()) {
    blePeriodDue = false;
    startBLEAdvertiseWindow();
  }

  if(bleWindowState == BLE_WINDOW_ADVERTISING) {
    if(BLE.connected()) {
      startBLEConnectedWindow();
    } else if(bleWindowExpired) {
      closeBLEWindow();
    }
  }

  if(bleWindowState == BLE_WINDOW_CONNECTED) {
    if(!BLE.connected()) {
      closeBLEWindow();
      return;
    }

    serviceBLE_RX();
  }

  if(bleWindowState == BLE_WINDOW_RECOVERING) {
    if(bleWindowExpired) {
      finishBLERecovery();
    }
  }

  if(bleWindowState != BLE_WINDOW_SLEEP) serviceWatchdogDuringBLE();
}


bool loggerIdleForBLE()
{
  /*
   * BLE reporting is observational and opportunistic. It should only open a
   * radio window when no sample is due, no conversion is pending, and no flash
   * page is being programmed. Already-latched sensor/AEM events are serviced
   * before the next BLE opportunity rather than carried into a radio window.
   */
  return !sampleDue &&
         !measurementPending &&
         !conversionDue &&
         flashState == FLASH_IDLE &&
         !ensInterrupt &&
         !aemInterrupt;
}


bool loggerSafeToStop()
{
  return !sampleDue &&
         !conversionDue &&
         !measurementPending &&
         flashState == FLASH_IDLE &&
         !ensInterrupt &&
         !aemInterrupt &&
         !heartbeatDue &&
         !indicatorDue &&
         !ledOffDue &&
         ledPriority == 0
#if ENABLE_BLE_NUS
         && bleWindowState == BLE_WINDOW_SLEEP
#endif
         ;
}


void startBLEAdvertiseWindow()
{
  bleWindowExpired = false;
  bleWindowTimer.stop();

  if(!BLE.advertise()) {
    bleWindowState = BLE_WINDOW_RECOVERING;
    bleWindowTimer.start(bleWindowTimerHandler, BLE_RECOVER_MS);
    return;
  }

  bleWindowTimer.start(bleWindowTimerHandler, BLE_ADVERTISE_MS);
  bleWindowState = BLE_WINDOW_ADVERTISING;
}


void startBLEConnectedWindow()
{
  bleWindowExpired = false;
  bleWindowTimer.stop();
  bleWindowState = BLE_WINDOW_CONNECTED;
}


void closeBLEWindow()
{
  bleWindowTimer.stop();
  bleWindowExpired = false;

  // The phone owns disconnect. Calling BLE.disconnect() from the peripheral
  // side has produced repeatable high-current BLE states on this core.
  BLE.stopAdvertise();

  /*
   * Give the BLE stack a short quiet interval after advertising closes or the
   * phone disconnects. This avoids immediately reopening BLE at the same edge
   * where connection state is changing.
   */
  bleWindowState = BLE_WINDOW_RECOVERING;
  bleWindowTimer.start(bleWindowTimerHandler, BLE_RECOVER_MS);
}


void finishBLERecovery()
{
  bleWindowTimer.stop();
  bleWindowExpired = false;
  blePeriodDue = false;
  blePeriodTimer.stop();
  blePeriodTimer.start(blePeriodTimerHandler, BLE_PERIOD_MS);
  bleWindowState = BLE_WINDOW_SLEEP;
}


void serviceBLE_RX()
{
  int c;

  while((c = SerialBLE.read()) >= 0) {
    if(c == '?' || c == 'r' || c == 'R') {
      sendBLEReport();
    }
  }
}

void sendBLEReport()
{
  if(!latestSampleValid) {
    SerialBLE.println("NO SAMPLE YET");
    return;
  }

  const SensorSnapshot &sample = latestSample;
  bool chargeState = (sample.status0 & STATUS_CHARGING) != 0;

  SerialBLE.print(sample.hdcTemperature / 100.0f, 1);
  SerialBLE.print("C,");
  SerialBLE.print(sample.hdcHumidity / 100.0f, 1);
  SerialBLE.print("%,");
  SerialBLE.print(sample.pressureRaw / 4096.0f, 1);
  SerialBLE.print("hPa,");
  SerialBLE.print(sample.light[1] * APDS_LUX_PER_COUNT, 0);
  SerialBLE.print("lx,AQI");
  SerialBLE.print(sample.ensAQI);
  SerialBLE.print(",");
  SerialBLE.print(sample.ensTVOC);
  SerialBLE.print("ppb,");
  SerialBLE.print(sample.ensECO2);
  SerialBLE.println("ppm");

  SerialBLE.print(sample.batteryMillivolts / 1000.0f, 2);
  SerialBLE.print("V,");
  SerialBLE.print(sample.batteryPercent);
  SerialBLE.print("%,");
  SerialBLE.print(sample.storageMillivolts / 1000.0f, 2);
  SerialBLE.print("V,");
  SerialBLE.print(sample.aemAPMValid ? (sample.aemAPMSource1 / 1000UL) : 0UL);
  SerialBLE.print("K,");
  SerialBLE.print(chargeState ? "CHG," : "noCHG,");
  SerialBLE.print("0x");
  if(sample.status0 < 0x10) SerialBLE.print("0");
  SerialBLE.print(sample.status0, HEX);
  SerialBLE.print(",P");
  SerialBLE.println(sample.aemPolicyState);
}


void printSample(const SensorSnapshot &sample)
{
  if(!serialDebug) return;
  Serial.print("Sample "); Serial.print(sample.sequence);
  Serial.print(": T="); Serial.print(sample.hdcTemperature / 100.0f, 2);
  Serial.print(" C, RH="); Serial.print(sample.hdcHumidity / 100.0f, 2);
  Serial.print(" %, P="); Serial.print(sample.pressureRaw / 4096.0f, 2);
  Serial.print(" hPa, AQI="); Serial.print(sample.ensAQI);
  Serial.print(", TVOC="); Serial.print(sample.ensTVOC);
  Serial.print(" ppb, eCO2="); Serial.print(sample.ensECO2);
  Serial.print(" ppm");
  Serial.print(", VBAT="); Serial.print(sample.batteryMillivolts / 1000.0f, 3);
  Serial.print(" V, RSOC="); Serial.print(sample.batteryPercent);
  Serial.print(" %");
  Serial.print(", light="); Serial.print(sample.light[1] * APDS_LUX_PER_COUNT, 2);
  Serial.print(" lux, valid=0x"); Serial.print(sample.validMask, HEX);
#if ENABLE_AEM13921
  Serial.print(", APM=");
  if(sample.aemAPMValid) {
    Serial.print("SRC1 "); Serial.print(sample.aemAPMSource1);
    Serial.print(", SRC2 "); Serial.print(sample.aemAPMSource2);
    Serial.print(", LOAD "); Serial.print(sample.aemAPMLoad);
    Serial.print(", 5V "); Serial.print(sample.aemAPM5V);
    Serial.print(", ERR 0x"); Serial.print(sample.aemAPMError, HEX);
    Serial.print(", POLICY "); Serial.println(sample.aemPolicyState);
  } else {
    Serial.print("not ready, POLICY "); Serial.println(sample.aemPolicyState);
  }
#else
  Serial.println();
#endif
}
