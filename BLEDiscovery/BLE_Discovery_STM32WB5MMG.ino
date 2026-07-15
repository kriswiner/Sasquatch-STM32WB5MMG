/*
  Sasquatch STM32WB5MMG BLE discovery scanner, based on the working Scan example from:

    TleraCorp STM32WB Arduino core 0.1.7
    libraries/BLE/examples/Scan/Scan.ino

  This sketch uses the same STM32WB BLE API calls as Scan_Commented, but changes
  the output from a packet-by-packet decoder into a small BLE discovery tool.

  Normal mode:
    - remembers recently seen devices in a fixed-size table
    - smooths RSSI so proximity estimates are less jumpy
    - prints a compact summary at a fixed interval

  Verbose mode:
    - also prints the detailed decoded advertising report for every packet

  Before running BLE sketches on a fresh STM32WB board, the wireless stack must
  be installed with the FWUpdate example and can be checked with FWInfo.
*/

#include "Arduino.h"
#include "BLE.h"
#include "RTC.h"
#include "SFLASH.h"
#include "STM32WB.h"
#include "TimerMillis.h"
#include <string.h>

// Run configuration.
const bool ENABLE_FLASH_LOGGING = true;         // Append compact discovery summaries to QSPI flash.
const bool SERIAL_DEBUG = false;                // Set false for headless battery logging.
const bool PRINT_LIVE_SUMMARY = false;          // Useful on the bench; off for quiet field logging.
const bool PASSIVE_SCAN = false;                // False enables active scan and scan-response data.
const bool VERBOSE_OUTPUT = false;              // Set true for full decoded packet output over Serial.

const float LOW_BATTERY_THRESHOLD_V = 3.60;
const unsigned long SUMMARY_INTERVAL_MS = 10000;
const unsigned long ENVIRONMENT_INTERVAL_MS = 300000;
const unsigned long SUMMARY_LED_PULSE_MS = 1;
const unsigned long REPORT_LED_PULSE_MS = 1;
const unsigned long REPORT_LED_MIN_INTERVAL_MS = 250;
const unsigned long LOW_BATTERY_LED_PULSE_MS = 100;
const unsigned long DEVICE_STALE_MS = 60000;
const int MAX_TRACKED_DEVICES = 48;
const int RSSI_SMOOTHING_PERCENT = 25;          // New RSSI weight; 25% is responsive but not too jumpy.
const uint8_t LOG_MAGIC_0 = 0x42;               // 'B'
const uint8_t LOG_MAGIC_1 = 0x4C;               // 'L'
const uint8_t LOG_VERSION = 1;
const uint8_t PAGE_TYPE_SESSION = 1;
const uint8_t PAGE_TYPE_DEVICE = 2;
const uint8_t PAGE_TYPE_CHECKPOINT = 3;
const uint8_t LOG_MARKER = 0xB5;
const uint16_t LOG_PAGE_SIZE = 256;
const uint16_t LOG_HEADER_SIZE = 23;
const uint16_t LOG_DEVICE_RECORD_SIZE = 23;
const uint8_t LOG_DEVICE_RECORDS_PER_PAGE = 10;
const uint16_t LOG_CRC_MSB_BYTE = 253;
const uint16_t LOG_CRC_LSB_BYTE = 254;
const uint16_t LOG_MARKER_BYTE = 255;
const uint32_t FLASH_TIMEOUT_MS = 1000;

struct DeviceRecord {
    bool active;
    char address[24];
    BLEAddressType addressType;
    char name[32];
    char firstServiceUuid[40];
    uint16_t manufacturerId;
    bool hasManufacturerId;
    bool connectable;
    bool sawAdvertisement;
    bool sawScanResponse;
    int lastRssi;
    int smoothedRssi;
    int bestRssi;
    unsigned long firstSeenMs;
    unsigned long lastSeenMs;
    unsigned long seenCount;
    unsigned long lastLoggedSeenCount;
};

DeviceRecord devices[MAX_TRACKED_DEVICES];
TimerMillis summaryTimer;
TimerMillis environmentTimer;
TimerMillis ledTimer;
uint8_t logPage[LOG_PAGE_SIZE];
uint8_t flashMid = 0;
uint16_t flashDid = 0;
uint32_t nextLogPage = 0;
uint32_t logPageCount = 0;
uint32_t logSequenceNumber = 0;
uint32_t logSessionId = 0;
uint32_t logBatchIndex = 0;
unsigned long reportCount = 0;
unsigned long lastReportLedMs = 0;
volatile bool summaryDue = false;
volatile bool environmentDue = false;
volatile bool ledOffDue = false;
bool lowBattery = false;
bool flashLoggingReady = false;
float batteryVoltage = 0.0;
float mcuTemperatureC = 0.0;
uint8_t activeLedPin = 0xff;

void summaryTimerCallback();
void environmentTimerCallback();
void ledTimerCallback();
void sampleEnvironment();
void setupStatusLed();
void ledOff(uint8_t pin);
void ledPulse(uint8_t pin, unsigned long durationMs);
void serviceStatusLed();
void print2Digits(uint8_t value);
void printHexByte(uint8_t value);
void printHexBytes(const uint8_t data[], int length);
void printRtcDate();
void printRtcTime();
void printStartupInfo(const char scannerAddress[]);
const char *addressTypeName(BLEAddressType type);
const char *rssiLabel(int rssi);
const char *companyName(uint16_t companyId);
void printPacketKind(BLEScanEvent event);
void copyText(char destination[], int destinationSize, const char source[]);
int findDevice(const char addressText[], BLEAddressType addressType);
int findFreeDeviceSlot();
int smoothRssi(int previousRssi, int newRssi);
void updateDevice(BLEAddress address, int rssi, BLEScanEvent event, BLEAdvertisingData data);
int activeDeviceCount();
void printSummary();
void printVerboseReport(BLEAddress address, int rssi, BLEScanEvent event, BLEAdvertisingData data);
bool beginFlashLog();
bool writeSessionPage();
void logSummaryToFlash();
void writeDeviceBatch(uint8_t sortedIndices[], uint8_t recordCount, uint8_t activeCount);
void writeCheckpointPage();
void clearLogPage();
void fillLogHeader(uint8_t pageType, uint32_t batchIndex, uint8_t batchPageIndex, uint8_t batchPageCount, uint8_t recordCount, uint8_t activeCount, uint8_t flags);
void fillLogDeviceRecord(uint16_t offset, DeviceRecord &device);
bool commitLogPage();
bool findFirstErasedLogPage();
bool logPageBufferIsErased();
bool waitForFlash();
uint16_t calculateCRC16(const uint8_t *data, uint16_t length);
uint16_t textHash16(const char text[]);
uint32_t textHash32(const char text[]);
uint16_t clampU16(unsigned long value);
uint8_t logAddressType(BLEAddressType type);
uint8_t logFlags(const DeviceRecord &device);
void parseAddressBytes(const char addressText[], uint8_t addressBytes[]);
void putLogU16(uint16_t offset, uint16_t value);
void putLogU32(uint16_t offset, uint32_t value);
void putLogI16(uint16_t offset, int16_t value);

void setup()
{
    char string[64];

    // Serial is optional. Battery-only logging must not wait for USB Serial.
    if (SERIAL_DEBUG) {
        Serial.begin(9600);
    }

    // The RGB LED gives simple visible status without needing a display.
    setupStatusLed();

    // Start BLE as a scanner only. The local name is harmless here, but useful
    // if this sketch is later expanded to advertise or connect.
    BLE.begin();
    BLE.setLocalName("Sasquatch");

    BLE.address().toString(string, sizeof(string) - 1);
    printStartupInfo(string);

    // Take an initial slow-environment sample so the first summary has context.
    sampleEnvironment();

    // Set up append-only flash logging before BLE scanning starts. Discovery
    // still runs over Serial if the flash is absent or full.
    if (ENABLE_FLASH_LOGGING) {
        flashLoggingReady = beginFlashLog();
        if (SERIAL_DEBUG && Serial) {
            Serial.print("Flash log ready: ");
            Serial.println(flashLoggingReady ? "yes" : "no");
        }
    }

    // Configure scan behavior before starting the BLE observation procedure.
    BLE.setScanType(PASSIVE_SCAN ? BLE_SCAN_TYPE_PASSIVE : BLE_SCAN_TYPE_ACTIVE);
    BLE.scan(BLE_SCAN_MODE_NONE);               // Keep duplicates so RSSI/proximity and seen counts stay useful.

    // TimerMillis callbacks only set flags and wake the MCU. The loop does the
    // real work so callbacks remain short and interrupt-like.
    summaryTimer.start(summaryTimerCallback, SUMMARY_INTERVAL_MS, SUMMARY_INTERVAL_MS);
    environmentTimer.start(environmentTimerCallback, ENVIRONMENT_INTERVAL_MS, ENVIRONMENT_INTERVAL_MS);

    if (SERIAL_DEBUG && Serial) {
        Serial.println("Discovering nearby BLE advertisers...");
        Serial.println();
    }
}

void loop()
{
    BLEAddress address;
    BLEScanEvent event;
    BLEAdvertisingData data;
    int rssi;

    serviceStatusLed();

    // Consume one pending BLE report, update the discovery table, and optionally
    // print the full decoded packet for protocol-level inspection.
    if (BLE.report(address, rssi, event, data)) {
        reportCount++;
        updateDevice(address, rssi, event, data);

        if ((millis() - lastReportLedMs) >= REPORT_LED_MIN_INTERVAL_MS) {
            lastReportLedMs = millis();
            ledPulse(PIN_LED_BLUE, REPORT_LED_PULSE_MS);
        }

        if (SERIAL_DEBUG && Serial && VERBOSE_OUTPUT) {
            printVerboseReport(address, rssi, event, data);
        }
    }

    // Slow environmental values are sampled separately from the BLE summary
    // cadence because battery voltage and temperature change slowly.
    if (environmentDue) {
        noInterrupts();
        environmentDue = false;
        interrupts();
        sampleEnvironment();
        writeCheckpointPage();
    }

    // Print/log summaries on the timer cadence using the latest known table and
    // environment values.
    if (summaryDue) {
        noInterrupts();
        summaryDue = false;
        interrupts();
        ledPulse(lowBattery ? PIN_LED_RED : PIN_LED_GREEN,
                 lowBattery ? LOW_BATTERY_LED_PULSE_MS : SUMMARY_LED_PULSE_MS);
        if (SERIAL_DEBUG && Serial && PRINT_LIVE_SUMMARY) {
            printSummary();
        }
        logSummaryToFlash();
    }

    // When STM32WB.stop() is fixed in the core, call it here after all pending
    // BLE reports, timer flags, and LED events have been serviced.
}

void summaryTimerCallback()
{
    summaryDue = true;
    STM32WB.wakeup();
}

void environmentTimerCallback()
{
    environmentDue = true;
    STM32WB.wakeup();
}

void ledTimerCallback()
{
    ledOffDue = true;
    STM32WB.wakeup();
}

void sampleEnvironment()
{
    batteryVoltage = STM32WB.readBattery();
    mcuTemperatureC = STM32WB.readTemperature();
    lowBattery = batteryVoltage > 0.0 && batteryVoltage < LOW_BATTERY_THRESHOLD_V;
}

void setupStatusLed()
{
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_LED_BLUE, OUTPUT);
    ledOff(PIN_LED_RED);
    ledOff(PIN_LED_GREEN);
    ledOff(PIN_LED_BLUE);
}

void ledOff(uint8_t pin)
{
    digitalWrite(pin, HIGH);                    // Sasquatch RGB LED channels are active LOW.
}

void ledPulse(uint8_t pin, unsigned long durationMs)
{
    if (activeLedPin != 0xff) {
        ledOff(activeLedPin);
    }

    activeLedPin = pin;
    digitalWrite(pin, LOW);
    ledOffDue = false;
    ledTimer.start(ledTimerCallback, durationMs);
}

void serviceStatusLed()
{
    if (ledOffDue) {
        noInterrupts();
        ledOffDue = false;
        interrupts();

        if (activeLedPin != 0xff) {
            ledOff(activeLedPin);
            activeLedPin = 0xff;
        }
    }
}

void print2Digits(uint8_t value)
{
    if (value < 10) {
        Serial.print("0");
    }

    Serial.print(value);
}

void printHexByte(uint8_t value)
{
    if (value < 16) {
        Serial.print("0");
    }

    Serial.print(value, HEX);
}

void printHexBytes(const uint8_t data[], int length)
{
    for (int index = 0; index < length; index++) {
        printHexByte(data[index]);
    }
}

void printRtcDate()
{
    uint8_t day, month;
    uint16_t year;

    day = RTC.getDay();
    month = RTC.getMonth();
    year = RTC.getYear();

    Serial.print(year);
    Serial.print("-");
    print2Digits(month);
    Serial.print("-");
    print2Digits(day);
}

void printRtcTime()
{
    uint8_t hours, minutes, seconds;
    uint16_t milliSeconds;

    RTC.getTime(hours, minutes, seconds, milliSeconds);

    print2Digits(hours);
    Serial.print(":");
    print2Digits(minutes);
    Serial.print(":");
    print2Digits(seconds);
    Serial.print(".");

    if (milliSeconds < 100) {
        Serial.print("0");
    }

    if (milliSeconds < 10) {
        Serial.print("0");
    }

    Serial.print(milliSeconds);
}

void printStartupInfo(const char scannerAddress[])
{
    if (!SERIAL_DEBUG || !Serial) {
        return;
    }

    Serial.println();
    Serial.println();
    Serial.println("Sasquatch STM32WB5MMG BLE discovery");
    Serial.println("------------------------------------");
    Serial.print("Scanner address: ");
    Serial.println(scannerAddress);
    Serial.print("Verbose output:  ");
    Serial.println(VERBOSE_OUTPUT ? "yes" : "no");
    Serial.print("Scan type:       ");
    Serial.println(PASSIVE_SCAN ? "passive" : "active");
    Serial.print("Scan duplicates: ");
    Serial.println("yes");
    Serial.print("Flash logging:   ");
    Serial.println(ENABLE_FLASH_LOGGING ? "yes" : "no");
    Serial.print("Low battery:     ");
    Serial.print(LOW_BATTERY_THRESHOLD_V, 2);
    Serial.println(" V");
    Serial.print("Summary period:  ");
    Serial.print(SUMMARY_INTERVAL_MS / 1000);
    Serial.println(" s");
    Serial.print("Environment period: ");
    Serial.print(ENVIRONMENT_INTERVAL_MS / 60000);
    Serial.println(" min");
    Serial.print("Stale timeout:   ");
    Serial.print(DEVICE_STALE_MS / 1000);
    Serial.println(" s");
    Serial.print("RTC date now:    ");
    printRtcDate();
    Serial.println();
    Serial.print("RTC time now:    ");
    printRtcTime();
    Serial.println();
    Serial.println();
}

const char *addressTypeName(BLEAddressType type)
{
    switch (type) {
        case BLE_ADDRESS_TYPE_PUBLIC: return "public";
        case BLE_ADDRESS_TYPE_RANDOM_STATIC: return "random static";
        case BLE_ADDRESS_TYPE_RANDOM_PRIVATE_RESOLVABLE: return "random private resolvable";
        case BLE_ADDRESS_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE: return "random private non-resolvable";
        default: return "unknown";
    }
}

const char *rssiLabel(int rssi)
{
    if (rssi > -50) {
        return "strong";
    }

    if (rssi > -75) {
        return "medium";
    }

    return "weak";
}

const char *companyName(uint16_t companyId)
{
    switch (companyId) {
        case 0x0006: return "Microsoft";
        case 0x000D: return "Texas Instruments";
        case 0x0030: return "STMicroelectronics";
        case 0x004C: return "Apple";
        case 0x0059: return "Nordic Semiconductor";
        case 0x00E0: return "Google";
        default: return "unknown company";
    }
}

void printPacketKind(BLEScanEvent event)
{
    if (event & BLE_SCAN_EVENT_RESPONSE) {
        Serial.print("scan response");
    } else if (event & BLE_SCAN_EVENT_ADVERTISEMENT) {
        Serial.print("advertisement");
    } else {
        Serial.print("unknown");
    }

    if (event & BLE_SCAN_EVENT_CONNECTABLE) {
        Serial.print(", connectable");
    } else {
        Serial.print(", non-connectable");
    }
}

void copyText(char destination[], int destinationSize, const char source[])
{
    if (destinationSize <= 0) {
        return;
    }

    strncpy(destination, source, destinationSize - 1);
    destination[destinationSize - 1] = '\0';
}

int findDevice(const char addressText[], BLEAddressType addressType)
{
    for (int index = 0; index < MAX_TRACKED_DEVICES; index++) {
        if (devices[index].active &&
            devices[index].addressType == addressType &&
            strcmp(devices[index].address, addressText) == 0) {
            return index;
        }
    }

    return -1;
}

int findFreeDeviceSlot()
{
    int oldestIndex = 0;
    unsigned long oldestSeenMs = devices[0].lastSeenMs;

    for (int index = 0; index < MAX_TRACKED_DEVICES; index++) {
        if (!devices[index].active) {
            return index;
        }

        if (devices[index].lastSeenMs < oldestSeenMs) {
            oldestSeenMs = devices[index].lastSeenMs;
            oldestIndex = index;
        }
    }

    return oldestIndex;
}

int smoothRssi(int previousRssi, int newRssi)
{
    return ((previousRssi * (100 - RSSI_SMOOTHING_PERCENT)) +
            (newRssi * RSSI_SMOOTHING_PERCENT)) / 100;
}

void updateDevice(BLEAddress address, int rssi, BLEScanEvent event, BLEAdvertisingData data)
{
    char addressText[24];
    char name[32];
    char uuidText[40];
    uint8_t manufacturerData[32];
    BLEUuid uuid;
    BLEUuid uuidTable[8];
    int count;
    int length;
    unsigned long nowMs = millis();

    address.toString(addressText, sizeof(addressText) - 1);

    int index = findDevice(addressText, address.type());
    bool isNewDevice = false;

    if (index < 0) {
        index = findFreeDeviceSlot();
        memset(&devices[index], 0, sizeof(devices[index]));
        devices[index].active = true;
        devices[index].addressType = address.type();
        copyText(devices[index].address, sizeof(devices[index].address), addressText);
        devices[index].firstSeenMs = nowMs;
        devices[index].bestRssi = rssi;
        devices[index].smoothedRssi = rssi;
        isNewDevice = true;
    }

    devices[index].lastSeenMs = nowMs;
    devices[index].seenCount++;
    devices[index].lastRssi = rssi;

    if (!isNewDevice) {
        devices[index].smoothedRssi = smoothRssi(devices[index].smoothedRssi, rssi);
    }

    if (rssi > devices[index].bestRssi) {
        devices[index].bestRssi = rssi;
    }

    if (event & BLE_SCAN_EVENT_CONNECTABLE) {
        devices[index].connectable = true;
    }

    if (event & BLE_SCAN_EVENT_ADVERTISEMENT) {
        devices[index].sawAdvertisement = true;
    }

    if (event & BLE_SCAN_EVENT_RESPONSE) {
        devices[index].sawScanResponse = true;
    }

    if (data.getLocalName(name, sizeof(name))) {
        copyText(devices[index].name, sizeof(devices[index].name), name);
    }

    if (data.getServiceUuids(uuidTable, (sizeof(uuidTable) / sizeof(uuidTable[0])), count) && count > 0) {
        uuidTable[0].toString(uuidText, sizeof(uuidText) - 1);
        copyText(devices[index].firstServiceUuid, sizeof(devices[index].firstServiceUuid), uuidText);
    }

    if (data.getServiceData(uuid, manufacturerData, sizeof(manufacturerData), length)) {
        uuid.toString(uuidText, sizeof(uuidText) - 1);
        copyText(devices[index].firstServiceUuid, sizeof(devices[index].firstServiceUuid), uuidText);
    }

    if (data.getManufacturerData(manufacturerData, sizeof(manufacturerData), length) && length >= 2) {
        devices[index].manufacturerId = (uint16_t)manufacturerData[0] | ((uint16_t)manufacturerData[1] << 8);
        devices[index].hasManufacturerId = true;
    }
}

int activeDeviceCount()
{
    int count = 0;
    unsigned long nowMs = millis();

    for (int index = 0; index < MAX_TRACKED_DEVICES; index++) {
        if (devices[index].active && (nowMs - devices[index].lastSeenMs) <= DEVICE_STALE_MS) {
            count++;
        }
    }

    return count;
}

void printSummary()
{
    unsigned long nowMs = millis();

    Serial.println();
    Serial.print("BLE discovery summary at ");
    printRtcTime();
    Serial.print("  (active ");
    Serial.print(activeDeviceCount());
    Serial.print("/");
    Serial.print(MAX_TRACKED_DEVICES);
    Serial.print(", battery ");
    if (batteryVoltage > 0.0) {
        Serial.print(batteryVoltage, 2);
        Serial.print(" V");
        if (lowBattery) {
            Serial.print(" LOW");
        }
    } else {
        Serial.print("n/a");
    }
    Serial.print(", MCU ");
    Serial.print(mcuTemperatureC, 1);
    Serial.print(" C");
    Serial.println(")");
    Serial.println("Age  RSSI  Best  Seen  Type                         Flags  Identity");
    Serial.println("---  ----  ----  ----  ---------------------------  -----  --------");

    for (int index = 0; index < MAX_TRACKED_DEVICES; index++) {
        if (!devices[index].active || (nowMs - devices[index].lastSeenMs) > DEVICE_STALE_MS) {
            continue;
        }

        Serial.print((nowMs - devices[index].lastSeenMs) / 1000);
        Serial.print("s");
        Serial.print("   ");
        Serial.print(devices[index].smoothedRssi);
        Serial.print("   ");
        Serial.print(devices[index].bestRssi);
        Serial.print("   ");
        Serial.print(devices[index].seenCount);
        Serial.print("   ");
        Serial.print(addressTypeName(devices[index].addressType));
        Serial.print("   ");

        Serial.print(devices[index].connectable ? "C" : "-");        // C = connectable
        Serial.print(devices[index].sawAdvertisement ? "A" : "-");   // A = advertisement seen
        Serial.print(devices[index].sawScanResponse ? "R" : "-");    // R = scan response seen
        Serial.print("    ");

        Serial.print(devices[index].address);

        if (devices[index].name[0]) {
            Serial.print("  name=");
            Serial.print(devices[index].name);
        }

        if (devices[index].hasManufacturerId) {
            Serial.print("  mfg=0x");
            printHexByte((uint8_t)(devices[index].manufacturerId >> 8));
            printHexByte((uint8_t)(devices[index].manufacturerId & 0xFF));
            Serial.print("(");
            Serial.print(companyName(devices[index].manufacturerId));
            Serial.print(")");
        }

        if (devices[index].firstServiceUuid[0]) {
            Serial.print("  uuid=");
            Serial.print(devices[index].firstServiceUuid);
        }

        Serial.print("  ");
        Serial.print(rssiLabel(devices[index].smoothedRssi));
        Serial.println();
    }

    Serial.println();
}

bool beginFlashLog()
{
    if (!SFLASH.begin() || !SFLASH.identify(flashMid, flashDid)) {
        SFLASH.end();
        return false;
    }

    if (SFLASH.pageSize() != LOG_PAGE_SIZE) {
        SFLASH.end();
        return false;
    }

    logPageCount = SFLASH.length() / SFLASH.pageSize();
    if (logPageCount > 0xFFFF) {
        logPageCount = 0xFFFF;
    }

    if (!findFirstErasedLogPage()) {
        SFLASH.end();
        return false;
    }

    logSessionId = millis();
    return writeSessionPage();
}

bool writeSessionPage()
{
    uint8_t hours, minutes, seconds;
    uint16_t milliseconds;
    const struct _armv7m_fault_info_t *faultInfo;

    if (!ENABLE_FLASH_LOGGING) {
        return false;
    }

    clearLogPage();
    fillLogHeader(PAGE_TYPE_SESSION, 0, 0, 1, 0, 0, 0);

    putLogU16(23, RTC.getYear());
    logPage[25] = RTC.getMonth();
    logPage[26] = RTC.getDay();
    RTC.getTime(hours, minutes, seconds, milliseconds);
    logPage[27] = hours;
    logPage[28] = minutes;
    logPage[29] = seconds;
    putLogU16(30, SUMMARY_INTERVAL_MS / 1000);
    putLogU16(32, ENVIRONMENT_INTERVAL_MS / 1000);
    logPage[34] = MAX_TRACKED_DEVICES;
    putLogU32(35, STM32WB.resetCause());
    putLogU32(39, STM32WB.wakeupReason());
    putLogU32(43, STM32WB.faultReason(faultInfo));

    return commitLogPage();
}

void logSummaryToFlash()
{
    uint8_t sortedIndices[MAX_TRACKED_DEVICES];
    uint8_t recordCount = 0;
    unsigned long nowMs = millis();

    if (!ENABLE_FLASH_LOGGING || !flashLoggingReady) {
        return;
    }

    for (uint8_t index = 0; index < MAX_TRACKED_DEVICES; index++) {
        if (devices[index].active && (nowMs - devices[index].lastSeenMs) <= DEVICE_STALE_MS) {
            sortedIndices[recordCount++] = index;
        }
    }

    for (uint8_t i = 0; i < recordCount; i++) {
        for (uint8_t j = i + 1; j < recordCount; j++) {
            if (devices[sortedIndices[j]].smoothedRssi > devices[sortedIndices[i]].smoothedRssi) {
                uint8_t temp = sortedIndices[i];
                sortedIndices[i] = sortedIndices[j];
                sortedIndices[j] = temp;
            }
        }
    }

    writeDeviceBatch(sortedIndices, recordCount, recordCount);
    logBatchIndex++;
}

void writeDeviceBatch(uint8_t sortedIndices[], uint8_t recordCount, uint8_t activeCount)
{
    uint8_t pagesInBatch = (recordCount + LOG_DEVICE_RECORDS_PER_PAGE - 1) / LOG_DEVICE_RECORDS_PER_PAGE;
    uint8_t remaining = recordCount;
    uint8_t nextRecord = 0;

    if (!ENABLE_FLASH_LOGGING || !flashLoggingReady) {
        return;
    }

    if (pagesInBatch == 0) {
        pagesInBatch = 1;
    }

    for (uint8_t pageIndex = 0; pageIndex < pagesInBatch; pageIndex++) {
        uint8_t recordsThisPage = (remaining > LOG_DEVICE_RECORDS_PER_PAGE) ? LOG_DEVICE_RECORDS_PER_PAGE : remaining;

        clearLogPage();
        fillLogHeader(PAGE_TYPE_DEVICE, logBatchIndex, pageIndex, pagesInBatch, recordsThisPage, activeCount, PASSIVE_SCAN ? 0x00 : 0x01);

        for (uint8_t slot = 0; slot < recordsThisPage; slot++) {
            uint8_t deviceIndex = sortedIndices[nextRecord++];
            fillLogDeviceRecord(LOG_HEADER_SIZE + (slot * LOG_DEVICE_RECORD_SIZE), devices[deviceIndex]);
        }

        if (remaining >= recordsThisPage) {
            remaining -= recordsThisPage;
        }

        if (!commitLogPage()) {
            flashLoggingReady = false;
            return;
        }
    }
}

void writeCheckpointPage()
{
    uint8_t hours, minutes, seconds;
    uint16_t milliseconds;

    if (!ENABLE_FLASH_LOGGING || !flashLoggingReady) {
        return;
    }

    clearLogPage();
    fillLogHeader(PAGE_TYPE_CHECKPOINT, logBatchIndex, 0, 1, 0, 0, 0);

    putLogU16(23, RTC.getYear());
    logPage[25] = RTC.getMonth();
    logPage[26] = RTC.getDay();
    RTC.getTime(hours, minutes, seconds, milliseconds);
    logPage[27] = hours;
    logPage[28] = minutes;
    logPage[29] = seconds;
    putLogU16(30, (uint16_t)(batteryVoltage * 1000.0f));
    putLogI16(32, (int16_t)(mcuTemperatureC * 100.0f));
    putLogI16(34, 0);                         // LIS2DW12 temperature placeholder.

    if (!commitLogPage()) {
        flashLoggingReady = false;
    }
}

void clearLogPage()
{
    memset(logPage, 0xFF, sizeof(logPage));
}

void fillLogHeader(uint8_t pageType, uint32_t batchIndex, uint8_t batchPageIndex, uint8_t batchPageCount, uint8_t recordCount, uint8_t activeCount, uint8_t flags)
{
    logPage[0] = LOG_MAGIC_0;
    logPage[1] = LOG_MAGIC_1;
    logPage[2] = LOG_VERSION;
    logPage[3] = pageType;
    putLogU16(4, (uint16_t)nextLogPage);
    putLogU32(6, logSequenceNumber);
    putLogU32(10, logSessionId);
    putLogU32(14, batchIndex);
    logPage[18] = batchPageIndex;
    logPage[19] = batchPageCount;
    logPage[20] = recordCount;
    logPage[21] = activeCount;
    logPage[22] = flags;
}

void fillLogDeviceRecord(uint16_t offset, DeviceRecord &device)
{
    uint8_t addressBytes[6];
    unsigned long seenDelta = device.seenCount - device.lastLoggedSeenCount;
    unsigned long ageSeconds = (millis() - device.lastSeenMs) / 1000;

    parseAddressBytes(device.address, addressBytes);

    for (uint8_t i = 0; i < 6; i++) {
        logPage[offset + i] = addressBytes[i];
    }

    logPage[offset + 6] = logAddressType(device.addressType);
    logPage[offset + 7] = logFlags(device);
    logPage[offset + 8] = (uint8_t)(int8_t)device.smoothedRssi;
    logPage[offset + 9] = (uint8_t)(int8_t)device.bestRssi;
    logPage[offset + 10] = (uint8_t)(int8_t)device.lastRssi;
    logPage[offset + 11] = (uint8_t)((ageSeconds > 255UL) ? 255 : ageSeconds);
    putLogU16(offset + 12, clampU16(seenDelta));
    putLogU16(offset + 14, device.hasManufacturerId ? device.manufacturerId : 0xFFFF);
    putLogU32(offset + 16, textHash32(device.firstServiceUuid));
    putLogU16(offset + 20, textHash16(device.name));
    logPage[offset + 22] = 0;

    device.lastLoggedSeenCount = device.seenCount;
}

bool commitLogPage()
{
    uint16_t crc = calculateCRC16(logPage, LOG_CRC_MSB_BYTE);
    bool wasScanning = false;

    logPage[LOG_CRC_MSB_BYTE] = (crc >> 8) & 0xFF;
    logPage[LOG_CRC_LSB_BYTE] = crc & 0xFF;
    logPage[LOG_MARKER_BYTE] = LOG_MARKER;

    if (nextLogPage >= logPageCount) {
        if (SERIAL_DEBUG && Serial) {
            Serial.println("Flash log is full.");
        }
        return false;
    }

    wasScanning = BLE.scanning();
    if (wasScanning) {
        BLE.stopScan();
        delay(2);
    }

    bool ok = SFLASH.program(nextLogPage * LOG_PAGE_SIZE, logPage, sizeof(logPage));
    if (ok) {
        ok = waitForFlash();
    }

    if (wasScanning) {
        BLE.scan(BLE_SCAN_MODE_NONE);
    }

    if (!ok) {
        if (SERIAL_DEBUG && Serial) {
            Serial.println("Flash log write failed.");
        }
        return false;
    }

    nextLogPage++;
    logSequenceNumber++;
    return true;
}

bool findFirstErasedLogPage()
{
    for (nextLogPage = 0; nextLogPage < logPageCount; nextLogPage++) {
        if (!SFLASH.read(nextLogPage * LOG_PAGE_SIZE, logPage, sizeof(logPage))) {
            return false;
        }

        if (logPageBufferIsErased()) {
            return true;
        }
    }

    return false;
}

bool logPageBufferIsErased()
{
    for (uint16_t i = 0; i < sizeof(logPage); i++) {
        if (logPage[i] != 0xFF) {
            return false;
        }
    }

    return true;
}

bool waitForFlash()
{
    uint32_t start = millis();

    while (SFLASH.busy()) {
        if ((millis() - start) >= FLASH_TIMEOUT_MS) {
            return false;
        }
    }

    return SFLASH.status() == SFLASH_STATUS_SUCCESS;
}

uint16_t calculateCRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    while (length--) {
        crc ^= (uint16_t)(*data++) << 8;

        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        }
    }

    return crc;
}

uint16_t textHash16(const char text[])
{
    return (uint16_t)(textHash32(text) & 0xFFFF);
}

uint32_t textHash32(const char text[])
{
    uint32_t hash = 2166136261UL;

    if (!text || !text[0]) {
        return 0;
    }

    while (*text) {
        hash ^= (uint8_t)*text++;
        hash *= 16777619UL;
    }

    return hash;
}

uint16_t clampU16(unsigned long value)
{
    return (value > 0xFFFFUL) ? 0xFFFF : (uint16_t)value;
}

uint8_t logAddressType(BLEAddressType type)
{
    switch (type) {
        case BLE_ADDRESS_TYPE_PUBLIC: return 0;
        case BLE_ADDRESS_TYPE_RANDOM_STATIC: return 2;
        case BLE_ADDRESS_TYPE_RANDOM_PRIVATE_RESOLVABLE: return 3;
        case BLE_ADDRESS_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE: return 4;
        default: return 1;
    }
}

uint8_t logFlags(const DeviceRecord &device)
{
    uint8_t flags = 0;

    if (device.connectable) {
        flags |= 0x01;
    }

    if (device.sawAdvertisement) {
        flags |= 0x02;
    }

    if (device.sawScanResponse) {
        flags |= 0x04;
    }

    return flags;
}

void parseAddressBytes(const char addressText[], uint8_t addressBytes[])
{
    uint8_t byteIndex = 0;
    uint8_t nibbleCount = 0;

    memset(addressBytes, 0, 6);

    for (uint8_t i = 0; addressText[i] && byteIndex < 6; i++) {
        char c = addressText[i];
        uint8_t value;

        if (c >= '0' && c <= '9') {
            value = c - '0';
        } else if (c >= 'a' && c <= 'f') {
            value = c - 'a' + 10;
        } else if (c >= 'A' && c <= 'F') {
            value = c - 'A' + 10;
        } else {
            continue;
        }

        addressBytes[byteIndex] = (addressBytes[byteIndex] << 4) | value;
        nibbleCount++;

        if (nibbleCount == 2) {
            nibbleCount = 0;
            byteIndex++;
        }
    }
}

void putLogU16(uint16_t offset, uint16_t value)
{
    logPage[offset] = (value >> 8) & 0xFF;
    logPage[offset + 1] = value & 0xFF;
}

void putLogU32(uint16_t offset, uint32_t value)
{
    logPage[offset] = (value >> 24) & 0xFF;
    logPage[offset + 1] = (value >> 16) & 0xFF;
    logPage[offset + 2] = (value >> 8) & 0xFF;
    logPage[offset + 3] = value & 0xFF;
}

void putLogI16(uint16_t offset, int16_t value)
{
    putLogU16(offset, (uint16_t)value);
}

void printVerboseReport(BLEAddress address, int rssi, BLEScanEvent event, BLEAdvertisingData data)
{
    char string[64];
    char name[32];
    uint8_t serviceData[32];
    uint8_t manufacturerData[32];
    BLEUuid uuid;
    BLEUuid uuidTable[16];
    int txPower;
    int count;
    int length;
    int beaconMeasuredPower;
    uint16_t intervalMin, intervalMax;
    uint16_t major, minor;

    address.toString(string, sizeof(string) - 1);

    Serial.print("Report #");
    Serial.println(reportCount);

    Serial.print("  RTC date:         ");
    printRtcDate();
    Serial.println();

    Serial.print("  RTC time:         ");
    printRtcTime();
    Serial.println();

    Serial.print("  Uptime:           ");
    Serial.print(millis());
    Serial.println(" ms");

    Serial.print("  Address:          ");
    Serial.print(string);
    Serial.print("  (type ");
    Serial.print(address.type());
    Serial.println(")");

    Serial.print("  Address type:     ");
    Serial.println(addressTypeName(address.type()));

    Serial.print("  RSSI:             ");
    Serial.print(rssi);
    Serial.println(" dBm");

    Serial.print("  Signal:           ");
    Serial.println(rssiLabel(rssi));

    Serial.print("  Packet kind:      ");
    printPacketKind(event);
    Serial.println();

    Serial.print("  Event flags:      ");

    if (event & BLE_SCAN_EVENT_CONNECTABLE) {
        Serial.print("connectable ");
    }

    if (event & BLE_SCAN_EVENT_ADVERTISEMENT) {
        Serial.print("advertisement ");
    }

    if (event & BLE_SCAN_EVENT_RESPONSE) {
        Serial.print("scan-response ");
    }

    Serial.println();

    Serial.print("  Adv length:       ");
    Serial.print(data.length());
    Serial.println(" bytes");

    Serial.print("  Raw adv data:     ");
    if (data.length()) {
        printHexBytes(data.data(), data.length());
    } else {
        Serial.print("(empty)");
    }
    Serial.println();

    if (data.getTxPowerLevel(txPower)) {
        Serial.print("  TX power:         ");
        Serial.print(txPower);
        Serial.println(" dBm");
    }

    if (data.getConnectionInterval(intervalMin, intervalMax)) {
        Serial.print("  Conn interval:    ");
        Serial.print(intervalMin);
        Serial.print(" to ");
        Serial.print(intervalMax);
        Serial.println(" BLE units");
    }

    if (data.getLocalName(name, sizeof(name))) {
        Serial.print("  Local name:       ");
        Serial.println(name);
    }

    if (data.getServiceUuids(uuidTable, (sizeof(uuidTable) / sizeof(uuidTable[0])), count)) {
        Serial.print("  Service UUIDs:    ");

        for (int index = 0; index < count; index++) {
            uuidTable[index].toString(string, sizeof(string) - 1);

            if (index) {
                Serial.print(", ");
            }

            Serial.print(string);
        }

        Serial.println();
    }

    if (data.getServiceData(uuid, serviceData, sizeof(serviceData), length)) {
        uuid.toString(string, sizeof(string) - 1);

        Serial.print("  Service data UUID:");
        Serial.println(string);

        Serial.print("  Service data:     ");
        if (length) {
            printHexBytes(serviceData, length);
        } else {
            Serial.print("(empty)");
        }
        Serial.println();
    }

    if (data.getManufacturerData(manufacturerData, sizeof(manufacturerData), length)) {
        Serial.print("  Manufacturer ID:  ");

        if (length >= 2) {
            uint16_t companyId = (uint16_t)manufacturerData[0] | ((uint16_t)manufacturerData[1] << 8);

            printHexByte(manufacturerData[1]);
            printHexByte(manufacturerData[0]);
            Serial.print("  (");
            Serial.print(companyName(companyId));
            Serial.print(")");
        } else {
            Serial.print("(missing)");
        }
        Serial.println();

        Serial.print("  Manufacturer data:");
        if (length > 2) {
            printHexBytes(&manufacturerData[2], length - 2);
        } else {
            Serial.print("(none)");
        }
        Serial.println();
    }

    if (data.getBeacon(uuid, major, minor, beaconMeasuredPower)) {
        uuid.toString(string, sizeof(string) - 1);

        Serial.print("  Beacon UUID:      ");
        Serial.println(string);
        Serial.print("  Beacon major:     ");
        Serial.println(major);
        Serial.print("  Beacon minor:     ");
        Serial.println(minor);
        Serial.print("  Beacon power:     ");
        Serial.print(beaconMeasuredPower);
        Serial.println(" dBm");
    }

    Serial.println();
}
