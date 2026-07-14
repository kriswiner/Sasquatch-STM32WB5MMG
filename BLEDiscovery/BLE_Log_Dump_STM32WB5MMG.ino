/*
  BLE_Log_Dump_STM32WB5MMG

  Reads the BLE discovery binary log from Sasquatch QSPI flash and prints either
  raw device rows, a distilled analysis report, or both. This lets the logger
  keep writing compact binary pages while the reader does the human-level work.
*/

#include "Arduino.h"
#include "SFLASH.h"

const bool PRINT_RAW_ROWS = true;
const bool PRINT_ANALYSIS_SUMMARY = true;

const uint8_t LOG_MAGIC_0 = 0x42;      // 'B'
const uint8_t LOG_MAGIC_1 = 0x4C;      // 'L'
const uint8_t LOG_VERSION = 1;
const uint8_t PAGE_TYPE_SESSION = 1;
const uint8_t PAGE_TYPE_DEVICE = 2;
const uint8_t PAGE_TYPE_CHECKPOINT = 3;
const uint8_t LOG_MARKER = 0xB5;
const uint16_t LOG_PAGE_SIZE = 256;
const uint16_t HEADER_SIZE = 23;
const uint16_t DEVICE_RECORD_SIZE = 23;
const uint16_t CRC_MSB_BYTE = 253;
const uint16_t CRC_LSB_BYTE = 254;
const uint16_t MARKER_BYTE = 255;
const uint16_t MAX_ANALYZED_DEVICES = 512;
const uint16_t MAX_ROLLUPS = 288;      // 48 hours at one rollup per 10 minutes
const uint16_t ROLLUP_SECONDS = 600;

struct DeviceAnalysis {
    uint8_t address[6];
    uint8_t addressType;
    uint8_t flags;
    uint16_t mfgId;
    uint32_t uuidHash;
    uint16_t nameHash;
    uint32_t firstBatch;
    uint32_t lastBatch;
    uint16_t intervalsSeen;
    int32_t smoothRssiSum;
    int8_t bestRssi;
    int8_t lastRssi;
    bool used;
};

struct RollupAnalysis {
    uint32_t startBatch;
    uint16_t intervals;
    uint16_t activeSum;
    uint8_t peakActive;
    uint16_t strong;
    uint16_t medium;
    uint16_t weak;
    uint16_t apple;
    uint16_t st;
    uint16_t google;
    uint16_t named;
    uint16_t publicAddress;
    uint16_t randomStatic;
};

uint8_t flashPage[LOG_PAGE_SIZE];
uint8_t mid;
uint16_t did;
uint32_t pageCount;
uint16_t batchIntervalSeconds = 10;
uint32_t sessionStartSeconds = 0;
uint16_t analyzedDeviceCount = 0;
uint16_t overflowedAnalysisRecords = 0;
uint16_t rollupCount = 0;
uint32_t lastRollupBatch = 0xFFFFFFFF;
uint32_t totalIntervals = 0;
uint32_t totalActiveSum = 0;
uint8_t totalPeakActive = 0;
uint16_t firstValidYear = 0;
uint8_t firstValidMonth = 0;
uint8_t firstValidDay = 0;
uint8_t firstValidHour = 0;
uint8_t firstValidMinute = 0;
uint8_t firstValidSecond = 0;

DeviceAnalysis devices[MAX_ANALYZED_DEVICES];
RollupAnalysis rollups[MAX_ROLLUPS];

bool pageBufferIsErased();
bool pageIsValid(uint32_t expectedPage);
bool addressMatches(const DeviceAnalysis &device, uint16_t offset);
uint16_t calculateCRC16(const uint8_t *data, uint16_t length);
uint16_t getU16(uint16_t offset);
int16_t getI16(uint16_t offset);
uint32_t getU32(uint16_t offset);
uint16_t findOrAddDevice(uint16_t offset);
void printSessionPage(uint32_t physicalPage);
void printDevicePage(uint32_t physicalPage);
void printCheckpointPage(uint32_t physicalPage);
void analyzeDevicePage();
void updateRollup(uint32_t batch, uint8_t activeCount);
void updateDeviceAnalysis(uint16_t offset, uint32_t batch);
void printAnalysisSummary();
void printRollupSummary();
void printRollupValue(const char label[], uint16_t value);
void printManufacturerSummary();
void printPersistenceSummary();
void printDeviceIdentityTable();
void printAddress(uint16_t offset);
void printStoredAddress(const uint8_t *address);
void printTwoDigits(uint8_t value);
void printBatchTime(uint32_t batch);
void printDateTimeFromFields(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);
const char *manufacturerName(uint16_t mfgId);
const char *proximityClass(int8_t rssi);
const char *addressTypeName(uint8_t addressType);

void setup()
{
    Serial.begin(115200);
    while (!Serial) {
    }

    Serial.println();
    Serial.println("BLE log dump");
    Serial.println("------------");

    if (!SFLASH.begin() || !SFLASH.identify(mid, did)) {
        Serial.println("QSPI flash initialization failed.");
        SFLASH.end();
        return;
    }

    Serial.print("MID=0x"); Serial.println(mid, HEX);
    Serial.print("DID=0x"); Serial.println(did, HEX);
    Serial.print("PAGESIZE="); Serial.println(SFLASH.pageSize());
    Serial.print("LENGTH="); Serial.println(SFLASH.length());

    if (SFLASH.pageSize() != LOG_PAGE_SIZE) {
        Serial.println("Unsupported page size.");
        SFLASH.end();
        return;
    }

    pageCount = SFLASH.length() / SFLASH.pageSize();
    if (pageCount > 0xFFFF) {
        pageCount = 0xFFFF;
    }

    for (uint32_t page = 0; page < pageCount; page++) {
        if (!SFLASH.read(page * LOG_PAGE_SIZE, flashPage, sizeof(flashPage))) {
            Serial.print("Flash read failed at page ");
            Serial.println(page);
            break;
        }

        if (pageBufferIsErased()) {
            Serial.print("Reached first erased page: ");
            Serial.println(page);
            break;
        }

        if (!pageIsValid(page)) {
            Serial.print("Invalid BLE log page at ");
            Serial.println(page);
            break;
        }

        switch (flashPage[3]) {
            case PAGE_TYPE_SESSION:
                printSessionPage(page);
                break;

            case PAGE_TYPE_DEVICE:
                printDevicePage(page);
                analyzeDevicePage();
                break;

            case PAGE_TYPE_CHECKPOINT:
                printCheckpointPage(page);
                break;

            default:
                Serial.print("Unknown page type at ");
                Serial.println(page);
                break;
        }
    }

    if (PRINT_ANALYSIS_SUMMARY) {
        printAnalysisSummary();
    }

    SFLASH.end();
    Serial.println("Dump complete.");
}

void loop()
{
}

bool pageBufferIsErased()
{
    for (uint16_t i = 0; i < sizeof(flashPage); i++) {
        if (flashPage[i] != 0xFF) {
            return false;
        }
    }

    return true;
}

bool pageIsValid(uint32_t expectedPage)
{
    uint16_t storedPage = getU16(4);
    uint16_t storedCRC = getU16(CRC_MSB_BYTE);
    uint16_t calculatedCRC = calculateCRC16(flashPage, CRC_MSB_BYTE);

    return flashPage[0] == LOG_MAGIC_0 &&
           flashPage[1] == LOG_MAGIC_1 &&
           flashPage[2] == LOG_VERSION &&
           flashPage[MARKER_BYTE] == LOG_MARKER &&
           storedPage == (uint16_t)expectedPage &&
           storedCRC == calculatedCRC;
}

void printSessionPage(uint32_t physicalPage)
{
    firstValidYear = getU16(23);
    firstValidMonth = flashPage[25];
    firstValidDay = flashPage[26];
    firstValidHour = flashPage[27];
    firstValidMinute = flashPage[28];
    firstValidSecond = flashPage[29];
    sessionStartSeconds = ((uint32_t)firstValidHour * 3600UL) + ((uint32_t)firstValidMinute * 60UL) + firstValidSecond;
    batchIntervalSeconds = getU16(30);

    if (!PRINT_RAW_ROWS) {
        return;
    }

    Serial.print("# SESSION page=");
    Serial.print(physicalPage);
    Serial.print(",seq=");
    Serial.print(getU32(6));
    Serial.print(",session=");
    Serial.print(getU32(10));
    Serial.print(",start=");
    printDateTimeFromFields(firstValidYear, firstValidMonth, firstValidDay, firstValidHour, firstValidMinute, firstValidSecond);
    Serial.print(",batch_interval_s=");
    Serial.print(batchIntervalSeconds);
    Serial.print(",env_interval_s=");
    Serial.print(getU16(32));
    Serial.print(",top_n=");
    Serial.println(flashPage[34]);
}

void printDevicePage(uint32_t physicalPage)
{
    uint8_t recordCount = flashPage[20];

    if (!PRINT_RAW_ROWS) {
        return;
    }

    Serial.print("# DEVICE page=");
    Serial.print(physicalPage);
    Serial.print(",seq=");
    Serial.print(getU32(6));
    Serial.print(",batch=");
    Serial.print(getU32(14));
    Serial.print(",batch_page=");
    Serial.print(flashPage[18] + 1);
    Serial.print("/");
    Serial.print(flashPage[19]);
    Serial.print(",records=");
    Serial.print(recordCount);
    Serial.print(",active=");
    Serial.print(flashPage[21]);
    Serial.print(",overflow=");
    Serial.println((flashPage[21] > 48) ? (flashPage[21] - 48) : 0);

    Serial.println("batch,address,type,flags,smooth_rssi,best_rssi,last_rssi,age_s,seen_delta,mfg,uuid_hash,name_hash");

    for (uint8_t slot = 0; slot < recordCount; slot++) {
        uint16_t offset = HEADER_SIZE + (slot * DEVICE_RECORD_SIZE);

        Serial.print(getU32(14));
        Serial.print(",");
        printAddress(offset);
        Serial.print(",");
        Serial.print(addressTypeName(flashPage[offset + 6]));
        Serial.print(",");
        Serial.print((flashPage[offset + 7] & 0x01) ? "C" : "-");
        Serial.print((flashPage[offset + 7] & 0x02) ? "A" : "-");
        Serial.print((flashPage[offset + 7] & 0x04) ? "R" : "-");
        Serial.print(",");
        Serial.print((int8_t)flashPage[offset + 8]);
        Serial.print(",");
        Serial.print((int8_t)flashPage[offset + 9]);
        Serial.print(",");
        Serial.print((int8_t)flashPage[offset + 10]);
        Serial.print(",");
        Serial.print(flashPage[offset + 11]);
        Serial.print(",");
        Serial.print(getU16(offset + 12));
        Serial.print(",0x");
        if (getU16(offset + 14) < 0x1000) Serial.print("0");
        if (getU16(offset + 14) < 0x0100) Serial.print("0");
        if (getU16(offset + 14) < 0x0010) Serial.print("0");
        Serial.print(getU16(offset + 14), HEX);
        Serial.print(",0x");
        Serial.print(getU32(offset + 16), HEX);
        Serial.print(",0x");
        Serial.println(getU16(offset + 20), HEX);
    }
}

void printCheckpointPage(uint32_t physicalPage)
{
    if (!PRINT_RAW_ROWS) {
        return;
    }

    Serial.print("# CHECKPOINT page=");
    Serial.print(physicalPage);
    Serial.print(",seq=");
    Serial.print(getU32(6));
    Serial.print(",batch=");
    Serial.print(getU32(14));
    Serial.print(",time=");
    printDateTimeFromFields(getU16(23), flashPage[25], flashPage[26], flashPage[27], flashPage[28], flashPage[29]);
    Serial.print(",battery=");
    Serial.print(getU16(30));
    Serial.print("mV,mcu=");
    Serial.print(getI16(32) / 100.0f, 2);
    Serial.print("C,lis2dw12=");
    Serial.print(getI16(34) / 100.0f, 2);
    Serial.println("C");
}

void analyzeDevicePage()
{
    uint32_t batch = getU32(14);
    uint8_t batchPageIndex = flashPage[18];
    uint8_t activeCount = flashPage[21];
    uint8_t recordCount = flashPage[20];

    if (batchPageIndex == 0) {
        updateRollup(batch, activeCount);
    }

    for (uint8_t slot = 0; slot < recordCount; slot++) {
        uint16_t offset = HEADER_SIZE + (slot * DEVICE_RECORD_SIZE);
        updateDeviceAnalysis(offset, batch);
    }
}

void updateRollup(uint32_t batch, uint8_t activeCount)
{
    uint16_t batchesPerRollup = ROLLUP_SECONDS / batchIntervalSeconds;
    if (batchesPerRollup == 0) {
        batchesPerRollup = 1;
    }

    uint32_t rollupBatch = batch / batchesPerRollup;

    totalIntervals++;
    totalActiveSum += activeCount;
    if (activeCount > totalPeakActive) {
        totalPeakActive = activeCount;
    }

    if (rollupCount >= MAX_ROLLUPS) {
        return;
    }

    if (rollupCount == 0 || rollupBatch != lastRollupBatch) {
        lastRollupBatch = rollupBatch;
        rollups[rollupCount].startBatch = rollupBatch * batchesPerRollup;
        rollupCount++;
    }

    RollupAnalysis &rollup = rollups[rollupCount - 1];
    rollup.intervals++;
    rollup.activeSum += activeCount;
    if (activeCount > rollup.peakActive) {
        rollup.peakActive = activeCount;
    }
}

void updateDeviceAnalysis(uint16_t offset, uint32_t batch)
{
    uint16_t index = findOrAddDevice(offset);
    if (index >= MAX_ANALYZED_DEVICES) {
        overflowedAnalysisRecords++;
        return;
    }

    DeviceAnalysis &device = devices[index];
    int8_t smoothRssi = (int8_t)flashPage[offset + 8];
    int8_t bestRssi = (int8_t)flashPage[offset + 9];

    if (device.lastBatch != batch) {
        device.intervalsSeen++;
        device.smoothRssiSum += smoothRssi;
    }

    device.flags |= flashPage[offset + 7];
    device.mfgId = getU16(offset + 14);
    device.uuidHash = getU32(offset + 16);
    device.nameHash = getU16(offset + 20);
    device.lastBatch = batch;
    device.lastRssi = (int8_t)flashPage[offset + 10];

    if (bestRssi > device.bestRssi) {
        device.bestRssi = bestRssi;
    }

    if (rollupCount > 0) {
        RollupAnalysis &rollup = rollups[rollupCount - 1];

        if (smoothRssi >= -60) {
            rollup.strong++;
        } else if (smoothRssi >= -80) {
            rollup.medium++;
        } else {
            rollup.weak++;
        }

        if (device.mfgId == 0x004C) {
            rollup.apple++;
        } else if (device.mfgId == 0x0030) {
            rollup.st++;
        } else if (device.mfgId == 0x00E0) {
            rollup.google++;
        }

        if (device.nameHash != 0) {
            rollup.named++;
        }

        if (device.addressType == 0) {
            rollup.publicAddress++;
        } else if (device.addressType == 2) {
            rollup.randomStatic++;
        }
    }
}

uint16_t findOrAddDevice(uint16_t offset)
{
    for (uint16_t i = 0; i < MAX_ANALYZED_DEVICES; i++) {
        if (devices[i].used && addressMatches(devices[i], offset)) {
            return i;
        }
    }

    if (analyzedDeviceCount >= MAX_ANALYZED_DEVICES) {
        return MAX_ANALYZED_DEVICES;
    }

    DeviceAnalysis &device = devices[analyzedDeviceCount];
    for (uint8_t i = 0; i < 6; i++) {
        device.address[i] = flashPage[offset + i];
    }
    device.addressType = flashPage[offset + 6];
    device.flags = flashPage[offset + 7];
    device.mfgId = getU16(offset + 14);
    device.uuidHash = getU32(offset + 16);
    device.nameHash = getU16(offset + 20);
    device.firstBatch = getU32(14);
    device.lastBatch = 0xFFFFFFFF;
    device.intervalsSeen = 0;
    device.smoothRssiSum = 0;
    device.bestRssi = -127;
    device.lastRssi = -127;
    device.used = true;

    analyzedDeviceCount++;
    return analyzedDeviceCount - 1;
}

bool addressMatches(const DeviceAnalysis &device, uint16_t offset)
{
    if (device.addressType != flashPage[offset + 6]) {
        return false;
    }

    for (uint8_t i = 0; i < 6; i++) {
        if (device.address[i] != flashPage[offset + i]) {
            return false;
        }
    }

    return true;
}

void printAnalysisSummary()
{
    Serial.println();
    Serial.println("BLE discovery analysis");
    Serial.println("----------------------");
    Serial.print("Intervals analyzed: ");
    Serial.println(totalIntervals);
    Serial.print("Unique addresses:   ");
    Serial.println(analyzedDeviceCount);
    Serial.print("Average devices:    ");
    Serial.println(totalIntervals ? (float)totalActiveSum / totalIntervals : 0.0f, 1);
    Serial.print("Peak devices:       ");
    Serial.println(totalPeakActive);
    if (overflowedAnalysisRecords) {
        Serial.print("Analysis overflow:  ");
        Serial.print(overflowedAnalysisRecords);
        Serial.println(" records beyond local table");
    }

    printRollupSummary();
    printManufacturerSummary();
    printPersistenceSummary();
    printDeviceIdentityTable();
}

void printRollupSummary()
{
    Serial.println();
    Serial.println("Ten-minute rollups");

    for (uint16_t i = 0; i < rollupCount; i++) {
        RollupAnalysis &rollup = rollups[i];
        Serial.println();
        Serial.print("Rollup ");
        Serial.print(i + 1);
        Serial.print(": ");
        printBatchTime(rollup.startBatch);
        Serial.println();
        Serial.print("  intervals:      ");
        Serial.println(rollup.intervals);
        Serial.print("  avg active:     ");
        Serial.println(rollup.intervals ? (float)rollup.activeSum / rollup.intervals : 0.0f, 1);
        Serial.print("  peak active:    ");
        Serial.println(rollup.peakActive);
        printRollupValue("strong obs", rollup.strong);
        printRollupValue("medium obs", rollup.medium);
        printRollupValue("weak obs", rollup.weak);
        printRollupValue("Apple obs", rollup.apple);
        printRollupValue("ST obs", rollup.st);
        printRollupValue("Google obs", rollup.google);
        printRollupValue("named obs", rollup.named);
        printRollupValue("public addr obs", rollup.publicAddress);
        printRollupValue("random static obs", rollup.randomStatic);
    }
}

void printRollupValue(const char label[], uint16_t value)
{
    if (value == 0) {
        return;
    }

    Serial.print("  ");
    Serial.print(label);
    Serial.print(":");

    uint8_t labelLength = strlen(label);
    for (uint8_t i = labelLength; i < 18; i++) {
        Serial.print(" ");
    }

    Serial.println(value);
}

void printManufacturerSummary()
{
    uint16_t apple = 0;
    uint16_t st = 0;
    uint16_t google = 0;
    uint16_t microsoft = 0;
    uint16_t nordic = 0;
    uint16_t ti = 0;
    uint16_t unknown = 0;
    uint16_t other = 0;

    for (uint16_t i = 0; i < analyzedDeviceCount; i++) {
        switch (devices[i].mfgId) {
            case 0x004C: apple++; break;
            case 0x0030: st++; break;
            case 0x00E0: google++; break;
            case 0x0006: microsoft++; break;
            case 0x0059: nordic++; break;
            case 0x000D: ti++; break;
            case 0x0000:
            case 0xFFFF: unknown++; break;
            default: other++; break;
        }
    }

    Serial.println();
    Serial.println("Manufacturer summary");
    Serial.print("Apple:        "); Serial.println(apple);
    Serial.print("ST:           "); Serial.println(st);
    Serial.print("Google:       "); Serial.println(google);
    Serial.print("Microsoft:    "); Serial.println(microsoft);
    Serial.print("Nordic:       "); Serial.println(nordic);
    Serial.print("TI:           "); Serial.println(ti);
    Serial.print("Other known:  "); Serial.println(other);
    Serial.print("Unknown:      "); Serial.println(unknown);
}

void printPersistenceSummary()
{
    uint16_t persistent = 0;
    uint16_t occasional = 0;
    uint16_t transient = 0;

    for (uint16_t i = 0; i < analyzedDeviceCount; i++) {
        float percent = totalIntervals ? (100.0f * devices[i].intervalsSeen) / totalIntervals : 0.0f;

        if (percent >= 80.0f) {
            persistent++;
        } else if (percent >= 20.0f) {
            occasional++;
        } else {
            transient++;
        }
    }

    Serial.println();
    Serial.println("Persistence summary");
    Serial.print("Persistent >=80%: ");
    Serial.println(persistent);
    Serial.print("Occasional 20-80%: ");
    Serial.println(occasional);
    Serial.print("Transient <20%: ");
    Serial.println(transient);
}

void printDeviceIdentityTable()
{
    Serial.println();
    Serial.println("Device identity table");
    Serial.println("address,type,flags,mfg,first,last,seen_percent,best,avg,last,proximity,name_hash,uuid_hash");

    for (uint16_t i = 0; i < analyzedDeviceCount; i++) {
        DeviceAnalysis &device = devices[i];
        float seenPercent = totalIntervals ? (100.0f * device.intervalsSeen) / totalIntervals : 0.0f;
        float averageRssi = device.intervalsSeen ? (float)device.smoothRssiSum / device.intervalsSeen : 0.0f;

        printStoredAddress(device.address);
        Serial.print(",");
        Serial.print(addressTypeName(device.addressType));
        Serial.print(",");
        Serial.print((device.flags & 0x01) ? "C" : "-");
        Serial.print((device.flags & 0x02) ? "A" : "-");
        Serial.print((device.flags & 0x04) ? "R" : "-");
        Serial.print(",");
        Serial.print(manufacturerName(device.mfgId));
        Serial.print(",");
        printBatchTime(device.firstBatch);
        Serial.print(",");
        printBatchTime(device.lastBatch);
        Serial.print(",");
        Serial.print(seenPercent, 1);
        Serial.print(",");
        Serial.print(device.bestRssi);
        Serial.print(",");
        Serial.print(averageRssi, 1);
        Serial.print(",");
        Serial.print(device.lastRssi);
        Serial.print(",");
        Serial.print(proximityClass(device.bestRssi));
        Serial.print(",0x");
        Serial.print(device.nameHash, HEX);
        Serial.print(",0x");
        Serial.println(device.uuidHash, HEX);
    }
}

void printAddress(uint16_t offset)
{
    for (uint8_t i = 0; i < 6; i++) {
        if (i) Serial.print(":");
        if (flashPage[offset + i] < 0x10) Serial.print("0");
        Serial.print(flashPage[offset + i], HEX);
    }
}

void printStoredAddress(const uint8_t *address)
{
    for (uint8_t i = 0; i < 6; i++) {
        if (i) Serial.print(":");
        if (address[i] < 0x10) Serial.print("0");
        Serial.print(address[i], HEX);
    }
}

void printTwoDigits(uint8_t value)
{
    if (value < 10) {
        Serial.print("0");
    }
    Serial.print(value);
}

void printBatchTime(uint32_t batch)
{
    uint32_t seconds = sessionStartSeconds + (batch * batchIntervalSeconds);
    uint8_t hour = (seconds / 3600UL) % 24;
    uint8_t minute = (seconds / 60UL) % 60;
    uint8_t second = seconds % 60;

    printTwoDigits(hour);
    Serial.print(":");
    printTwoDigits(minute);
    Serial.print(":");
    printTwoDigits(second);
}

void printDateTimeFromFields(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second)
{
    Serial.print(year);
    Serial.print("-");
    printTwoDigits(month);
    Serial.print("-");
    printTwoDigits(day);
    Serial.print(" ");
    printTwoDigits(hour);
    Serial.print(":");
    printTwoDigits(minute);
    Serial.print(":");
    printTwoDigits(second);
}

const char *manufacturerName(uint16_t mfgId)
{
    switch (mfgId) {
        case 0x004C: return "Apple";
        case 0x0030: return "ST";
        case 0x00E0: return "Google";
        case 0x0006: return "Microsoft";
        case 0x0059: return "Nordic";
        case 0x000D: return "TI";
        case 0x0000: return "unknown";
        case 0xFFFF: return "unknown";
        default: return "other";
    }
}

const char *proximityClass(int8_t rssi)
{
    if (rssi >= -60) {
        return "strong";
    }

    if (rssi >= -80) {
        return "medium";
    }

    return "weak";
}

const char *addressTypeName(uint8_t addressType)
{
    switch (addressType) {
        case 0: return "public";
        case 1: return "random";
        case 2: return "random_static";
        case 3: return "rpa";
        case 4: return "nrpa";
        default: return "unknown";
    }
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

uint16_t getU16(uint16_t offset)
{
    return ((uint16_t)flashPage[offset] << 8) | flashPage[offset + 1];
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
