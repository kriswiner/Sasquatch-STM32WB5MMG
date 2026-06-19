/*
  Sasquatch STM32WB5MMG BLE scanner, based on the working Scan example from:

    TleraCorp STM32WB Arduino core 0.1.7
    libraries/BLE/examples/Scan/Scan.ino

  This version keeps the same STM32WB BLE API calls as the stock example, but
  adds comments and prints each advertising report in a more readable format.

  Before running BLE sketches on a fresh STM32WB board, the wireless stack must
  be installed with the FWUpdate example and can be checked with FWInfo.
*/

#include "Arduino.h"  // Provides the standard Arduino sketch API: setup(), loop(), Serial, types, etc.
#include "BLE.h"      // Provides the STM32WB BLE classes used for scanning and decoding advertisements.
#include "STM32WB.h"  // Provides STM32WB board/core support used by this Arduino core.

// Print one byte as two hexadecimal characters.
// The stock example prints each nibble separately; this helper keeps that logic readable.
void printHexByte(uint8_t value)
{
    // Print a leading zero for values below 0x10 so every byte is two characters.
    if (value < 16) {
        Serial.print("0");
    }

    // Print the byte itself in hexadecimal.
    Serial.print(value, HEX);
}

// Print a byte array as continuous hexadecimal text.
void printHexBytes(const uint8_t data[], int length)
{
    // Walk over every byte in the buffer.
    for (int index = 0; index < length; index++) {
        // Print this byte in two-character hex form.
        printHexByte(data[index]);
    }
}

void setup()
{
    // Holds printable text such as a BLE address or UUID after conversion.
    char string[64];

    // Holds a decoded advertised local name, when the advertiser includes one.
    char name[32];

    // Holds service data bytes decoded from an advertisement.
    uint8_t service_data[32];

    // Holds manufacturer-specific data bytes decoded from an advertisement.
    uint8_t manufacturer_data[32];

    // Holds the BLE address of the remote device that sent the advertising report.
    BLEAddress address;

    // Holds flags describing the advertising report, such as connectable/response.
    BLEScanEvent event;

    // Holds the parsed advertising data for the current report.
    BLEAdvertisingData data;

    // Holds a single UUID when decoding service data or beacon data.
    BLEUuid uuid;

    // Holds multiple advertised service UUIDs, up to 16 entries.
    BLEUuid uuid_table[16];

    // Holds the received signal strength, in dBm, for the current advertising report.
    int rssi;

    // Holds the advertiser's transmit power level, if present in the advertisement.
    int txPower;

    // Used as a loop counter while printing lists of UUIDs or bytes.
    int index;

    // Holds the number of service UUIDs returned by getServiceUuids().
    int count;

    // Holds the number of bytes returned for service or manufacturer data.
    int length;

    // Holds the decoded beacon measured power value so it does not overwrite report RSSI.
    int beaconMeasuredPower;

    // Holds the advertiser's preferred minimum and maximum connection interval, if present.
    uint16_t interval_min, interval_max;

    // Holds iBeacon-style major and minor values, if the advertisement is a beacon.
    uint16_t major, minor;

    // Count reports so the Serial Monitor output is easier to follow.
    unsigned long reportCount = 0;

    // Open the USB serial port at the same baud rate as the stock scan example.
    Serial.begin(9600);

    // Wait until the Serial Monitor is open so the first messages are not missed.
    while (!Serial) {
    }

    // Start the STM32WB BLE stack.
    BLE.begin();

    // Set the local BLE name for this Sasquatch while it is using the BLE stack.
    BLE.setLocalName("Sasquatch");

    // Print a clean heading.
    Serial.println();
    Serial.println();
    Serial.println("Sasquatch STM32WB5MMG BLE scanner");
    Serial.println("----------------------------------");

    // Convert this board's BLE address to text.
    BLE.address().toString(string, sizeof(string) - 1);

    // Show the scanner's own BLE address before displaying remote devices.
    Serial.print("Scanner address: ");
    Serial.println(string);
    Serial.println();

    // Start scanning and ignore duplicate reports from the same advertiser.
    BLE.scan(BLE_SCAN_MODE_WITHOUT_DUPLICATES);

    // Let the user know the infinite scan loop is active.
    Serial.println("Scanning for nearby BLE advertisers...");
    Serial.println();

    // The stock example scans forever inside setup(), so loop() is intentionally unused.
    while (1) {
        // BLE.report() returns true when a new advertising report has been received.
        // It fills in address, RSSI, event flags, and decoded advertising data.
        if (BLE.report(address, rssi, event, data)) {
            // Count this report.
            reportCount++;

            // Convert the remote advertiser's address to printable text.
            address.toString(string, sizeof(string) - 1);

            // Print a separator and report number for readability.
            Serial.print("Report #");
            Serial.println(reportCount);

            // Print the advertiser address and address type.
            Serial.print("  Address:          ");
            Serial.print(string);
            Serial.print("  (type ");
            Serial.print(address.type());
            Serial.println(")");

            // Print received signal strength. Less negative means a stronger signal.
            Serial.print("  RSSI:             ");
            Serial.print(rssi);
            Serial.println(" dBm");

            // Decode the event flags that describe what kind of BLE report this is.
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

            // Print the advertised transmit power level, if the device included one.
            if (data.getTxPowerLevel(txPower)) {
                Serial.print("  TX power:         ");
                Serial.print(txPower);
                Serial.println(" dBm");
            }

            // Print the preferred connection interval, if the device advertised it.
            if (data.getConnectionInterval(interval_min, interval_max)) {
                Serial.print("  Conn interval:    ");
                Serial.print(interval_min);
                Serial.print(" to ");
                Serial.print(interval_max);
                Serial.println(" BLE units");
            }

            // Print the advertised local name, if present.
            if (data.getLocalName(name, sizeof(name))) {
                Serial.print("  Local name:       ");
                Serial.println(name);
            }

            // Print any advertised service UUIDs.
            if (data.getServiceUuids(uuid_table, (sizeof(uuid_table) / sizeof(uuid_table[0])), count)) {
                Serial.print("  Service UUIDs:    ");

                // Walk over each UUID returned by the parser.
                for (index = 0; index < count; index++) {
                    // Convert this UUID to printable text.
                    uuid_table[index].toString(string, sizeof(string) - 1);

                    // Separate multiple UUIDs with a comma.
                    if (index) {
                        Serial.print(", ");
                    }

                    // Print the UUID.
                    Serial.print(string);
                }

                Serial.println();
            }

            // Print service data, if the advertisement contains it.
            if (data.getServiceData(uuid, service_data, sizeof(service_data), length)) {
                // Convert the service-data UUID to printable text.
                uuid.toString(string, sizeof(string) - 1);

                // Print the UUID associated with the service-data payload.
                Serial.print("  Service data UUID:");
                Serial.println(string);

                // Print the service-data bytes, if any were included.
                Serial.print("  Service data:     ");
                if (length) {
                    printHexBytes(service_data, length);
                } else {
                    Serial.print("(empty)");
                }
                Serial.println();
            }

            // Print manufacturer data, if the advertisement contains it.
            if (data.getManufacturerData(manufacturer_data, sizeof(manufacturer_data), length)) {
                Serial.print("  Manufacturer ID:  ");

                // BLE manufacturer IDs are little-endian in the advertising packet.
                if (length >= 2) {
                    printHexByte(manufacturer_data[1]);
                    printHexByte(manufacturer_data[0]);
                } else {
                    Serial.print("(missing)");
                }
                Serial.println();

                // Print the remaining manufacturer payload bytes after the company ID.
                Serial.print("  Manufacturer data:");
                if (length > 2) {
                    printHexBytes(&manufacturer_data[2], length - 2);
                } else {
                    Serial.print("(none)");
                }
                Serial.println();
            }

            // Print iBeacon-style data, if the parser recognizes a beacon payload.
            if (data.getBeacon(uuid, major, minor, beaconMeasuredPower)) {
                // Convert the beacon UUID to printable text.
                uuid.toString(string, sizeof(string) - 1);

                // Print the beacon identity and calibrated measured power.
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

            // Leave a blank line before the next advertising report.
            Serial.println();
        }
    }
}

void loop()
{
    // The scanner never reaches loop() because the stock example scans forever in setup().
}
