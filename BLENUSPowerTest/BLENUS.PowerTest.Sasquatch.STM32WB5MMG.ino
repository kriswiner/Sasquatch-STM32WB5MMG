#include "Arduino.h"
#include "STM32WB.h"
#include "BLE.h"

BLEUart SerialBLE(BLE_UART_PROTOCOL_NORDIC);

const uint32_t BLE_PERIOD_MS       = 60000UL;  // one BLE opportunity per minute
const uint32_t BLE_ADVERTISE_MS    = 5000UL;  // advertise for 30 seconds
const uint32_t BLE_CONNECTED_MS    = 30000UL;  // stay connected briefly so the phone can query
const uint32_t BLE_SERVICE_STOP_MS = 100UL;    // wake often enough to service BLE UART RX

void setup()
{
    Serial.begin(9600);

    BLE.begin();
    BLE.setLocalName("STM32WB");
    BLE.setServiceUuid(SerialBLE.uuid());

    BLE.addService(SerialBLE);

    // Do not advertise continuously from setup.
    BLE.stopAdvertise();
}


void loop()
{
    /*
     * Keep this diagnostic sketch intentionally simple:
     *   1) advertise for a bounded window,
     *   2) if the phone connects, accept query characters for a bounded window,
     *   3) explicitly stop advertising/disconnect,
     *   4) sleep until the next opportunity.
     *
     * The short STOP slices during BLE activity keep current low while still
     * waking often enough to service the BLE UART.
     */
    BLE.advertise();
    serviceAdvertiseWindow();

    BLE.disconnect();
    BLE.stopAdvertise();

    STM32WB.stop(BLE_PERIOD_MS);
}


void serviceAdvertiseWindow()
{
    const uint16_t advertiseSlices = BLE_ADVERTISE_MS / BLE_SERVICE_STOP_MS;

    for(uint16_t i = 0; i < advertiseSlices; i++) {
        if(BLE.connected()) {
            serviceConnectedWindow();
            return;
        }

        STM32WB.stop(BLE_SERVICE_STOP_MS);
    }
}


void serviceConnectedWindow()
{
    const uint16_t connectedSlices = BLE_CONNECTED_MS / BLE_SERVICE_STOP_MS;

    for(uint16_t i = 0; i < connectedSlices; i++) {
        if(!BLE.connected()) {
            return;
        }

        serviceBLERX();
        STM32WB.stop(BLE_SERVICE_STOP_MS);
    }
}


void sendBLEReport()
{
    SerialBLE.print("Temperature = ");
    SerialBLE.print(STM32WB.readTemperature());
    SerialBLE.println(" *C");

    SerialBLE.print("Battery = ");
    SerialBLE.print(STM32WB.readBattery());
    SerialBLE.println(" V");

    SerialBLE.println();
}


void serviceBLERX()
{
    int c;

    while((c = SerialBLE.read()) >= 0) {
        if(c == '\n') Serial.write('\r');
        Serial.write(c);

        // Query-driven report: send '?' or 'r' from the phone to request data.
        if(c == '?' || c == 'r' || c == 'R') {
            sendBLEReport();
        }
    }
}
