#include "Arduino.h"
#include "STM32WB.h"
#include "SFLASH.h"

bool SerialDebug = false;

// QSPI flash variables
uint8_t  mid;
uint16_t did;
// create a pointer to the page_number variable to maintain value 
// in BBRAM (SRAM2) during STANDBY for use after wakeup
uint16_t __SECTION_BBRAM_NOINIT page_number;  // stored in SRAM2
uint8_t  flashPage[256];                      // array to hold the data for flash page write
uint32_t block_address, start, end;

// define rgb led pins and colors
#define greenLed 22 // green led active LOW


void setup() {

  Serial.begin(115200);
  while (!Serial) { }

  pinMode(greenLed, OUTPUT); digitalWrite(greenLed, LOW);  // set rgb leds as output, active LOW

  // put your setup code here, to run once:
  // Test QSPI flash memory
  if(SerialDebug) Serial.println("QSPI Flash Check");
  SFLASH.begin();
  SFLASH.identify(mid, did);
  if(SerialDebug) {
  Serial.print("MID = ");       Serial.println(mid, HEX); 
  Serial.print("DID = ");       Serial.println(did, HEX); 
  Serial.print("CAPACITY = ");  Serial.println(SFLASH.capacity());
  Serial.print("BLOCKSIZE = "); Serial.println(SFLASH.blockSize());
  Serial.print("PAGESIZE = ");  Serial.println(SFLASH.pageSize());
  Serial.print("LENGTH = ");    Serial.println(SFLASH.length()); Serial.println(" ");
  }
  
    // erase QSPI flash memory in preparation for data logging
    Serial.println("Wait while flash memory is erased!");
    start = millis();
    for (block_address = 0; block_address < SFLASH.length(); block_address += SFLASH.blockSize()) {
      SFLASH.erase(block_address);
    }
    while (SFLASH.busy()) { }
    end = millis();
    Serial.println("Flash erased!");
    Serial.print(" Erase time = "); Serial.print(((end - start) / 1000.0)); Serial.println(" sec");
    Serial.print((float)SFLASH.length() / ((end - start) / 1000.0));
    Serial.println(" bytes/second"); Serial.println(" ");

  SFLASH.end();

  digitalWrite(greenLed, HIGH);  // turn off green led after flash erased
}

void loop() {
  // put your main code here, to run repeatedly:

}
