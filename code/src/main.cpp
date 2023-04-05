/*
    Testing of the SD-card library
    Author: Auri Åhs

*/
#include "SdFat.h"
#include "sdios.h"
#include "telemetry_storage.h"



void setup() {
    struct telemetryStorage *stor = new telemetryStorage();
    char toWrite[4] = "foo";
    char *buffer;
    int retVal;

    Serial.begin(9600);
    delay(2000);
    while (!Serial) {
        yield();
    }
    Serial.print("trying malloc...");
    delay(500);
    stor->bufferSize = 512;
    delay(500);
    Serial.print("trying init...");
    delay(500);
    retVal = telemetry_storage_init(15, stor);
    if (retVal != 1) {
        Serial.print("init is fucked ");
        Serial.print(retVal);
        return;
    }
    Serial.print("trying write...");
    telemetry_storage_write(stor, toWrite);
    Serial.print("trying sync...");
    telemetry_storage_sync(stor);

}
//------------------------------------------------------------------------------
void loop() {

}
