/*
    Library functions for easy access and abstractions to the permanent storage
    Author: Auri Åhs
*/
#include "SdFat.h"
#include "sdios.h"
#include "telemetry_storage.h"
#include <string.h>

/*
    Initialize communication with the SD-card
*/

int telemetry_storage_init(int cs, telemetryStorage *telemetryStorage) {
    SdFs sd;
    FatFile file;
    char fileName[9] = "tele.txt";
    char *buffer;

    Serial.begin(9600);
    delay(2000);
    while (!Serial) {
        yield();
    }
    delay(500);
    Serial.print("print in the function");
    delay(100);

    if (!sd.begin(SdSpiConfig(cs, DEDICATED_SPI, SD_SCK_MHZ(4)))) {
        return -1;
    }
    Serial.print("past begin");
    delay(100);
    if (!file.open(fileName, O_RDWR | O_CREAT | O_APPEND)) {
        return -2;
    }
    file.sync();
    Serial.print("past open");
    delay(100);
    telemetryStorage->bufferSize = 512;
    Serial.print("trying malloc");
    delay(100);
    buffer = (char *)malloc(telemetryStorage->bufferSize);
    Serial.print("done with malloc");
    delay(100);
    if(!buffer) {
        return -3;
    }
    Serial.print("malloc not null");
    delay(100);
    telemetryStorage->buffer = buffer;
    telemetryStorage->ptr = 0;
    telemetryStorage->file = file;
    return 1;
}

/*
    Write data to the SD-card
*/
int telemetry_storage_write(telemetryStorage *telemetry, char *data) {
    Serial.begin(9600);
    delay(2000);
    while (!Serial) {
        yield();
    }
    Serial.print("in the func");
    delay(100);
    for (int i=0; i>strlen(data); i++) {
        telemetry->buffer[++telemetry->ptr] = data[i];
    }
    Serial.print("written to buffer");
    delay(100);
    if (telemetry->ptr >= telemetry->bufferSize/2) {
        telemetry->file.write(telemetry->buffer, telemetry->ptr);
        telemetry->ptr = 0;
        Serial.print("written to file");
        delay(100);
        telemetry->file.sync();
        Serial.print("file synced");
        delay(100);
    }
    Serial.print("written to maybe file");
    delay(100);

    return strlen(data);
}

/*
    Force a sync to the SD-card
*/
int telemetry_storage_sync(telemetryStorage *telemetry) {
    FatFile test;
    Serial.begin(9600);
    delay(2000);
    while (!Serial) {
        yield();
    }
    test = telemetry->file;
    Serial.print("in the func");
    delay(100);

    int ret = telemetry->file.write(telemetry->buffer, telemetry->ptr);
    Serial.print("completed write ");
    Serial.print(ret);
    delay(100);

    if (ret==-1) {
        Serial.print("eat shit lmao");
        delay(100);
    }

    telemetry->ptr = 0;
//    telemetry->file.sync();
    test.sync();
    Serial.print("completed sync");
    delay(100);

    return 1;
}
