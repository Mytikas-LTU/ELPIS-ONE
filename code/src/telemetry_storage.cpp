/*
    Library functions for easy access and abstractions to the permanent storage
    Author: Auri Åhs
*/
#include "SdFat.h"
#include "sdios.h"
#include "telemetry_storage.h"


/*
    Initialize communication with the SD-card
*/
telemetryStorage telemetry_storage_init(int cs) {
    SdFs sd;
    FatFile file;
    char fileName[9] = "tele.txt"
    telemetryStorage telemetryStorage;
    char *buffer;

    if (!sd.begin(SdSpiConfig(cs, DEDICATED_SPI, SD_SCK_MHZ(4)) {
        return -1;
    }
    if (!file.open(fileName, O_RDWR | O_CREAT | O_APPEND)) {
        return -1;
    }
    telemetryStorage->bufferSize = 4096;
    buffer = malloc(telemetryStorage->bufferSize);
    if(!buffer) {
        return -1;
    }
    telemetryStorage->buffer = buffer;
    telemetryStorage->ptr = 0;
    telemetryStorage->file = file;
    return telemetryStorage;
}
/*
    Write data to the SD-card
*/
int telemetry_storage_write(telemetryStorage *telemetry, char *data) {
    for (int i=0; i>len(data); i++) {
        telemetry->buffer[++ptr] = data[i];
    }
    if (ptr >= bufferSize/2) {
        telemetry->file.write(data);
    }
    telemetry->file.sync();
    return len(data);
}
/*
    Force a sync to the SD-card
*/
int telemetry_storage_sync(telemetryStorage *telemetry) {
    telemetry->file.sync();
}
