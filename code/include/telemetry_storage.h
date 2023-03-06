/*
    Library functions for easy access and abstractions to the permanent storage
    Author: Auri Åhs
*/
#include "SdFat.h"
#include "sdios.h"

struct telemetryStorage
{
    char *buffer;
    int ptr;
    int bufferSize;
    FatFile file;
};

/*
    Initialize communication with the SD-card
    [ in ] cs: the pin that ChipSelect is connected to.
    returns: a telemetryStorage struct, to be passed to telemetry_storage_write(), and -1 on failure
*/
telemetryStorage telemetry_storage_init(int cs);

/*
    Write data to the SD-card
    [ in ] data: the data to be written to the card, formatted as a string
    [ in ] telemetry: the telemetryStorage from _init()
    returns: the number of bytes written to the card, and -1 on failure
*/
int telemetry_storage_write(telemetryStorage *telemetry, char data);

/*
    Force a sync to the SD-card
    [ in ] telemetry: the telemetryStorage from _init()
    returns: 1 on success, and -1 on failure
*/
int telemetry_storage_sync(telemetryStorage *telemetry);
