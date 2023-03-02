/*
    Library functions for easy access and abstractions to the permanent storage
    Author: Auri Åhs
*/
#include "SdFat.h"
#include "sdios.h"

/*
    Initialize communication with the SD-card
    [ in ] cs: the pin that ChipSelect is connected to.
    returns: a FatFile object, to be passed to telemetry_write(), and -1 on failure
*/
FatFile telemetry_storage_init(int cs);

/*
    Write data to the SD-card
    [ in ] data: the data to be written to the card, formatted as a string
    returns: the number of bytes written to the card, and -1 on failure
*/
int telemetry_storage_write(char data);

/*
    Force a sync to the SD-card
*/
int telemetry_storage_sync();
