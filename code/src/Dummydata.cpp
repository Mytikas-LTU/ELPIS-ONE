#include<stage_recognition.h>
//#include<basicIO.h>
#include "Dummydata.h"

//yeah, shits empty, fight me
Dummydata::Dummydata(){
    //SIKE, its not empty after all
}

void Dummydata::getData(telemetry* tel){
#if ENABLE_BAROMETER
    Serial.print("dummydata")
    if (telemetry.flight_time < 10000) {
        telemetry.altitude = 0;
        
    } else (telemetry.flight_time < 30000) {
        
    }
#endif
}
