/*
    This class handles all the Barometer related stuffs

    author: Isak H, Aurora Å/
*/
#ifndef __BAROMETER__
#define __BAROMETER__

#include<stage_recognition.h>
#include<Adafruit_BMP280.h>

class Barometer{
    public:
        Barometer();
        void init();
        void getData(telemetry* tel);
        float calibrate();
    
    private:
        Adafruit_BMP280 bmp;
};

#endif