#ifndef __STORAGE__
#define __STORAGE__

#include<SD.h>
#include<stage_recognition.h>

class Storage{
    public:
        Storage();
        void init();
        void writeHeader(telemetry* data);
        void write(telemetry* data);

    private:
        int ptr;
        File file;

};

#endif