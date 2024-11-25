/*
This is a class to handle everything the accelerometer does

author: Isak Henningsson
*/


#include <stage_recognition.h>
#include <Adafruit_BNO08x.h>

#define ACC_ADDR 74

class Accelerometer {
    public:
        //Is called on object instantiation
        Accelerometer();
        //initialize the accelerometer
        void init();
        //put all the data into telemetry struct
        void getData(telemetry* data);
        int poll();
        
        private:
            Vec3 acc;
            Quat rot;
            bool setupError;
            Adafruit_BNO08x sensor;
            sh2_SensorValue sensorValue;
            bool SetReports();
};
