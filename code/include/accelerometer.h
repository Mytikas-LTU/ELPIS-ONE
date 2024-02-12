/*
This is a class to handle everything the accelerometer does

author: Isak Henningsson
*/


#include <stage_recognition.h>
#include <Adafruit_BNO08x.h>


class Accelerometer {
    public:
        //initialize the accelerometer
        Accelerometer();
        //put all the data into telemetry struct
        void getData(telemetry* data);
        
        private:
            vec3 acc;
            quat rot;
            bool setupError;
            Adafruit_BNO08x sensor;
            sh2_SensorValue sensorValue;
            bool SetReports();
};