#include <stage_recognition.h>
#include <Adafruit_BNO08x.h>


class Accelerometer {
    public:
        Accelerometer();
        void getData(telemetry* data);
        
        private:
            vec3 acc;
            vec3 rot;
            bool setupError;
            Adafruit_BNO08x sensor(-1);
            sh2_SensorValue sensorValue;
}