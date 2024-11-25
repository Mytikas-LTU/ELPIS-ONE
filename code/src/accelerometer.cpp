//

#include <accelerometer.h>
#include <Adafruit_BNO08x.h>
#include <stage_recognition.h>
#include <basicIO.h>

//sets all the sensor outputs to recieve
//for more information on the sh2 sensorValues refer to the sh2 reference manual(downloaded in docs folder)
bool Accelerometer::SetReports() {
    if(!sensor.enableReport(SH2_ROTATION_VECTOR, 1000000/sampleRate)) {
        Serial.println("Could not enable rotation vector reports!");
        return false;
    }
    if(!sensor.enableReport(SH2_ACCELEROMETER, 1000000/sampleRate)) {
        Serial.println("Could not enable accelerometer reports!");
        return false;
    }
    return true;
}


Accelerometer::Accelerometer(){
    //Initialize the values
    acc = Vec3(0.0, 0.0, 0.0);
    rot = Quat(0.0, 0.0, 0.0, 0.0);
    setupError = false;
}



void Accelerometer::init(){
#if ENABLE_ACCELEROMETER
    //initialize the BNO
    Serial.println("Init BNO085!");
    while(!sensor.begin_I2C()){
        Serial.println("BNO init Error");
        digitalWrite(ERROR_LED_PIN, HIGH);
        delay(10);
    }
    digitalWrite(ERROR_LED_PIN, LOW);
    delay(100);

    //check if the reports could be set
    if(!SetReports()){
        digitalWrite(ERROR_LED_PIN, HIGH);
        setupError = true;
    }

    delay(1000);
    Serial.println("BNO085 Initialized");
#else  
    Serial.println("Accelerometer Disabled");
#endif
}


void Accelerometer::getData(telemetry* data){
#if ENABLE_ACCELEROMETER
    if(setupError)
        return;

    //temporary variables to handle missed readings
    Vec3 tacc = {0.0,0.0,0.0};
    Quat trot = {0.0,0.0,0.0,0.0};

    data->bnoReset = false;
    if(sensor.wasReset()) {
        Serial.println("BNO085 was reset");
        SetReports();
        data->bnoReset = true;
    }

    //get the BNO085 sensor data
    while(sensor.getSensorEvent(&sensorValue)){
        if(sensorValue.sensorId == SH2_ACCELEROMETER) {
            tacc.x = sensorValue.un.accelerometer.x;
            tacc.y = sensorValue.un.accelerometer.y;
            tacc.z = sensorValue.un.accelerometer.z;
        }
        if(sensorValue.sensorId == SH2_ROTATION_VECTOR){
            //you might think this is wrong, but trust me, the vector is messed up. DAMN AND BLAST THE AUTHORS OF THE BNO08X LIBRARY!! actually nvm they fixed it
            trot.r = sensorValue.un.rotationVector.real;
            trot.i = sensorValue.un.rotationVector.i;
            trot.j = sensorValue.un.rotationVector.j;
            trot.k = sensorValue.un.rotationVector.k;
        }
    }

    data->bnoMissed = 0;
    //the sensor sometimes misses, so if it did, just use the last value
    if(tacc.x == 0.0 && tacc.y == 0.0 && tacc.z == 0.0){
        tacc = acc;
        data->bnoMissed += 1;
    }
    if(trot.r == 0.0 && trot.i == 0.0 && trot.j == 0.0 && trot.k == 0.0){
        trot = rot;
        data->bnoMissed += 2;
    }

    acc = tacc;

    rot = trot;

    data->acc = acc;
    data->rot = rot;

    //rotate the acceleration vector
    Vec3 racc = acc.rotate(rot);
    data->rotAcc = racc;
#endif
}



