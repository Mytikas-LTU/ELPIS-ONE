//

#include <accelerometer.h>
#include <Adafruit_BNO08x.h>
#include <stage_recognition.h>

#define SAMPLE_RATE 10
#define ERROR_LED_PIN 4

//sets all the sensor outputs to recieve
//for more information on the sh2 sensorValues refer to the sh2 reference manual(downloaded in docs folder)
bool Accelerometer::SetReports() {
    if(!sensor.enableReport(SH2_ROTATION_VECTOR, 1000000/SAMPLE_RATE)) {
        Serial.println("Could not enable rotation vector reports!");
        return false;
    }
    if(!sensor.enableReport(SH2_ACCELEROMETER, 1000000/SAMPLE_RATE)) {
        Serial.println("Could not enable accelerometer reports!");
        return false;
    }
    return true;
}

Accelerometer::Accelerometer(){
    //Initialize the values
    acc = vec3{0.0, 0.0, 0.0};
    rot = quat{0.0, 0.0, 0.0, 0.0};

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
}


//The Hamilton Product, gracefully copied from wikipedia, equates to q1*q2
quat multiply_quat(quat q1, quat q2) {
    float r = q1.r*q2.r - q1.i*q2.i - q1.j*q2.j - q1.k*q2.k;
    float i = q1.r*q2.i + q1.i*q2.r + q1.j*q2.k - q1.k*q2.j;
    float j = q1.r*q2.j - q1.i*q2.k + q1.j*q2.r + q1.k*q2.i;
    float k = q1.r*q2.k + q1.i*q2.j - q1.j*q2.i + q1.k*q2.r;
    return quat{r, i, j, k};
}

quat invert_quat(quat q){
    return{q.r, -q.i, -q.j, -q.k};
}

void Accelerometer::getData(telemetry* data){
    if(setupError)
        return;

    //temporary variables to handle missed readings
    vec3 tacc = {0.0,0.0,0.0};
    quat trot = {0.0,0.0,0.0,0.0};

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
        if(sensorValue.sensorId == SH2_ARVR_STABILIZED_RV){
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
    quat tempAcc = {0, acc.x, acc.y, acc.z};
    quat q = rot;
    quat q_ = invert_quat(rot);

    quat t = multiply_quat(q,tempAcc);
    quat rotAcc = multiply_quat(t,q_);
    vec3 racc;

    racc.x = rotAcc.i;
    racc.y = rotAcc.j;
    racc.z = rotAcc.k;
    data->rotAcc = racc;
}