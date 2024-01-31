/***************************************************************************
    Author: Auri Åhs
    Simple telemetry using only a barometer
    Testing is underway with integrating the accelerometer
    REMEMBER TO ENABLE WRITING TO THE SD-CARD BEFORE LAUNCH
 ***************************************************************************/
// Plug in SCK ==> GPIO5 (D1)
// Plug in SDA ==> GPIO4 (D2)

#define ENABLE_BAROMETER 0
#define ENABLE_ACCELEROMETER 1
#define ENABLE_CARDWRITER 0
#define ENABLE_LOGGING 0
#define ENABLE_SERVO 0

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <SD.h>
#include <Adafruit_BNO08x.h>
#include<Servo.h>
#include "stage_recognition.h"


#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)
#define SERVO_PIN (9)

#define BNO08X_RESET -1

#define SERVO_OPEN 180
#define SERVO_LOCKED 90

Adafruit_BMP280 bmp; // I2C
Adafruit_BNO08x bno08x(BNO08X_RESET);


const uint8_t LED_PIN = 2; // Define the LED-pin
const uint8_t ERROR_LED_PIN = 3; // Define the LED-pin
const uint8_t LAUNCH_LED_PIN = 4; // Define the LED-pin
const uint8_t CARD_LED_PIN = 5; // Define the LED-pin


const int flashTime = 100; // time of a flash of the status LED, in millis
const int pressureSamples = 10; // do better
const int sampleRate = 10; // samlpes per second

const int chipSelect = SS1;

Servo shuteServo;

sh2_SensorValue_t sensorValue; //contains the sensor data for bno085

#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;
#endif


char filename[10] = "tele.txt";
float oldalt;
float basePressure;
int ptr=0;
long lastLoop;
File file;
// Initialisation of stage_recognition
int alt_index = 0;
float presArr[50];
int prevStage = 1;
struct telemetry flight_data;
vec3 acc;
quat rot;

void rotation(float i, float j, float k, float r, vec3* vec, float x, float y, float z) {
    float s = pow(-2,(sqrt((i*i)+(j*j)+(k*k)+(r*r))));

    float r11 = 1-2*s*((j*j)+(k*k));
    float r12 = 2*s*((i*i)*(j*j)-(k*k)*(r*r));
    float r13 = 2*s*((i*i)*(k*k)+(j*j)*(r*r));

    float r21 = 2*s*((i*i)*(j*j)+(k*k)*(r*r));
    float r22 = 1-2*s*((i*i)+(k*k));
    float r23 = 2*s*((j*j)*(k*k)-(i*i)*(r*r));

    float r31 = 2*s*((i*i)*(k*k)-(j*j)*(r*r));
    float r32 = 2*s*((j*j)*(k*k)+(i*i)*(r*r));
    float r33 = 1-2*s*((i*i)+(j*j));

    vec->x=(r11*x+r12*y+r13*z);
    vec->y=(r21*x+r22*y+r23*z);
    vec->z=(r31*x+r32*y+r33*z);
}
/*
void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, sh2_Accelerometer_t* accelerometer, vec3* vec) {
    rotation(rotational_vector->i, rotational_vector->j, rotational_vector->k, rotational_vector->real, vec, accelerometer->x, accelerometer->y, accelerometer->z);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, sh2_Accelerometer_t* accelerometer, vec3* vec) {
    rotation(rotational_vector->i, rotational_vector->j, rotational_vector->k, rotational_vector->real, vec, accelerometer->x, accelerometer->y, accelerometer->z);
}
*/
void flash(int times) {
    for (int i = 0; i<times; i++) {
        digitalWrite(LED_PIN,HIGH);
        delay(flashTime);
        digitalWrite(LED_PIN,LOW);
        delay(flashTime);
    }
}

//sets all the sensor outputs to recieve
//for more information on the sh2 sensorValues refer to the sh2 reference manual(downloaded in docs folder)
bool SetReports() {
    if(!bno08x.enableReport(SH2_ROTATION_VECTOR, 1000000/sampleRate)) {
        Serial.println("Could not enable rotation vector reports!");
        return false;
    }
    if(!bno08x.enableReport(SH2_ACCELEROMETER, 1000000/sampleRate)) {
        Serial.println("Could not enable accelerometer reports!");
        return false;
    }
    return true;
}

//handles potential errors when setting BNO reports, currenlty unused
void BNOError(){
    //maybye flash the light to indicate errors?
}

//prints a vec3, what do you think?
void printVec3(const char *vecName, float x, float y, float z, bool lineBreak) {
    Serial.print(vecName);
    Serial.print("\tx: ");
    Serial.print(x);
    Serial.print(",\ty: ");
    Serial.print(y);
    Serial.print(",\tz: ");
    Serial.print(z);
    if(lineBreak)
        Serial.println();
}

//prints a quaternion
void printQuat(const char *quatName, float r, float i, float j, float k, bool lineBreak){
    Serial.print(quatName);
    Serial.print(",\tr: ");
    Serial.print(r);
    Serial.print(",\ti: ");
    Serial.print(i);
    Serial.print(",\tj: ");
    Serial.print(j);
    Serial.print(",\tk: ");
    Serial.print(k);
    if(lineBreak)
        Serial.println();
}

//The Hamilton Product, gracefully copied from wikipedia, equates to q1*q2
quat multiply_quat(quat q1, quat q2) {
    float r = q1.R*q2.R - q1.I*q2.I - q1.J*q2.J - q1.K*q2.K;
    float i = q1.R*q2.I + q1.I*q2.R + q1.J*q2.K - q1.K*q2.J;
    float j = q1.R*q2.J - q1.I*q2.K + q1.J*q2.R + q1.K*q2.I;
    float k = q1.R*q2.K + q1.I*q2.J - q1.J*q2.I + q1.K*q2.R;
    return quat{r, i, j, k};
}

quat invert_quat(quat q){
    return{q.R, -q.I, -q.J, -q.K};
}

void setup() {
    float pressures = 0;
    pinMode(LED_PIN,OUTPUT);
    pinMode(LAUNCH_LED_PIN,OUTPUT);
    pinMode(ERROR_LED_PIN,OUTPUT);
    pinMode(CARD_LED_PIN,OUTPUT);
    digitalWrite(LAUNCH_LED_PIN,LOW);
    digitalWrite(ERROR_LED_PIN,LOW);
    Serial.begin(9600);
    while (!Serial) {
        yield();
    }
    Serial.println("Begin boot process");
    flash(3);
    delay(flashTime*3);

#if ENABLE_ACCELEROMETER
    Serial.println("Init BNO085!");
    while(!bno08x.begin_I2C()) {
        Serial.println("BNO error");
        digitalWrite(ERROR_LED_PIN,HIGH);
    }
    delay(100);
    //check whether all the reports could be found, otherwise prevent the rest of code from runnning
    if(!SetReports()) {
        while(true) {
            delay(1000);
        }
    }
    delay(1000);
    Serial.println("BNO085 Initialized");
#else
    Serial.println("BNO085 Disabled");
#endif

#if ENABLE_BAROMETER
    Serial.println("Initializing BMP280");
    flash(1);
    delay(flashTime*3);
    while(!bmp.begin()) {
        Serial.println("BMP280 Error");
        digitalWrite(ERROR_LED_PIN,HIGH);
        flash(1);
        delay(flashTime*3);

      }
    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    Serial.println("BMP280 initialized!");
#else
    Serial.println("BMP280 Disabled");
#endif

#if ENABLE_CARDWRITER
    Serial.println("Initializing SD-card");
    flash(2);
    delay(flashTime*3);
    while (!SD.begin(chipSelect)) {
        digitalWrite(ERROR_LED_PIN,HIGH);
        Serial.println("Failure to communicate with SD-card");
        flash(2);
        delay(flashTime*3);
    }
    Serial.println("SD-card initialized!");

#if ENABLE_LOGGING
    Serial.println("Opening file");
    flash(3);
    delay(flashTime*3);

    file = SD.open(filename, O_RDWR | O_CREAT | O_APPEND);
    while (!file) {
        Serial.println("Failure to open file");
        digitalWrite(ERROR_LED_PIN,HIGH);
        flash(3);
        delay(flashTime*3);
        file = SD.open(filename, O_RDWR | O_CREAT | O_APPEND);
    }
    Serial.println("File open!");
#else
    Serial.println("Logging disabled");
#endif
#else
    Serial.println("Card writer disabled");
#endif

#if ENABLE_BAROMETER
    Serial.print("Calibrating pressure at ground level");
    for(int i=0; i<pressureSamples; i++) {
        Serial.print(".");
        pressures += bmp.readPressure();
        delay(500);
        digitalWrite(LED_PIN,HIGH);
        delay(500);
        digitalWrite(LED_PIN,LOW);
    }
    Serial.println();
    basePressure = (pressures/pressureSamples)/100;
    Serial.print("Calibrated at ");
    Serial.print(basePressure);
    Serial.println(" hPa");
#else
    Serial.println("Pressure calibration disabled");
#endif

#if ENABLE_LOGGING
    file.print("Sample rate: ");
    file.print(sampleRate);
    file.print(" Base pressure: ");
    file.println(basePressure);
    file.flush();
    flash(4);
    digitalWrite(LED_PIN,LOW);
#else
    Serial.println("Logging disabled, closing file");
    file.close();
#endif

#if ENABLE_SERVO
    Serial.println("Enabling servo");
    shuteServo.attach(9);
    shuteServo.write(SERVO_OPEN);
    Serial.println("Servo in OPEN position");
    delay(5000);
    shuteServo.write(SERVO_LOCKED);
    Serial.println("Servo in LOCKED position");
#else
    Serial.println("Servo disabled");
#endif
    digitalWrite(LAUNCH_LED_PIN,HIGH);
}

void loop() {
    float temp,
          alt,
          pres;
    digitalWrite(LED_PIN,HIGH);

#if ENABLE_ACCELEROMETER
    if(bno08x.wasReset()) {
        Serial.println("BNO085 was reset");
        SetReports();
    }

    vec3 tacc;
    tacc.x = 0.0;
    tacc.y = 0.0;
    tacc.z = 0.0;

    quat trot;
    trot.R = 0.0;
    trot.I = 0.0;
    trot.J = 0.0;
    trot.K = 0.0;

    int i = 0;
    //continuously poll the bno for data
    while(bno08x.getSensorEvent(&sensorValue)&&i<36){
        i++;
        if(sensorValue.sensorId==SH2_ACCELEROMETER) {
            tacc.x=sensorValue.un.accelerometer.x;
            tacc.y=sensorValue.un.accelerometer.y;
            tacc.z=sensorValue.un.accelerometer.z;
        }
        if(sensorValue.sensorId==SH2_ROTATION_VECTOR) {
            //It would appear that the bno sends the data in the wrong order. Source: Trust me bro
            trot.R = sensorValue.un.rotationVector.real;
            trot.I = sensorValue.un.rotationVector.i;
            trot.J = sensorValue.un.rotationVector.j;
            trot.K = sensorValue.un.rotationVector.k;
        }
    }

    if(tacc.x == 0.0 && tacc.y == 0.0 && tacc.z == 0.0){
        tacc.x = acc.x;
        tacc.y = acc.y;
        tacc.z = acc.z;
    }
    if(trot.R == 0.0 && trot.I == 0.0 && trot.J == 0.0 && trot.K == 0.0){
        trot.R = rot.R;
        trot.I = rot.I;
        trot.J = rot.J;
        trot.K = rot.K;
    }

    acc.x = tacc.x;
    acc.y = tacc.y;
    acc.z = tacc.z;

    rot.R = trot.R;
    rot.I = trot.I;
    rot.J = trot.J;
    rot.K = trot.K;

    flight_data.acc = acc;
    flight_data.rot = rot;
    //printVec3("Unrotated acceleration vector", acc.x, acc.y, acc.z, true);

    //rotate the acceleration vector
    quat tempAcc = {0, acc.x, acc.y, acc.z};
    //quat rotAcc = multiply_quat(multiply_quat(rot, tempAcc), invert_quat(rot));
    quat q = rot;
    quat q_ = invert_quat(rot);

    quat t = multiply_quat(q,tempAcc);
    quat rotAcc = multiply_quat(t,q_);
    vec3 racc;
    racc.x = rotAcc.I;
    racc.y = rotAcc.J;
    racc.z = rotAcc.K;

#endif

#if ENABLE_BAROMETER
    temp = bmp.readTemperature();
    pres = bmp.readPressure();
    alt = bmp.readAltitude(basePressure);
#endif

    alt_index++;
    presArr[alt_index%50] = flight_data.pres;
    if (alt_index >=50)
    {
        flight_data.direction = approx_direction(presArr, basePressure); //will be called before state_of_flight in execution order.
        if (flight_data.direction==0){
            //Serial.print("Error in state of flight");
        } else {
            prevStage = state_of_flight_func(&flight_data, prevStage);
        }
    }
    emergency_chute(&flight_data, prevStage);

    ptr += 1;
    file.println(pres);
    if(ptr >= 800) {
        digitalWrite(CARD_LED_PIN,HIGH);
#if ENABLE_LOGGING
//        file.write(&floatBuffer, (size_t)ptr);
        file.flush();
        Serial.println("Written to file!");
#else
        Serial.println("(simulated) Written to file!");
#endif
        ptr = 0;
        digitalWrite(CARD_LED_PIN,LOW);
    }

#if ENABLE_SERVO
    if(flight_data.parachute_state == 1){
        shuteServo.write(SERVO_OPEN);
    }
    else {
        shuteServo.write(SERVO_LOCKED);
    }
#endif

#if ENABLE_BAROMETER
    Serial.print(temp);
    Serial.print(" *C, ");

    Serial.print(pres);
    Serial.print(" Pa, ");

    Serial.print(alt);
    Serial.print(" m, ");

    Serial.print(alt-oldalt);
    Serial.print(" m/s, ");

    oldalt = alt;

#endif

#if ENABLE_ACCELEROMETER
    //printVec3("Acceleration vector", tacc.x, tacc.y, tacc.z, true);

    //printVec3("Gravity vector", grav.x, grav.y, grav.z, true);

    //printQuat("Rotation Vector", trot.R, trot.I, trot.J, trot.K, true);

    printVec3("Rotated Acceleration", racc.x, racc.y, racc.z, true);
/*
    Serial.print("xr: ");
    Serial.print(vec.xr);                        Serial.print("\t");
    Serial.print("yr: ");
    Serial.print(vec.yr);                        Serial.print("\t");
    Serial.print("zr: ");
    Serial.print(vec.zr);                        Serial.print("\t");*/
#endif

    //Serial.println();
    // constant time loop
    digitalWrite(LED_PIN,LOW);
    while(millis()-lastLoop < 1000/sampleRate) {}
    lastLoop = millis();
}
