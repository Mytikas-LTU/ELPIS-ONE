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
#define ENABLE_DUMMYDATA 0


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

struct rot_acc {
    float xr;
    float yr;
    float zr;
} vec;


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
long begin_flight_time = 0;
int in_flight = 0; 
vec3 acc;
quat rot;


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
    if(!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
        Serial.println("Could not enable rotation vector reports!");
        return false;
    }
    if(!bno08x.enableReport(SH2_ACCELEROMETER)) {
        Serial.println("Could not enable accelerometer reports!");
        return false;
    }
    return true;
}

//handles potential errors when setting BNO reports, currenlty unused
void BNOError(){
    //maybye flash the light to indicate errors?
}

//prints a vec3 to serial. Was this comment necessary? probably not
void printVec3(char *vecName, vec3 vec, bool lineBreak) {
    Serial.print(vecName);
    Serial.print("\tx: ");
    Serial.print(vec.x);
    Serial.print(",\ty: ");
    Serial.print(vec.y);
    Serial.print(",\tz: ");
    Serial.print(vec.z);
    if(lineBreak)
        Serial.println();
}

//print a quaternion to Serial. Do I really need to explain these
void printQuat(char *quatName, quat _quat, bool lineBreak){
    Serial.print(quatName);
    Serial.print(",\tr: ");
    Serial.print(_quat.R);
    Serial.print(",\ti: ");
    Serial.print(_quat.I);
    Serial.print(",\tj: ");
    Serial.print(_quat.J);
    Serial.print(",\tk: ");
    Serial.print(_quat.K);
    if(lineBreak)
        Serial.println();
}

void setup() {
    long int boottime = millis();
    float pressures = 0;
    pinMode(LED_PIN,OUTPUT);
    pinMode(ERROR_LED_PIN,OUTPUT);
    pinMode(LAUNCH_LED_PIN,OUTPUT);
    pinMode(CARD_LED_PIN,OUTPUT);
    digitalWrite(LAUNCH_LED_PIN,LOW);
    digitalWrite(ERROR_LED_PIN,LOW);
    Serial.begin(9600);

    while (!Serial && millis() - boottime < 10000) {
        yield();
    }

    Serial.println("Begin boot process");
    flash(3);
    delay(flashTime*3);

#if ENABLE_ACCELEROMETER
    delay(1000);
    Serial.println("Init BNO085!");
    while(!bno08x.begin_I2C()) {
        Serial.println("BNO error");
        digitalWrite(ERROR_LED_PIN,HIGH);
    }
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
    flight_data.base_pres = (pressures/pressureSamples)/100;
    Serial.print("Calibrated at ");
    Serial.print(flight_data.base_pres);
    Serial.println(" hPa");
#else
    Serial.println("Pressure calibration disabled");
#endif

#if ENABLE_LOGGING
    file.print("Sample rate: ");
    file.print(sampleRate);
    file.print(" Base pressure: ");
    file.println(flight_data.base_pres);
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

    flight_data.acc_globZ = 0.1;

}

void loop() {
    float temp,
          alt,
          pres;
    int written;
    digitalWrite(LED_PIN,HIGH);

    flight_data.time = millis();

#if ENABLE_ACCELEROMETER
    //temporary variables to handle missed readings
    vec3 tacc = {0.0,0.0,0.0};
    quat trot = {0.0,0.0,0.0,0.0};

    flight_data.bnoReset = false;
    if(bno08x.wasReset()) {
        Serial.println("BNO085 was reset");
        SetReports();
        flight_data.bnoReset = true;
    }

    //get the BNO085 sensor data
    while(bno08x.getSensorEvent(&sensorValue)){
        if(sensorValue.sensorId == SH2_ACCELEROMETER) {
            tacc.x = sensorValue.un.accelerometer.x;
            tacc.y = sensorValue.un.accelerometer.y;
            tacc.z = sensorValue.un.accelerometer.z;
        }
        if(sensorValue.sensorId == SH2_ROTATION_VECTOR){
            //you might think this is wrong, but trust me, the vector is messed up. DAMN AND BLAST THE AUTHORS OF THE BNO08X LIBRARY!! actually nvm they fixed it
            trot.R = sensorValue.un.rotationVector.real;
            trot.I = sensorValue.un.rotationVector.i;
            trot.J = sensorValue.un.rotationVector.j;
            trot.K = sensorValue.un.rotationVector.k;
        }
    }

    //the next section will probably work, as testing shows that data loss occurs really infrequently
    //if we didnt recieve any data, use the one stored from last iteration
    flight_data.bnoMissed = 0;
    if(tacc.x ==0.0 && tacc.y == 0.0 && tacc.z == 0.0){
        tacc = acc;
        flight_data.bnoMissed |= 0b1;
    }
    if(trot.I == 0.0 &&trot.K == 0.0 &&trot.J == 0.0 &&trot.R == 0.0){
        trot = rot;
        flight_data.bnoMissed |= 0b10;
    }

    //store the data of the current iteration
    acc = tacc;
    rot = trot;

    flight_data.acc = acc;
    flight_data.rot = rot;
#endif

#if ENABLE_BAROMETER
    flight_data.temp = bmp.readTemperature();
    flight_data.pres = bmp.readPressure();
    flight_data.alt = bmp.readAltitude(basePressure);
#endif

#if ENABLE_DUMMYDATA
    gen_dummy_data(&flight_data, alt_index, prevStage);
#endif
    if ((flight_data.pres - flight_data.base_pres*100) <= -120 && begin_flight_time == 0)
    {
       begin_flight_time = millis();
       in_flight = 1;
    }
    if (in_flight == 1)
    {
        flight_data.flight_time = millis() - begin_flight_time;
    }
    emergency_chute(&flight_data, prevStage);


#if ENABLE_LOGGING
    ptr += 1;
    written = file.write((byte *) &flight_data, sizeof(flight_data));
    //file.println(flight_data.pres);
    if(ptr >= 80) {
        digitalWrite(CARD_LED_PIN,HIGH);
//        file.write(&floatBuffer, (size_t)ptr);
        file.flush();
        Serial.println("Written to file!");
        ptr = 0;
        digitalWrite(CARD_LED_PIN,LOW);
    }
#endif

#if ENABLE_SERVO
    if(flight_data.parachute_state == 1){
        shuteServo.write(SERVO_OPEN);
    }
    else {
        shuteServo.write(SERVO_LOCKED);
    }
#endif

#if ENABLE_BAROMETER || ENABLE_DUMMYDATA

   Serial.print(flight_data.pres - flight_data.base_pres*100);
    Serial.print(" Pa, ");


   // Serial.print("State of flight,");
   // Serial.print(prevStage);

    Serial.print("Parachute:");
    Serial.print(flight_data.parachute_state);
    Serial.print(", ");

    Serial.print(flight_data.flight_time);
    Serial.print("ms, ");

    

    oldalt = flight_data.alt;

#endif

#if ENABLE_ACCELEROMETER 
    //printVec3("Acceleration vector", acc, true);

    //printQuat("Rotation Vector", rot, true);

    //printVec3("Rotated Acceleration", acc_.xr, vec.yr, vec.zr, true);
/*
    Serial.print("xr: ");
    Serial.print(vec.xr);                        Serial.print("\t");
    Serial.print("yr: ");
    Serial.print(vec.yr);                        Serial.print("\t");
    Serial.print("zr: ");
    Serial.print(vec.zr);                        Serial.print("\t");*/

    
#endif
    Serial.println();
    Serial.print(written);
    Serial.println(" bytes to file write-buffer");


    Serial.println();
    // constant time loop
    digitalWrite(LED_PIN,LOW);
    while(millis()-lastLoop < 1000/sampleRate) {}
    lastLoop = millis();
}
