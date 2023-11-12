/***************************************************************************
    Author: Auri Åhs
    Simple telemetry using only a barometer
    Testing is underway with integrating the accelerometer
    REMEMBER TO ENABLE WRITING TO THE SD-CARD BEFORE LAUNCH
 ***************************************************************************/
// Plug in SCK ==> GPIO5 (D1)
// Plug in SDA ==> GPIO4 (D2)

#define ENABLE_BAROMETER 0
#define ENABLE_ACCELEROMETER 0
#define ENABLE_CARDWRITER 1
#define ENABLE_LOGGING 1
#define ENABLE_SERVO 1

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
//#include "SdFat.h"
//#include "sdios.h"
//#include <MPU6050_light.h>
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

Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
//MPU6050 mpu(Wire);
Adafruit_BNO08x bno08x(BNO08X_RESET);

const int8_t DISABLE_CS_PIN = -1; //needed according to examples

const uint8_t SD_CS_PIN = SS; // Define the ChipSelect-pin

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

//#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(10))
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
char buffer[8192];
float floatBuffer[2048];
int ptr=0;
long lastLoop;
File file;
bool rotated = false;

// Initialisation of stage_recognition
int alt_index = 0; 
float presArr[50]; 
int state_of_flight = 1;
int prevStage = 1;
struct telemetry flight_data;

void rotation(float i, float j, float k, float r, rot_acc* vec, float x, float y, float z){
  
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

    vec->xr=(r11*x+r12*y+r13*z);
    vec->yr=(r21*x+r22*y+r23*z);
    vec->zr=(r31*x+r32*y+r33*z);
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, sh2_Accelerometer_t* accelerometer, rot_acc* vec) {
    rotation(rotational_vector->i, rotational_vector->j, rotational_vector->k, rotational_vector->real, vec, accelerometer->x, accelerometer->y, accelerometer->z);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, sh2_Accelerometer_t* accelerometer, rot_acc* vec) {
    rotation(rotational_vector->i, rotational_vector->j, rotational_vector->k, rotational_vector->real, vec, accelerometer->x, accelerometer->y, accelerometer->z);
}

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
    if(!bno08x.enableReport(SH2_GRAVITY)) {
        Serial.println("Could not enable gravity reports!");
        return false;
    }
    return true;
}

//handles potential errors when setting BNO reports, currenlty unused
void BNOError(){
    //maybye flash the light to indicate errors?   
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
    //delay(10000);   // wait for native usb
    while (!Serial) {
        yield();
    }
//    Wire.begin(4,5);
    Serial.println("Flash");
    flash(3);
    //analogWrite(LED_PIN,128);
    delay(flashTime*3);

#if ENABLE_ACCELEROMETER
    Serial.println("Init BNO085!");
    //Wire.begin(0,5);
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
#else
    Serial.println("BNO085 Disabled");
#endif

#if ENABLE_BAROMETER
    Serial.println("Initializing BMP280");
    flash(1);
    //analogWrite(LED_PIN,128);
    delay(flashTime*3);
    unsigned status;
    status = bmp.begin();
    if (!status) {
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
        Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) {
            flash(1);
            digitalWrite(LED_PIN,HIGH);

            delay(flashTime*3);
            digitalWrite(ERROR_LED_PIN,HIGH);
        }
      }
    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    Serial.println(F("BMP280 initialized!"));
#else
    Serial.println("BMP280 Disabled");
#endif

#if ENABLE_CARDWRITER
    Serial.println(F("Initializing SD-card"));
    flash(2);
    //analogWrite(LED_PIN,128);
    delay(flashTime*3);
    while (!SD.begin(chipSelect)) {
        digitalWrite(ERROR_LED_PIN,HIGH);
        Serial.println("Failure to communicate with SD-card");
        flash(2);
        digitalWrite(LED_PIN,HIGH);
        delay(flashTime*3);
    }
    Serial.println("SD-card initialized!");

    Serial.println("Opening file");
    flash(3);
    //analogWrite(LED_PIN,128);
    delay(flashTime*3);
    
    file = SD.open(filename, O_RDWR | O_CREAT | O_APPEND);
    if (!file) {
        Serial.println("Failure to open file");
        while (1) {
            digitalWrite(ERROR_LED_PIN,HIGH);
            flash(3);
            digitalWrite(LED_PIN,HIGH);
            delay(flashTime*3);
        }

    }
    Serial.println("File open!");
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
    Serial.println("enabling servo");
    shuteServo.attach(9);
    shuteServo.write(180);
    Serial.println("servo 180");

    delay(5000);
    shuteServo.write(90);
    Serial.println("servo locked");
    delay(5000);
    shuteServo.write(180);
    Serial.println("servo deployed");

#endif
    digitalWrite(LAUNCH_LED_PIN,HIGH);
}

void loop() {
/*
    char buf[10];
    float temp,
          alt,
          pres;
    digitalWrite(LED_PIN,HIGH);
    
    sh2_Accelerometer_t acc, grav;
    sh2_RotationVectorWAcc_t rotVec;

    if(bno08x.wasReset()) {
        Serial.println("BNO085 was reset");
        SetReports();
    }

    //get the BNO085 sensor data
    if(!bno08x.getSensorEvent(&sensorValue)){
        Serial.println("Could not get Sensor Values!");
        return;
    }

    //read the sensor data
    switch(sensorValue.sensorId) {
        case SH2_ACCELEROMETER:
            acc=sensorValue.un.accelerometer;
            break;
        case SH2_ROTATION_VECTOR:
            rotVec = sensorValue.un.rotationVector;
            break;
        case SH2_GRAVITY:
            grav = sensorValue.un.gravity;
            break;
        case SH2_ARVR_STABILIZED_RV:
            quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &sensorValue.un.accelerometer, &vec);
        case SH2_GYRO_INTEGRATED_RV:
            // faster (more noise?)
            quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &sensorValue.un.accelerometer, &vec);
            break;
    }

    //Gather data
#if ENABLE_BAROMETER
    temp = bmp.readTemperature();
    pres = bmp.readPressure();
    alt = bmp.readAltitude(basePressure);
#endif

    //a drop of 1.2 kPa is equal to 100 m altitude

    // write data to card
    
  alt_index++;
  presArr[alt_index%50] = flight_data.pres;
  if (alt_index >=50)
  {
//    flight_data.direction = approx_direction(&presArr) //will be called before state_of_flight in execution order.
//    state_of_flight = state_of_flight_func(&flight_data, prevStage)
  }
  emergency_chute(&flight_data);
//    memcpy(&floatBuffer[ptr], &pres, 4);
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
    if(flight_data.parachute_state == 1 || alt_index >= 50){ 
        shuteServo.write(90); 
        
    }
    else { 
        shuteServo.write(0); 
        rotated = false;    
    }
#endif

    Serial.print(temp);
    Serial.print(" *C, ");


    Serial.print(pres);
    Serial.print(" Pa, ");

    Serial.print(alt);
    Serial.print(" m, ");

    Serial.print(alt-oldalt);
    Serial.print(" m/s, ");

    Serial.print("Acceleration vector: X: ");
    Serial.print(acc.x);
    Serial.print(", y: ");
    Serial.print(acc.y);
    Serial.print(", z: ");
    Serial.println(acc.z);

    Serial.print("Gravity vector: X: ");
    Serial.print(grav.x);
    Serial.print(", Y: ");
    Serial.print(grav.y);
    Serial.print(", Z: ");
    Serial.println(grav.z);

    Serial.print("Rotation vector: r: ");
    Serial.print(rotVec.real);
    Serial.print(", i: ");
    Serial.print(rotVec.i);
    Serial.print(", j: ");
    Serial.print(rotVec.j);
    Serial.print(", k: ");
    Serial.println(rotVec.k);


    Serial.print("xr: ");
    Serial.print(vec.xr);                        Serial.print("\t");
    Serial.print("yr: ");
    Serial.print(vec.yr);                        Serial.print("\t");
    Serial.print("zr: ");
    Serial.print(vec.zr);                        Serial.print("\t");

    oldalt = alt;

    Serial.println();


*/
    // constant time loop
    Serial.println("loop disabled");
    digitalWrite(LED_PIN,LOW);
    while(millis()-lastLoop < 1000/sampleRate) {}
    lastLoop = millis();
}
