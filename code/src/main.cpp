/***************************************************************************
    Author: Auri Åhs
    Simple telemetry using only a barometer
    REMEMBER TO ENABLE WRITING TO THE SD-CARD BEFORE LAUNCH
 ***************************************************************************/
// Plug in SCK ==> GPIO5 (D1)
// Plug in SDA ==> GPIO4 (D2)

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include "SdFat.h"
#include "sdios.h"
//#include <MPU6050_light.h>
#include <Adafruit_BNO08x.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

#define BNO08X_RESET -1

Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
//MPU6050 mpu(Wire);
Adafruit_BNO08x bno08x(BNO08X_RESET);

const int8_t DISABLE_CS_PIN = -1; //needed according to examples

const uint8_t SD_CS_PIN = SS; // Define the ChipSelect-pin

const uint8_t LED_PIN = 2; // Define the LED-pin

const int flashTime = 100; // time of a flash of the status LED, in millis
const int pressureSamples = 10; // do better
const int sampleRate = 100; // samlpes per second

sh2_SensorValue_t sensorValue; //contains the sensor data for bno085

#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(10))

SdFs sd;
FatFile file;
char filename[10] = "tele.txt";
float oldalt;
float basePressure;
static ArduinoOutStream cout(Serial);
char buffer[8192];
float floatBuffer[2048];
int ptr=0;
long lastLoop;

void errorPrint() {
    if (sd.sdErrorCode()) {
        cout << F("SD errorCode: ") << hex << showbase;
        printSdErrorSymbol(&Serial, sd.sdErrorCode());
        cout << F(" = ") << int(sd.sdErrorCode()) << endl;
        cout << F("SD errorData = ") << int(sd.sdErrorData()) << dec << endl;
    }
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
    char pre1[] = "Sample rate: ";
    char pre2[] = "Base pressure: ";
    char pre3[] = "\n";
    char buf[10];
    pinMode(LED_PIN,OUTPUT);
    Serial.begin(9600);
    //delay(10000);   // wait for native usb
    while (!Serial) {
        yield();
    }
    Wire.begin(4,5);
    Serial.println("Flash");
    flash(3);
    analogWrite(LED_PIN,128);
    delay(flashTime*3);

    Serial.println("Init BNO085!");
    //Wire.begin(0,5);
    while(!bno08x.begin_I2C()) {
        Serial.println("BNO error");
    }

    //check whether all the reports could be found, otherwise prevent the rest of code from runnning
    if(!SetReports()) {
        while(true) {
            delay(1000);
        }
    }
    delay(1000);

    Serial.println("Initializing BMP280");
    flash(1);
    analogWrite(LED_PIN,128);
    delay(flashTime*3);
    unsigned status;
    //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
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
        }
      }

    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    Serial.println(F("BMP280 initialized!"));

    Serial.println(F("Initializing SD-card"));
    flash(2);
    analogWrite(LED_PIN,128);
    delay(flashTime*3);
    while (!sd.begin(SD_CONFIG)) {
        errorPrint();
        flash(2);
        digitalWrite(LED_PIN,HIGH);
        delay(flashTime*3);
    }
    Serial.println("SD-card initialized!");

    Serial.println("Opening file");
    flash(3);
    analogWrite(LED_PIN,128);
    delay(flashTime*3);
    if (!file.open(filename, O_RDWR | O_CREAT | O_APPEND)) {
        Serial.println("Failure to open file");
        while (1) {
            flash(3);
            digitalWrite(LED_PIN,HIGH);
            delay(flashTime*3);
        }

    }
    Serial.println("File open!");

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
    //remove below
//    file.close();
//    sd.end();

    file.write(&pre1, 14);
    itoa(sampleRate, buf, 10);
    file.write(&buf, 4);
    file.write(&pre2, 16);
    dtostrf(basePressure, 9, 2, buf);
    file.write(&buf, 10);
    file.write(&pre3, 2);
    file.sync();
    flash(4);
    digitalWrite(LED_PIN,LOW);
}

void loop() {
    char buf[10];
    float temp,
          alt,
          pres;

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
    }

    //Gather data
    temp = bmp.readTemperature();
    pres = bmp.readPressure();
    alt = bmp.readAltitude(basePressure);
    delay(5);

    //a drop of 1.2 kPa is equal to 100 m altitude

    dtostrf(pres, 9, 2, buf);

    // write data to card
    memcpy(&floatBuffer[ptr], &pres, 4);
    ptr += 1;
//    floatBuffer[ptr] = pres;
//    buffer[ptr-1] = '\n';
    if(ptr >= 800) {
        digitalWrite(LED_PIN,HIGH);
        file.write(&floatBuffer, ptr);
        file.sync();
        delay(100);
        Serial.println("Written to file!");
        ptr = 0;
        digitalWrite(LED_PIN,LOW);
    }
/*
    Serial.print(temp);
    Serial.print(" *C, ");


    Serial.print(buf);
    Serial.print(" Pa, ");

    Serial.print(alt);
    Serial.print(" m, ");

    Serial.print(alt-oldalt);
    Serial.print(" m/s, ");
*/
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

    oldalt = alt;

    Serial.println();
//    delay(1000/sampleRate);
    while(millis()-lastLoop < 1000/sampleRate) {}
    lastLoop = millis();
}
