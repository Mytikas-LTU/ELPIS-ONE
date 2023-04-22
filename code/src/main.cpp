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

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

const int8_t DISABLE_CS_PIN = -1; //needed according to examples

const uint8_t SD_CS_PIN = SS; // Define the ChipSelect-pin

const uint8_t LED_PIN = 2; // Define the LED-pin

const int flashTime = 100; // time of a flash of the status LED, in millis
const int pressureSamples = 10; // do better
const int sampleRate = 10; // samlpes per second

#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(4))

SdFs sd;
FatFile file;
char filename[10] = "tele.txt";
float oldalt;
float basePressure;
static ArduinoOutStream cout(Serial);
char buffer[8192];
int ptr=0;


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

    Serial.println("Flash");
    flash(3);
    analogWrite(LED_PIN,128);

    Serial.println("Initializing BMP280");
    flash(1);
    analogWrite(LED_PIN,128);
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
    file.close();
    sd.end();

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

    //Gather data
    temp = bmp.readTemperature();
    pres = bmp.readPressure();
    alt = bmp.readAltitude(basePressure);

    //a drop of 1.2 kPa is equal to 100 m altitude

    dtostrf(pres, 9, 2, buf);

    // write data to card
    memcpy(&buffer[ptr], &buf, 9);
    ptr += 10;
    buffer[ptr-1] = '\n';
    if(ptr >= 800) {
        digitalWrite(LED_PIN,HIGH);
//        file.write(&buffer, ptr);
//        file.sync();
        delay(100);
        Serial.println("Written to file!");
        ptr = 0;
        digitalWrite(LED_PIN,LOW);
    }
/*
    Serial.print(F("Temperature = "));
    Serial.print(temp);
    Serial.println(" *C");


    Serial.print(F("Pressure = "));
    Serial.print(buf);
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(alt);
    Serial.println(" m");

    Serial.print(F("Vertical speed = "));
    Serial.print(alt-oldalt);
    Serial.println(" m/s");
*/

    Serial.print(temp);
    Serial.print(" *C, ");


    Serial.print(buf);
    Serial.print(" Pa, ");

    Serial.print(alt);
    Serial.print(" m, ");

    Serial.print(alt-oldalt);
    Serial.print(" m/s, ");

    oldalt = alt;

    Serial.println();
    delay(1000/sampleRate);
}
