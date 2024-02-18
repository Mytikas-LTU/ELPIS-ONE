/***************************************************************************
    Author: Aurora Å, Isak H, Elias R, Alexander B
    Full telemetry and logging suite.
 ***************************************************************************/


#define ENABLE_BAROMETER 1
#define ENABLE_ACCELEROMETER 1
#define ENABLE_CARDWRITER 1
#define ENABLE_LOGGING 1
#define ENABLE_SERVO 1
#define ENABLE_DUMMYDATA 0


#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include<Servo.h>
#include "stage_recognition.h"
#include "accelerometer.h"
#include "Barometer.h"
#include "basicIO.h"


#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)
#define SERVO_PIN (9)

#define SERVO_OPEN 180
#define SERVO_LOCKED 90

Barometer barom_sensor;
Accelerometer acc_sensor;

const int chipSelect = SS1;

Servo shuteServo;

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

void setup() {
    long int boottime = millis();
    pinMode(LED_PIN,OUTPUT);
    pinMode(ERROR_LED_PIN,OUTPUT);
    pinMode(LAUNCH_LED_PIN,OUTPUT);
    pinMode(CARD_LED_PIN,OUTPUT);
    digitalWrite(LAUNCH_LED_PIN,LOW);
    digitalWrite(ERROR_LED_PIN,LOW);
    Serial.begin(9600);

    while (!Serial && millis() - boottime < 2000) {
        yield();
    }

    Serial.println("Begin boot process");
    flash(3);
    delay(flashTime*3);

#if ENABLE_SERVO
    Serial.println("Enabling servo");
    shuteServo.attach(9);
    shuteServo.write(SERVO_OPEN);
    Serial.println("Servo in OPEN position");
#else
    Serial.println("Servo disabled");
#endif


#if ENABLE_ACCELEROMETER
    acc_sensor = Accelerometer();
#else
    Serial.println("BNO085 Disabled");
#endif

#if ENABLE_BAROMETER
    barom_sensor = Barometer();
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
    flight_data.base_pres = barom_sensor.calibrate();
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
    shuteServo.write(SERVO_LOCKED);
    Serial.println("Servo in LOCKED position");
#endif
    digitalWrite(LAUNCH_LED_PIN,HIGH);

}

void loop() {
    int written;
    digitalWrite(LED_PIN,HIGH);

    flight_data.time = millis();

#if ENABLE_ACCELEROMETER
    acc_sensor.getData(&flight_data);
#endif

#if ENABLE_BAROMETER
    barom_sensor.getData(&flight_data);
#endif

#if ENABLE_DUMMYDATA
    gen_dummy_data(&flight_data, alt_index, prevStage);
#endif
    if (flight_data.acc.x < -12.00 && begin_flight_time == 0)
    {
        Serial.println("begin flight!!----------------------------------------");
        Serial.println("begin flight!!----------------------------------------");
        Serial.println("begin flight!!----------------------------------------");
        Serial.println("begin flight!!----------------------------------------");
        Serial.println("begin flight!!----------------------------------------");
       begin_flight_time = millis();
       in_flight = 1;
    }
    if (in_flight == 1)
    {
        digitalWrite(CARD_LED_PIN,HIGH);
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

flight_data.acc.print("Local Acceleration", true);
flight_data.rotAcc.print("Global Acceleration", true);
flight_data.rot.print("Rotation Vector", true);

#endif
/*    Serial.println();
    Serial.print(written);
    Serial.println(" bytes to file write-buffer");
*/

    //Serial.println();
    // constant time loop
    digitalWrite(LED_PIN,LOW);
    while(millis()-lastLoop < 1000/sampleRate) {}
    lastLoop = millis();
}
