/***************************************************************************
    Author: Aurora Å, Isak H, Elias R, Alexander B, Gustav M
    Full telemetry and logging suite.
 ***************************************************************************/
#include <Wire.h>
#include <SPI.h>
#include<Servo.h>
#include "stage_recognition.h"
#include "accelerometer.h"
#include "Barometer.h"
#include "storage.h"
#include "basicIO.h"


#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)
#define SERVO_PIN (9)

// Define constants in the code
#define SERVO_OPEN 0
#define SERVO_LOCKED 180
#define FALL_DIST 1    // distance needed to fall
#define ARM_DIST 2     // distance to arm parachute

Barometer barom_sensor;
Accelerometer acc_sensor;
Storage storage;

Servo shuteServo;

struct rot_acc {
    float xr;
    float yr;
    float zr;
} vec;

float max_alt_height = 0;
float max_alt_time = 0;
float oldalt;
float basePressure;
long lastLoop;
// Initialisation of stage_recognition
int alt_index = 0;
float presArr[50];
int prevStage = 1;
struct telemetry flight_data;
long begin_flight_time = 0;
int in_flight = 0;
int parachute_arm = 0;

void setup() {
    long int boottime = millis();
    pinMode(LED_PIN,OUTPUT);
    pinMode(ERROR_LED_PIN,OUTPUT);
    pinMode(LAUNCH_LED_PIN,OUTPUT);
    pinMode(CARD_LED_PIN,OUTPUT);
    digitalWrite(LAUNCH_LED_PIN,LOW);
    digitalWrite(ERROR_LED_PIN,LOW);
    Serial.begin(9600);
    Wire.begin();
    Wire.setClock(400000);

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

    acc_sensor.init();
    barom_sensor.init();
    storage.init();

    flight_data.base_pres = barom_sensor.calibrate();
    storage.writeHeader(&flight_data);

#if ENABLE_SERVO
    shuteServo.write(SERVO_LOCKED);
    Serial.println("Servo in LOCKED position");
#endif
    digitalWrite(LAUNCH_LED_PIN,HIGH);

}

void loop() {
    digitalWrite(LED_PIN,HIGH);

    flight_data.time = millis();


    if(acc_sensor.poll()) {
        Serial.print("Lost connection with accelerometer ");
    } else {
        acc_sensor.getData(&flight_data);
    }

    if(barom_sensor.poll()) {
        Serial.print("Lost connection with barometer ");
    } else {
        barom_sensor.getData(&flight_data);
    }


#if ENABLE_DUMMYDATA
    gen_dummy_data(&flight_data, alt_index, prevStage);
#endif
    if (flight_data.acc.y < -12.00 && begin_flight_time == 0)
    {
        Serial.println("begin flight!!----------------------------------------");
        Serial.println("begin flight!!----------------------------------------");
        Serial.println("begin flight!!----------------------------------------");
        Serial.println("begin flight!!----------------------------------------");
        Serial.println("begin flight!!----------------------------------------");
       begin_flight_time = millis();
       in_flight = 1;
    }
    if (parachute_arm) {
        digitalWrite(CARD_LED_PIN,HIGH);
    }
    if (in_flight == 1) {
        flight_data.flight_time = millis() - begin_flight_time;
    }
//    emergency_chute(&flight_data, prevStage);


    storage.write(&flight_data);

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

    Serial.print(flight_data.alt);
    Serial.print(" m, ");

   // Serial.print("State of flight,");
   // Serial.print(prevStage);

    Serial.print("Parachute:");
    Serial.print(flight_data.parachute_state);
    Serial.print(", ");
    if(parachute_arm) {
        Serial.print("ARMED ");
    }

    Serial.print(flight_data.flight_time);
    Serial.print("ms, ");
  
    oldalt = flight_data.alt;

#endif

#if ENABLE_SERVO && ENABLE_BAROMETER
    if(flight_data.alt >= ARM_DIST) {
        parachute_arm = 1;
    }
    // Update maximum altitude if higher than the current maximum
    if (flight_data.alt > max_alt_height) {
        max_alt_height = flight_data.alt;
        max_alt_time = flight_data.flight_time;
    }
    // Check if the altitude has dropped significantly (3m) during a 1s time frame
    if (flight_data.alt + FALL_DIST < max_alt_height && flight_data.parachute_state == 0 && parachute_arm) { //&& in_flight == 1
        flight_data.parachute_state = 1; // Deploy parachute
    }
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
    Serial.println();
    while(millis()-lastLoop < 1000/sampleRate) {}
    lastLoop = millis();
}
