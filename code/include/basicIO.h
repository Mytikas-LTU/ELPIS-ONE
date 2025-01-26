#ifndef _IO_
#define _IO_

const int flashTime = 100; //time of a flash of the status LED, in millis
const int sampleRate = 10; //samples per second

const uint8_t LED_PIN = 2; // Define the LED-pin
const uint8_t ERROR_LED_PIN = 3; // Define the LED-pin
const uint8_t LAUNCH_LED_PIN = 4; // Define the LED-pin
const uint8_t CARD_LED_PIN = 5; // Define the LED-pin

#define ENABLE_BAROMETER 1
#define ENABLE_ACCELEROMETER 1
#define ENABLE_CARDWRITER 0
#define ENABLE_LOGGING 0
#define ENABLE_SERVO 1
#define ENABLE_DUMMYDATA 0

const char filename[10] = "tele.txt";

static void flash(int times) {
    for (int i = 0; i<times; i++) {
        digitalWrite(LED_PIN,HIGH);
        delay(flashTime);
        digitalWrite(LED_PIN,LOW);
        delay(flashTime);
    }
}

#endif
