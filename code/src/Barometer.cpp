#include<stage_recognition.h>
#include<Barometer.h>
#include<basicIO.h>

const int pressureSamples = 10; // do better

Barometer::Barometer(){
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
}

float Barometer::calibrate(){
    float pressures = 0;
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
    float base_pres = (pressures/pressureSamples)/100;
    Serial.print("Calibrated at ");
    Serial.print(base_pres);
    Serial.println(" hPa");
    return base_pres;
}

void Barometer::getData(telemetry* tel){
    tel->temp = bmp.readTemperature();
    tel->pres = bmp.readPressure();
    tel->alt = bmp.readAltitude(tel->base_pres);
}