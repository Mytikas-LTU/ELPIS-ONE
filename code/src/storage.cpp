#include<storage.h>
#include<basicIO.h>
#include<stage_recognition.h>
#include<SD.h>

const int chipSelect = SS1;

Storage::Storage(){
    ptr = 0;
}

void Storage::init() {
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
    Serial.println("Cardwriter Disabled");
#endif
}

void Storage::writeHeader(telemetry* data){
#if ENABLE_LOGGING
    file.print("Sample rate: ");
    file.print(sampleRate);
    file.print(" Base pressure: ");
    file.println(data->base_pres);
    file.flush();
    flash(4);
    digitalWrite(LED_PIN,LOW);
#else
    Serial.println("Logging disabled, closing file");
    file.close();
#endif
}

void Storage::write(telemetry* data){
#if ENABLE_LOGGING
    file.print("Local Acceleration: ");
    file.print(data->acc.x);
    file.print(", ");
    file.print(data->acc.y);
    file.print(", ");
    file.println(data->acc.z);

    file.print("Global Acceleration: ");
    file.print(data->rotAcc.x);
    file.print(", ");
    file.print(data->rotAcc.y);
    file.print(", ");
    file.println(data->rotAcc.z);

    file.print("Quaternion Rotation: ");
    file.print(data->rot.i);
    file.print(", ");
    file.print(data->rot.j);
    file.print(", ");
    file.print(data->rot.k);
    file.print(", ");
    file.println(data->rot.r);

    file.println("Other data:");
    file.print(data->pres - data->base_pres*100);
    file.print(" Pa, ");

    file.print(data->alt);
    file.print(" m, ");

    file.print("Parachute:");
    file.print(data->parachute_state);
    file.print(", ");

    file.print(data->flight_time);
    file.print("ms, ");

    file.flush();
#endif
}
