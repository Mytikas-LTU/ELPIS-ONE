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
    ptr += 1;
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
}