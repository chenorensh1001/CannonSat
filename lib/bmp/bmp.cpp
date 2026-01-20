#include <Arduino.h>
#include <Adafruit_BMP3XX.h>
#include <Wire.h>
#include "bmp.h"

namespace bmp {

static Adafruit_BMP3XX bmp;

int setup() {
    Wire1.begin();
    Wire1.setClock(100000);   // start conservative (100 kHz). You can raise later.

    // Explicit address 0x77 (since scan confirmed it)
    if (!bmp.begin_I2C(0x77, &Wire1)) {
        Serial.println("Failed to init BMP3xx at 0x77 on Wire1");
        return 1;
    }
    Serial.println("BMP3xx init OK at 0x77 on Wire1");

    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    return 0;
}

Reading read() {
    Reading r;

    if (!bmp.performReading()) {
        static unsigned long lastErr = 0;
        unsigned long now = millis();
        if (now - lastErr > 1000) {
            lastErr = now;
            Serial.println("BMP3xx: performReading() failed");
        }
        r.valid = false;
        return r;
    }

    r.temperature = bmp.temperature;          // C
    r.pressure    = bmp.pressure;             // Pa
    r.altitude    = bmp.readAltitude(1013.25);
    r.valid       = true;
    return r;
}

} // namespace bmp
