#include <Arduino.h>
#include <Adafruit_BMP3XX.h>

#include "bmp.h"

namespace bmp {

static Adafruit_BMP3XX bmp;

int setup() {
    if (!bmp.begin_I2C()) {
        Serial.println("Failed to find BMP388");
        return 1; 
    }
    Serial.println("BMP388 found!");

    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    return 0; 
}

Reading read() {
    Reading r;

    if (!bmp.performReading()) {
        Serial.println("Failed to perform BMP388 reading");
        r.valid = false;
        return r;
    }

    r.temperature = bmp.temperature;
    r.pressure    = bmp.pressure;
    r.altitude    = bmp.readAltitude(1013.25); // assuming sea level pressure in hPa
    r.valid       = true;

    return r;
}

}
