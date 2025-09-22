#include "bmp.h"
#include "config.h"

#include <Arduino.h>
#include <Adafruit_BMP3XX.h>

namespace bmp {
    Adafruit_BMP3XX bmp;

    int setup_bmp() {
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

    void read_bmp() {
        if (!bmp.performReading()) {
            Serial.println("Failed to perform reading :(");
            return;
        }
        //TODO
    }

}