#include "gnss.h"
#include "config.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

namespace gnss {
    TinyGPSPlus gnss;
    HardwareSerial& GNSS = Serial2;

    int setup() {
        GNSS.begin(GNSS_BAUD_RATE);
        return 0; 
    }

    void read() {
        while (GNSS.available() > 0) {
            gnss.encode(GNSS.read());
        }
        if (gnss.location.isUpdated() && gnss.location.isValid()) {
           //TODO: Handle updated location
        }
    }

    void end() {
        GNSS.end();
    }

}