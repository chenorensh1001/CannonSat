#include "gnss.h"
#include "config.h"
#include <Arduino.h>
#include <TinyGPS++.h>

namespace gnss {

static TinyGPSPlus gps;
HardwareSerial& GNSS = Serial2;

int setup() {
    GNSS.begin(GNSS_BAUD_RATE);
    return 0;
}

Location read() {
    while (GNSS.available() > 0) {
        gps.encode(GNSS.read());
    }

    Location loc;
    if (gps.location.isUpdated() && gps.location.isValid()) {
        loc.latitude  = gps.location.lat();
        loc.longitude = gps.location.lng();
        loc.altitude  = gps.altitude.meters();
        loc.speed     = gps.speed.mps();
        loc.course    = gps.course.deg();
        loc.valid     = true;
    }
    return loc;
}

void end() {
    GNSS.end();
}

}
