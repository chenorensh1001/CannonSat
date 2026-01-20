#include <Arduino.h>
#include <TinyGPS++.h>
#include "gnss.h"
#include "config.h"
#include <TimeLib.h>


namespace gnss {

static TinyGPSPlus gps;
HardwareSerial& GNSS = Serial1;

int setup() {
    GNSS.begin(GNSS_BAUD_RATE);
    return 0;
}

uint32_t convertToUnix(TinyGPSDate& d, TinyGPSTime& t) {
    if (!d.isValid() || !t.isValid()) return 0;

    tmElements_t tm;
    tm.Year   = d.year() - 1970;
    tm.Month  = d.month();
    tm.Day    = d.day();
    tm.Hour   = t.hour();
    tm.Minute = t.minute();
    tm.Second = t.second();

    return makeTime(tm);
}


Location read() {
    while (GNSS.available() > 0) {
        gps.encode(GNSS.read());
    }

    Location loc;
    loc.valid = false;
    loc.timestamp = 0;
    loc.verticalVelocity = 0;

    if (gps.location.isUpdated() && gps.location.isValid()) {
        loc.latitude  = gps.location.lat();
        loc.longitude = gps.location.lng();
        loc.altitude  = gps.altitude.meters();
        loc.speed     = gps.speed.mps();
        loc.course    = gps.course.deg();
        loc.valid     = true;
    }

    // Add timestamp if available
    if (gps.date.isValid() && gps.time.isValid()) {
        loc.timestamp = convertToUnix(gps.date, gps.time);
    }

    return loc;
}

void update() {
    while (GNSS.available() > 0) {
        gps.encode(GNSS.read());
    }
}


void end() {
    GNSS.end();
}


// // void setup() {
// //     Serial.begin(9600);
// //     Serial.println("=== GNSS PASSTHROUGH TEST ===");

// //     Serial1.begin(9600);   // try 38400 later if needed
// // }

// // void loop() {
// //     // Serial.println("=== GNSS PASSTHROUGH TEST ===");
// //     while (Serial1.available()) {
// //         Serial.write(Serial1.read());
// //     }
// // }


// void setup() {
//     Serial.begin(115200);   // USB to PC
//     Serial1.begin(9600);    // GNSS baud rate (check your module)
// }

// void loop() {
//     // PC → GNSS
//     if (Serial.available()) {
//         Serial1.write(Serial.read());
//     }

//     // GNSS → PC
//     if (Serial1.available()) {
//         Serial.write(Serial1.read());
//     }
// }

}
