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

    Location loc{};
    loc.valid = false;

    static bool haveFix = false;
    static double lastLat = 0.0, lastLon = 0.0;
    static float  lastAlt = 0.0f;
    static float  lastSpd = 0.0f;
    static float  lastCrs = 0.0f;
    
    // Track last GPS sync time
    static unsigned long lastGpsSync = 0;
    static unsigned long lastGpsTimestamp = 0;

    if (gps.location.isValid()) {
        loc.latitude  = gps.location.lat();
        loc.longitude = gps.location.lng();

        loc.altitude  = gps.altitude.isValid() ? gps.altitude.meters() : 0.0f;
        loc.speed     = gps.speed.isValid()    ? gps.speed.mps()      : 0.0f;
        loc.course    = gps.course.isValid()   ? gps.course.deg()     : 0.0f;

        loc.valid = true;

        haveFix = true;
        lastLat = loc.latitude;
        lastLon = loc.longitude;
        lastAlt = loc.altitude;
        lastSpd = loc.speed;
        lastCrs = loc.course;
    } else if (haveFix) {
        loc.latitude  = lastLat;
        loc.longitude = lastLon;
        loc.altitude  = lastAlt;
        loc.speed     = lastSpd;
        loc.course    = lastCrs;
        loc.valid     = false;
    }

    // Timestamp logic with GPS sync and Teensy fallback
    if (gps.date.isValid() && gps.time.isValid() && gps.date.year() > 2020) {
        // GPS time is valid AND reasonable - use it and sync our reference
        loc.timestamp = convertToUnix(gps.date, gps.time);
        lastGpsTimestamp = loc.timestamp;
        lastGpsSync = millis();
    } else if (lastGpsSync > 0) {
        // GPS lost but we had it before - calculate elapsed time
        unsigned long elapsed = (millis() - lastGpsSync) / 1000; // seconds
        loc.timestamp = lastGpsTimestamp + elapsed;
    } else {
        // Never had GPS or GPS date is invalid (year 2000) - return 0
        loc.timestamp = 0;
    }

    Serial1.println("[GNSS DEBUG] returning timestamp = "); 
    Serial.println(loc.timestamp);
    return loc;
}

void update() {
    while (GNSS.available() > 0) {
        gps.encode((char)GNSS.read());
    }
}

void end() {
    GNSS.end();
}

} // namespace gnss



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

