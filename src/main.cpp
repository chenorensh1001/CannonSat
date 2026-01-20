#include <Arduino.h>
#include <Wire.h>
#include <sample.h>
#include <SD.h>
#include "config.h"
#include "bmp.h"
#include "gnss.h"
#include "imu.h"
#include "lora_driver.h"
#include "mic.h"
#include "sd_driver.h"
#include "pm.h"
#include "interface.h"
#include "linear_actuator.h"

// Path to SD log file
const char* SD_LOG_PATH = "/logs/descent.txt";

// RAM buffer for samples
constexpr int SAMPLE_BUFFER_SIZE = 512;
Sample sampleBuffer[SAMPLE_BUFFER_SIZE];
int sampleBufferIndex = 0;

//STATE MACHINE DFINITION//
enum class Status {
    ACTIVE,
    ARMED,
    DESCENT,
    TOUCHDOWN
};

Status status = Status::ACTIVE;


//GLOBAL VATIABLES AND THRESHOLDS//
// Timing
unsigned long armTimeMs = 0; // logging the time the armed button was pressed
unsigned long lastCommandMs = 0; // logging the exact time the command packet was last received
const unsigned long MIN_FREEFALL_TIME_MS = 1000;    // example: 3 seconds after arming
const unsigned long COMMAND_TIMEOUT_MS   = 20000;   // 40 seconds
static bool gnssEverValid = false; // variables for syncing the teensy internal clock and the GNSS
static uint32_t lastGnssUnix = 0; // variables for syncing the teensy internal clock and the GNSS
static uint32_t lastGnssMillis = 0; // variables for syncing the teensy internal clock and the GNSS
unsigned long activeStartMs = 0; // variables for syncing the teensy internal clock and the GNSS
bool actuatorDeployed = false;


// Freefall detection counters
int freefallAccelCount   = 0; 
int altitudeDropCount    = 0;

// Thresholds 
const float FREEFALL_ACCEL_THRESHOLD = 11.0f;   // m/s², magnitude below this = near freefall
const float ALTITUDE_DROP_MIN        = 0.02f;   // meters drop between samples - set to 1 m between samples (which are sampled at 0.05 seconds)
const int   FREEFALL_ACCEL_SAMPLES   = 5;     // consecutive samples detect freefalll
const int   ALTITUDE_DROP_SAMPLES    = 3;      // consecutive

// For altitude trend
float lastAltitude = 0.0f;
bool  hasLastAltitude = false;

// Touchdown detection
const float TOUCHDOWN_ACCEL_THRESHOLD = 20.0f; // m/s² spike or similar, to tune

//CMP history buffer//
constexpr int BMP_HISTORY_SIZE = 200; // 20 seconds at 10 Hz
BmpSample bmpHistory[BMP_HISTORY_SIZE];
int bmpHistoryIndex = 0;


//HELPER FUNCTIONS
    //TIMESTAMPING//
uint32_t getUnifiedTimestamp(uint32_t gnssTimestamp) {
    static bool gnssEverValid = false;
    static uint32_t lastGnssUnix = 0;
    static uint32_t lastGnssMillis = 0;

    uint32_t nowMs = millis();

    // Case 1: GNSS timestamp available
    if (gnssTimestamp != 0) {
        gnssEverValid = true;
        lastGnssUnix = gnssTimestamp;
        lastGnssMillis = nowMs;
        return gnssTimestamp;
    }

    // Case 2: GNSS lost but we had a fix before
    if (gnssEverValid) {
        return lastGnssUnix + (nowMs - lastGnssMillis) / 1000;
    }

    // Case 3: GNSS never valid → fallback to internal clock
    return nowMs / 1000;
}

// GNSS
gnss::Location getEnrichedLocation(float bmpAltitude) {
    gnss::Location raw = gnss::read();
    gnss::Location loc = raw;   // copy base fields
    loc.timestamp = getUnifiedTimestamp(raw.timestamp);

    uint32_t nowMs = millis();

    // -------------------------------
    // 2. VERTICAL VELOCITY FALLBACK
    // -------------------------------
    static float lastGnssAlt = raw.altitude;   // GNSS altitude
    static float lastBmpAlt  = bmpAltitude;    // BMP altitude
    static uint32_t lastAltMs = nowMs;

    float dt = (nowMs - lastAltMs) / 1000.0f;

    if (dt > 0.05f) {   // avoid division by zero and noise

        if (raw.valid) {
            // GNSS available → use GNSS altitude
            loc.verticalVelocity = (raw.altitude - lastGnssAlt) / dt;
            lastGnssAlt = raw.altitude;
        } else {
            // GNSS unavailable → use BMP altitude
            loc.verticalVelocity = (bmpAltitude - lastBmpAlt) / dt;
            lastBmpAlt = bmpAltitude;
        }

        lastAltMs = nowMs;
    }

    return loc;
}


void flash_storeSample(const Sample& s) {
    if (sampleBufferIndex < SAMPLE_BUFFER_SIZE) {
        sampleBuffer[sampleBufferIndex++] = s;

        Serial.print("Buffered sample ");
        Serial.print(sampleBufferIndex);
        Serial.print(": t=");
        Serial.print(s.timestampMs);
        Serial.print(" ms, T=");
        Serial.print(s.temperature);
        Serial.print(" C, P=");
        Serial.print(s.pressure);
        Serial.print(" Pa, Alt=");
        Serial.print(s.altitude);
        Serial.println(" m");
        Serial.print(", PM10=");
        Serial.print(s.pm10_0);
        Serial.print(" ug/m3, PM2.5=");
        Serial.print(s.pm2_5);
        Serial.println(" ug/m3");


    } else {
        Serial.println("WARNING: Sample buffer FULL — cannot store more samples!");
    }
}


bool detectTouchdown(float currentAltitude) {
    static float lastAlt = NAN;
    static uint32_t stableSince = 0;

    const float ALT_STABLE_BAND = 0.5f;      // meters (tune)
    const uint32_t STABLE_TIME_MS = 3000;    // 3 seconds (tune)

    uint32_t now = millis();

    if (isnan(lastAlt)) {
        lastAlt = currentAltitude;
        stableSince = now;
        return false;
    }

    float dAlt = fabsf(currentAltitude - lastAlt);

    if (dAlt <= ALT_STABLE_BAND) {
        // still stable
        if (now - stableSince >= STABLE_TIME_MS) return true;
    } else {
        // not stable -> reset timer
        stableSince = now;
        lastAlt = currentAltitude;
    }

    return false;
}
//READ SD CARD FUNCTION
void dumpSdFile(const char* filename) {
    Serial.print("Opening file: ");
    Serial.println(filename);

    File f = SD.open(filename);
    if (!f) {
        Serial.println("Failed to open file");
        return;
    }

    Serial.println("----- FILE START -----");
    while (f.available()) {
        Serial.write(f.read());
    }
    Serial.println("\n----- FILE END -----");

    f.close();
}


void flash_flushToSD() {
    if (sampleBufferIndex == 0) {
        Serial.println("No samples to flush.");
        return;
    }

    if (sd::setup() != 0) {
        Serial.println("SD setup failed!");
        return;
    }

    
    // If file does not exist yet → create it and write header
    if (!sd::exists(SD_LOG_PATH)) {
        if (sd::open(SD_LOG_PATH, false)) {   // false = overwrite/create
            sd::writeLine("timestampMs,temperature,pressure,altitude");
            sd::close();
        }
    }
   
    if (!sd::open(SD_LOG_PATH, true)) {
        Serial.println("Failed to open SD log file!");
        return;
    }

    // (B) Add a batch marker before writing samples
    char batchHeader[64];
    snprintf(batchHeader, sizeof(batchHeader), "FLUSH %lu", millis());
    sd::writeLine(batchHeader);
    
    Serial.print("Flushing ");
    Serial.print(sampleBufferIndex);
    Serial.println(" samples to SD...");

    for (int i = 0; i < sampleBufferIndex; i++) {
        const Sample& s = sampleBuffer[i];

        char line[128];
        snprintf(line, sizeof(line),
            "%lu,%.2f,%.2f,%.2f",
            s.timestampMs,
            s.temperature,
            s.pressure,
            s.altitude
        );

        sd::writeLine(line);
    }

    sd::flush();
    sd::close();

    Serial.println("SD flush complete.");
    // dumpSdFile(SD_LOG_PATH);

    sampleBufferIndex = 0;
}

float vec3_mag(float x, float y, float z) {
    return sqrtf(x*x + y*y + z*z);
}

//FUNCTION TO GET SAMPLES N SECONDS AGO
bool getSampleSecondsAgo(float secondsAgo, BmpSample& out) {
    uint32_t targetTime = millis() - (uint32_t)(secondsAgo * 1000);

    for (int i = 0; i < BMP_HISTORY_SIZE; i++) {
        int idx = (bmpHistoryIndex - 1 - i + BMP_HISTORY_SIZE) % BMP_HISTORY_SIZE;
        if (bmpHistory[idx].timestampMs <= targetTime) {
            out = bmpHistory[idx];
            return true;
        }
    }
    return false; // not enough history yet
}

//QA + DEUBGGING FUNCTIONS

//DEBUG GNSS
void debugPrintGnss(const gnss::Location& loc) {
    Serial.println("----- GNSS DEBUG -----");
    Serial.print("Valid: "); Serial.println(loc.valid ? "YES" : "NO");
    Serial.print("Timestamp: "); Serial.println(loc.timestamp);
    Serial.print("Lat: "); Serial.println(loc.latitude, 6);
    Serial.print("Lon: "); Serial.println(loc.longitude, 6);
    Serial.print("Alt: "); Serial.println(loc.altitude);
    Serial.print("Vel (vertical): "); Serial.println(loc.verticalVelocity);
    Serial.println("----------------------");
}


//SETUP//

void setup() {
    Serial.begin(9600);
    Serial1.begin(9600);
    interface::setup();
    bool ok = true;
    gnss::setup();            // if it returns int, change to: ok &= (gnss::setup() == 0);
    imu::setup();             // same
    ok &= (bmp::setup() == 0);
    pm::setup();              // same
    actuator::setup();
    actuator::undeploy();
    mic:setup(16384)

    if (!ok) {
        // ERROR: at least one setup failed -> blink forever
        interface::startSystemBlinking();
        while (1) {
            interface::serviceBlink();   // keep blinking
            delay(10);
        }
    }

    // All good -> steady ON
    interface::stopSystemBlinking();   // this sets ACTIVE LED ON in your implementation
}


//ACTIVE// - without any LEDS or buttons, they will have to be added once arrived. I have a draft for the function that should be implemented when they arrive under this one
void handleActive() {
    static bool armedLatched = false;   // prevents re-arming logic from running again

    unsigned long now = millis();

    // Always update interface (debounce + any LED service you might have)
    interface::update();

    // If ARM button pressed -> go ARMED and turn on yellow LED
    if (!armedLatched && interface::armPressed()) {
        armedLatched = true;

        Serial.println("[ACTIVE] ARM button pressed -> ARMED");
        interface::setArmedLed(true);   // yellow ON

        status = Status::ARMED;
        armTimeMs = now;                // start ARMED timer
        return;
    }
}


//ARMED//

void handleArmed() {
    unsigned long now = millis();

    static unsigned long lastTickPrint = 0;
    if (now - lastTickPrint > 1000) {   // 1 Hz
        lastTickPrint = now;
        Serial.println("TICK: ARMED");
    }

    // Read sensors
    sensors_event_t accel, gyro, mag, temp;
    imu::read(accel, gyro, mag, temp);
    bmp::Reading b = bmp::read();

    // Compute accel magnitude
    float aMag = vec3_mag(accel.acceleration.x,
                          accel.acceleration.y,
                          accel.acceleration.z);

    // ---------------- DEBUG: ICM + BMP output ----------------
    static unsigned long lastPrint = 0;
    if (now - lastPrint > 500) {   // print at 2 Hz
        lastPrint = now;

        // IMU
        Serial.print("ACC [m/s^2]  X=");
        Serial.print(accel.acceleration.x, 3);
        Serial.print("  Y=");
        Serial.print(accel.acceleration.y, 3);
        Serial.print("  Z=");
        Serial.print(accel.acceleration.z, 3);
        Serial.print("  |a|=");
        Serial.print(aMag, 3);

        // BMP
        if (b.valid) {
            Serial.print("  |  ALT=");
            Serial.print(b.altitude, 2);
            Serial.print(" m  P=");
            Serial.print(b.pressure / 100.0, 2); // Pa → hPa
            Serial.print(" hPa  T=");
            Serial.print(b.temperature, 2);
            Serial.println(" C");
        } else {
            Serial.println("  |  BMP INVALID");
        }
    }
    // ---------------------------------------------------------

    // Freefall accel condition
    if (aMag < FREEFALL_ACCEL_THRESHOLD) {
        freefallAccelCount++;
    } else {
        freefallAccelCount = 0;
    }

    // // Altitude drop condition
    // if (b.valid) {
    //     if (hasLastAltitude) {
    //         float deltaAlt = lastAltitude - b.altitude; // positive if descending
    //         if (deltaAlt > ALTITUDE_DROP_MIN) {
    //             altitudeDropCount++;
    //         } else {
    //             altitudeDropCount = 0;
    //         }
    //     }
    //     lastAltitude = b.altitude;
    //     hasLastAltitude = true;
    // }

    bool accelOk = (freefallAccelCount >= FREEFALL_ACCEL_SAMPLES);
    bool altOk   = true ; //(altitudeDropCount  >= ALTITUDE_DROP_SAMPLES);
    bool timeOk  = (now - armTimeMs >= MIN_FREEFALL_TIME_MS);

    if (accelOk && altOk && timeOk) {
        Serial.println("Freefall detected → DESCENT");
        actuator::trigger(); // deploy
        Serial.println("ACTUATOR DE");
        status = Status::DESCENT;
        freefallAccelCount = 0;
        altitudeDropCount = 0;
        lastCommandMs = now;
    }

        // ---------------- DEBUG: Deployment conditions ----------------
    static unsigned long lastCondPrint = 0;
    if (now - lastCondPrint > 500) {   // 2 Hz, readable
        lastCondPrint = now;

        Serial.print("DEPLOY CHECK | ");

        Serial.print("ACC=");
        Serial.print(accelOk ? "OK" : "NO");
        Serial.print(" (");
        Serial.print(freefallAccelCount);
        Serial.print("/");
        Serial.print(FREEFALL_ACCEL_SAMPLES);
        Serial.print(") | ");

        Serial.print("ALT=");
        Serial.print(altOk ? "OK" : "NO");
        Serial.print(" (");
        Serial.print(altitudeDropCount);
        Serial.print("/");
        Serial.print(ALTITUDE_DROP_SAMPLES);
        Serial.print(") | ");

        Serial.print("TIME=");
        Serial.print(timeOk ? "OK" : "NO");
        Serial.print(" (");
        Serial.print(now - armTimeMs);
        Serial.print(" ms)");

        Serial.println();
    }
    // ---------------------------------------------------------------

    // delay(50); // sampling rate during ARMED
}

//DESCENT//
void handleDescent() {
    static uint32_t lastHzTick = 0;
    static uint32_t lastPrint = 0;

    // NEW: PM sampling timer
    static uint32_t lastPmRead = 0;
    static pm::Reading lastPmReading;   // store last valid reading

    // Serial.println("start");
    uint32_t now = millis();

    // GNSS must always be updated continuously
    gnss::update();
    pm::update();

    // 1 Hz block
    if (now - lastHzTick >= 1000) {
        lastHzTick = now;

        // Read sensors at 1 Hz
        bmp::Reading b = bmp::read();
        sensors_event_t accel, gyro, mag, temp;
        imu::read(accel, gyro, mag, temp);

        // --- PM SENSOR (2-second sampling) ---
        pm::Reading pmr;
        bool pmReady = false;

        if (now - lastPmRead >= 2000) {
            lastPmRead = now;
            pmr = pm::read();

            if (pmr.valid) {
                lastPmReading = pmr;   // store the newest valid reading
                pmReady = true;
            } else {
                Serial.println("[DESCENT] PM read failed");
            }
        }

        if (!b.valid) {
            Serial.println("[DESCENT] BMP read failed");
            return;
        }

        // Store BMP history (1 Hz)
        bmpHistory[bmpHistoryIndex] = {
            now,
            b.temperature,
            b.pressure,
            b.altitude,
        };
        bmpHistoryIndex = (bmpHistoryIndex + 1) % BMP_HISTORY_SIZE;

        // GNSS (continuous parsing, but used at 1 Hz)
        gnss::Location loc = getEnrichedLocation(b.altitude);

        // Build sample (1 Hz)
        Sample s;
        s.timestampMs = loc.timestamp;
        s.temperature = b.temperature;
        s.pressure    = b.pressure;
        s.altitude    = b.altitude;
        if (pmReady) {
            // s.pm1_0  = lastPmReading.pm1_0;
            s.pm2_5  = lastPmReading.pm2_5;
            s.pm10_0 = lastPmReading.pm10_0;
        } else {
            // No new PM packet this second → reuse last valid or mark invalid
            // s.pm1_0  = lastPmReading.valid ? lastPmReading.pm1_0  : -1;
            s.pm2_5  = lastPmReading.valid ? lastPmReading.pm2_5  : -1;
            s.pm10_0 = lastPmReading.valid ? lastPmReading.pm10_0 : -1;
        }

        // Serial.print("PM10 = ");
        // Serial.println(s.pm10_0);

        flash_storeSample(s);

        // Touchdown detection (1 Hz IMU + BMP)
        if (detectTouchdown(b.altitude)) {
            Serial.println("[DESCENT] TOUCHDOWN detected");
            status = Status::TOUCHDOWN;
            return;
        }

        // LoRa command handling (1 Hz)
        bool command = false;  // indoor test
        if (command) {
            lastCommandMs = now;
            lora::sendScience(s);
            flash_flushToSD();
        } else {
            if (now - lastCommandMs > COMMAND_TIMEOUT_MS) {
                flash_flushToSD();
                lastCommandMs = now;
            }
        }
    }

    // 1 Hz debug print
    if (now - lastPrint >= 1000) {
        lastPrint = now;
        Serial.println("[DESCENT] tick");
    }
}


void handleTouchdown() {
    static unsigned long lastPrint = 0;
    unsigned long now = millis();
    gnss::update();
    pm::update();

    // Read sensors
    bmp::Reading b = bmp::read();
    sensors_event_t accel, gyro, mag, temp;
    imu::read(accel, gyro, mag, temp);
    pm::Reading pmr = pm::read();

    gnss::Location loc = getEnrichedLocation(b.altitude);

    // Build sample
    Sample s;
    s.timestampMs = loc.timestamp;
    s.temperature = b.temperature;
    s.pressure    = b.pressure;
    s.altitude    = b.altitude;

    // Add PM readings
    if (pmr.valid) {
        s.pm10_0 = pmr.pm10_0;
        s.pm2_5  = pmr.pm2_5;
    } else {
        s.pm10_0 = -1;
        s.pm2_5  = -1;
    }

    // LoRa command handling
    bool command = false;  // indoor test
    if (command) {
        lastCommandMs = now;
        lora::sendScience(s);
        flash_flushToSD();
    } else {
        if (now - lastCommandMs > COMMAND_TIMEOUT_MS) {
            flash_flushToSD();
            lastCommandMs = now;
        }
    }

    // Non-blocking tick
    if (now - lastPrint > 1000) {
        lastPrint = now;
        Serial.println("[TOUCHDOWN] tick");
    }
}



void loop() {
    // Feed GNSS heavily before running any state logic so we get some samples
    for (int i = 0; i < 50; i++) {
        gnss::update();
    }

    actuator::update();
    interface::update();

    // RESET works from any state
    if (interface::resetPressed()) {
        status = Status::ACTIVE;
        interface::setArmedLed(false);
        freefallAccelCount = 0;
        altitudeDropCount = 0;
        hasLastAltitude = false;
        lastAltitude = 0;
        armTimeMs = millis();
        actuatorDeployed = false;  // optional depending on your mission rules

        Serial.println("RESET → ACTIVE");
    }

    // ARM button only works from ACTIVE
    if (interface::armPressed() && status == Status::ACTIVE) {
        status = Status::ARMED;
        interface::setArmedLed(true);
        Serial.println("ACTIVE → ARMED");
    }

// Status
    switch (status) {
        case Status::ACTIVE:    handleActive();    break;
        case Status::ARMED:     handleArmed();     break;
        case Status::DESCENT:   handleDescent();   break;
        case Status::TOUCHDOWN: handleTouchdown(); break;
    }
}





