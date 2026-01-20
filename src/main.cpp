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
#include "mic.h"    
#include <stdint.h>


// Path to SD log file
const char* SD_LOG_PATH = "/logs/descent.txt";
const char* SD_MIC_PATH = "/logs/acoustic.txt";

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
const unsigned long MIN_FREEFALL_TIME_MS = 3000;    // example: 3 seconds after arming
const unsigned long MIN_DESCENT_TIME_MS = 10000; // 10 seconds
unsigned long descentStartMs = 0;
const unsigned long COMMAND_TIMEOUT_MS   = 22000;   // 22 seconds
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

//Detonation event structure definition
struct DetonationEvent {
    uint8_t eventTime;   // seconds since ACTIVE start
    uint8_t  peak;
    uint8_t  rms;
    uint16_t duration;    // ms
};
static volatile uint8_t detEventCount = 0;
static DetonationEvent detEvents[4]; // store last 4 sound events in struct detEvents, when communication window communicate those

//HELPER FUNCTIONS
static bool commandReceived = false;
static uint8_t lastCmdByte = 0;

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

// MICROPHONE EVENT PROCESSING //
// Pushing the detonation events to the detEvents struct which keeps last 4 events
void pushDetonationEvent(const DetonationEvent& e) {
    noInterrupts();
    if (detEventCount < 4) {
        detEvents[detEventCount++] = e;
    } else {
        detEvents[0] = detEvents[1];
        detEvents[1] = detEvents[2];
        detEvents[2] = detEvents[3];
        detEvents[3] = e;
    }
    interrupts();
}

// Main event processing function
void processSoundEvents() {
    static bool isEventActive = false;
    static uint32_t eventStartTime = 0;
    static uint32_t lastLoudSampleTime = 0;
    static int16_t eventMaxPeak = 0;
    static double sumSquares = 0;
    static uint32_t sampleCount = 0;

    // Continuous 2kHz Downsampling
    static uint8_t downsampleCounter = 0;
    const uint8_t DOWNSAMPLE_FACTOR = 22;

    // Debug timing
    static uint32_t lastDebugPrint = 0;

    // Tunables
    const int SOUND_THRESHOLD = 150; //362 before
    const uint32_t COOLDOWN_MS = 50;

    int16_t tempBuffer[128];

    // --- OPEN RAW FILE ONCE PER CALL (NOT INSIDE THE LOOP) ---
    File rawFile = SD.open("/logs/mic_2khz.bin", FILE_WRITE);

    // Read all available samples
    while (mic::availableSamples() > 0) {
        size_t readCount = mic::readBuffer(tempBuffer, 128);
        uint32_t now = millis();

        for (size_t i = 0; i < readCount; i++) {
            int16_t sample = tempBuffer[i];
            uint16_t absVal = (uint16_t)abs(sample);

            // 1) RAW 2kHz LOGGING (downsample from 44.1kHz -> ~2kHz)
            downsampleCounter++;
            if (downsampleCounter >= DOWNSAMPLE_FACTOR) {
                if (rawFile) rawFile.write((const uint8_t*)&sample, sizeof(int16_t));
                downsampleCounter = 0;
            }

            // 2) PEAK TRACKING (for debug + event metadata)
            if ((int16_t)absVal > eventMaxPeak) eventMaxPeak = (int16_t)absVal;

            // 3) LOUD EVENT DETECTION
            if (absVal > SOUND_THRESHOLD) {
                lastLoudSampleTime = now;

                if (!isEventActive) {
                    isEventActive = true;
                    eventStartTime = now;
                    sumSquares = 0.0;
                    sampleCount = 0;
                }

                sumSquares += (double)sample * (double)sample;
                sampleCount++;
            }
        }
    }

    // Close the raw file once (optional: you can keep it open globally instead)
    if (rawFile) rawFile.close();

    // --- DEBUG OUTPUT (Every 100ms) ---
    uint32_t now = millis();
    // if (now - lastDebugPrint >= 100) {
    //     // Heartbeat so you KNOW the function runs in DESCENT
    //     Serial.print("[MIC] peak=");
    //     Serial.print(eventMaxPeak);

    //     if (eventMaxPeak > 0) {
    //         float dbfs = 20.0f * log10f((float)eventMaxPeak / 32767.0f);
    //         float dbSPL = dbfs + 120.0f;

    //         Serial.print("\tSPL=");
    //         Serial.print(dbSPL, 1);
    //         Serial.print(" dB\t[");

    //         for (int i = 0; i < (eventMaxPeak / 2000); i++) Serial.print("=");
    //         Serial.println("]");
    //     } else {
    //         Serial.println();
    //     }

    //     lastDebugPrint = now;

    //     // If not in an active loud event, reset peak for the next 100ms window
    //     if (!isEventActive) eventMaxPeak = 0;
    // }

    // 4) LOG SUMMARY EVENT TO SD (after cooldown)
    if (isEventActive && (now - lastLoudSampleTime > COOLDOWN_MS)) {
        // Guard against divide-by-zero
        if (sampleCount == 0) {
            isEventActive = false;
            eventMaxPeak = 0;
            return;
        }

        DetonationEvent e;
        e.eventTime = (uint8_t)((eventStartTime - activeStartMs) / 1000);
        e.peak      = (uint8_t)(eventMaxPeak / 128);
        e.rms       = (uint8_t)(sqrt(sumSquares / (double)sampleCount) / 128);
        e.duration  = (uint16_t)(lastLoudSampleTime - eventStartTime);

        File f = SD.open(SD_MIC_PATH, FILE_WRITE);
        if (f) {
            f.printf("%u,%u,%u,%u\n", e.eventTime, e.peak, e.rms, e.duration);
            f.close();
        }

        // Serial.println("!!! LOUD EVENT SAVED !!!");

        isEventActive = false;
        eventMaxPeak = 0;
        pushDetonationEvent(e);

    }
}

// Lora calls this function to get last 4 detonation events recorded in communication window 
uint8_t getDetonationEvents(DetonationEvent out[4]) {
    noInterrupts();
    uint8_t n = detEventCount;
    for (uint8_t i = 0; i < n; i++) out[i] = detEvents[i];
    interrupts();
    return n;
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
    delay(1000);
    Serial.println("hello");
    interface::setup();
    bool ok = true;
    Serial.println("before lora");
    ok &= (lora::setup() == 0);
    gnss::setup();            // if it returns int, change to: ok &= (gnss::setup() == 0);
    imu::setup();             // same
    ok &= (bmp::setup() == 0);
    pm::setup();              // same
    actuator::setup();
    actuator::undeploy();
    mic::setup(16384);

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
    Serial.println("setup finished");

}


//ACTIVE// 
void handleActive() {
    static bool armedLatched = false;   // prevents re-arming logic from running again

    unsigned long now = millis();
    mic::discardBuffer();

    // If ARM button pressed -> go ARMED and turn on yellow LED
    if (interface::armPressed()) {
        unsigned long now = millis();
        Serial.println("[ACTIVE] ARM button pressed -> ARMED");
        status = Status::ARMED;
        armTimeMs = now;                 
        freefallAccelCount = 0;          
        altitudeDropCount = 0;
        hasLastAltitude = false;
        interface::setArmedLed(true);
    }
}
   

//ARMED//
void handleArmed() {
    unsigned long now = millis();

    //  freefall timing condition
    if (now - armTimeMs < MIN_FREEFALL_TIME_MS) {
        static unsigned long lastWaitPrint = 0;
        if (now - lastWaitPrint > 500) {  // 2 Hz
            lastWaitPrint = now;
            Serial.print("[ARMED] Waiting before freefall logic: ");
            Serial.print(MIN_FREEFALL_TIME_MS - (now - armTimeMs));
            Serial.println(" ms remaining");
        }
        return;  
    }

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
        Serial.println("ACTUATOR DEPLOYED");
        status = Status::DESCENT;
        descentStartMs = now;
        freefallAccelCount = 0;
        altitudeDropCount = 0;
        lastCommandMs = now;
        return;
    }

    mic::discardBuffer();

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

}
//DESCENT//
void handleDescent() {
    static uint32_t lastHzTick  = 0;
    static uint32_t lastPrint   = 0;

    // PM sampling timer + last valid reading cache
    static uint32_t lastPmRead = 0;
    static pm::Reading lastPmReading;

    uint32_t now = millis();

    // Time since entering DESCENT (used to enable touchdown detection)
    bool descentTimeOk = (descentStartMs != 0) && (now - descentStartMs >= MIN_DESCENT_TIME_MS);

    // HIGH-FREQUENCY POLLING (runs every loop)
    gnss::update();
    pm::update();
    processSoundEvents();  // 2 kHz .bin logging + event detection

    // 1 Hz block
    if (now - lastHzTick >= 1000) {
        lastHzTick = now;

        // Read sensors at 1 Hz
        bmp::Reading b = bmp::read();
        sensors_event_t accel, gyro, mag, temp;
        imu::read(accel, gyro, mag, temp);

        // PM SENSOR (2-second sampling)
        bool pmReady = false;
        if (now - lastPmRead >= 2000) {
            lastPmRead = now;

            pm::Reading pmr = pm::read();
            if (pmr.valid) {
                lastPmReading = pmr;
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
        bmpHistory[bmpHistoryIndex] = { now, b.temperature, b.pressure, b.altitude };
        bmpHistoryIndex = (bmpHistoryIndex + 1) % BMP_HISTORY_SIZE;

        // GNSS used at 1 Hz (parsing is continuous above)
        gnss::Location loc = getEnrichedLocation(b.altitude);

        // Build sample (1 Hz)
        Sample s;
        s.timestampMs = loc.timestamp;
        s.temperature = b.temperature;
        s.pressure    = b.pressure;
        s.altitude    = b.altitude;

        if (pmReady) {
            s.pm2_5  = lastPmReading.pm2_5;
            s.pm10_0 = lastPmReading.pm10_0;
        } else {
            s.pm2_5  = lastPmReading.valid ? lastPmReading.pm2_5  : -1;
            s.pm10_0 = lastPmReading.valid ? lastPmReading.pm10_0 : -1;
        }

        flash_storeSample(s);

        // DEBUG: build + print science packet every 20 seconds
//         static uint32_t lastDbg = 0;
//         if (now - lastDbg >= 20000) {
//         lastDbg = now;
//         lora::debugSciencePacket(s);   // prints RAW + decoded
// }

        // Touchdown detection (only after minimum descent time)
        // if (descentTimeOk && detectTouchdown(b.altitude)) {
            // Serial.println("[DESCENT] TOUCHDOWN detected");

            // Finalize microphone logging
            // mic::stop();

            // status = Status::TOUCHDOWN;
            // return;
        // }

        // LoRa / SD flush handling (1 Hz)
        uint8_t cmdByte = 0;
        bool command = lora::receiveCommand(cmdByte);

        if (command) {
            Serial.print("[CMD] cmdByte=0x");
            Serial.println(cmdByte, HEX);

            uint8_t team = cmdByte & 0x0F;
            if (team != (TEAM_ID & 0x0F)) {
                Serial.print("[CMD] Not for us, team=");
                Serial.println(team);
            } else {
                bool reqSci = (cmdByte & (1u << 4));
                bool reqTel = (cmdByte & (1u << 5));

                delay(60); // >= 50 ms before responding

                if (reqSci) {
                    // lora::sendScience(s);
                }
                // if (reqTel) { lora::sendTelemetry(loc, b.altitude); }

                lastCommandMs = now;
                flash_flushToSD();
            }
        } else {
            if (now - lastCommandMs > COMMAND_TIMEOUT_MS) {
                flash_flushToSD();
                lastCommandMs = now;
            }
        }
        
    }

    // 1 Hz debug print (rich version + simple tick)
    if (now - lastPrint >= 1000) {
        lastPrint = now;
        Serial.print("[DESCENT] tick | dt=");
        Serial.print(descentStartMs ? (now - descentStartMs) : 0);
        Serial.print(" ms | touchdown_enabled=");
        Serial.println(descentTimeOk ? "YES" : "NO");
    }
}

// void handleDescent() {
//     static uint32_t lastHzTick  = 0;
//     static uint32_t lastPrint   = 0;
//     static volatile bool cmdPending = false;
//     static uint8_t pendingCmdByte = 0;
//     static uint32_t pendingCmdMs = 0;

//     // PM sampling timer + last valid reading cache
//     static uint32_t lastPmRead = 0;
//     static pm::Reading lastPmReading;

//     uint32_t now = millis();

//     // Time since entering DESCENT (used to enable touchdown detection)
//     bool descentTimeOk = (descentStartMs != 0) && (now - descentStartMs >= MIN_DESCENT_TIME_MS);

//     // HIGH-FREQUENCY POLLING (runs every loop)
//     gnss::update();
//     pm::update();
//     processSoundEvents();  // 2 kHz .bin logging + event detection

//     uint8_t cmd = 0;
//     if (lora::receiveCommand(cmd)) {
//     pendingCmdByte = cmd;
//     pendingCmdMs = now;
//     cmdPending = true;

//     Serial.print("[CMD RX] cmdByte=0x");
//     Serial.println(cmd, HEX);
// }
//     // 1 Hz block
//     if (now - lastHzTick >= 1000) {
//         lastHzTick = now;

//         // Read sensors at 1 Hz
//         bmp::Reading b = bmp::read();
//         sensors_event_t accel, gyro, mag, temp;
//         imu::read(accel, gyro, mag, temp);

//         // PM SENSOR (2-second sampling)
//         bool pmReady = false;
//         if (now - lastPmRead >= 2000) {
//             lastPmRead = now;

//             pm::Reading pmr = pm::read();
//             if (pmr.valid) {
//                 lastPmReading = pmr;
//                 pmReady = true;
//             } else {
//                 Serial.println("[DESCENT] PM read failed");
//             }
//         }

//         if (!b.valid) {
//             Serial.println("[DESCENT] BMP read failed");
//             return;
//         }

//         // Store BMP history (1 Hz)
//         bmpHistory[bmpHistoryIndex] = { now, b.temperature, b.pressure, b.altitude };
//         bmpHistoryIndex = (bmpHistoryIndex + 1) % BMP_HISTORY_SIZE;

//         // GNSS used at 1 Hz (parsing is continuous above)
//         gnss::Location loc = getEnrichedLocation(b.altitude);

//         // Build sample (1 Hz)
//         Sample s;
//         s.timestampMs = loc.timestamp;
//         s.temperature = b.temperature;
//         s.pressure    = b.pressure;
//         s.altitude    = b.altitude;

//         if (pmReady) {
//             s.pm2_5  = lastPmReading.pm2_5;
//             s.pm10_0 = lastPmReading.pm10_0;
//         } else {
//             s.pm2_5  = lastPmReading.valid ? lastPmReading.pm2_5  : -1;
//             s.pm10_0 = lastPmReading.valid ? lastPmReading.pm10_0 : -1;
//         }

//         flash_storeSample(s);

//         // DEBUG: build + print science packet every 20 seconds
// //         static uint32_t lastDbg = 0;
// //         if (now - lastDbg >= 20000) {
// //         lastDbg = now;
// //         lora::debugSciencePacket(s);   // prints RAW + decoded
// // }

//         // Touchdown detection (only after minimum descent time)
//         // if (descentTimeOk && detectTouchdown(b.altitude)) {
//             // Serial.println("[DESCENT] TOUCHDOWN detected");

//             // Finalize microphone logging
//             // mic::stop();

//             // status = Status::TOUCHDOWN;
//             // return;
//         // }

//         // LoRa / SD flush handling (1 Hz)
//         uint8_t cmdByte = 0;
//         bool command = lora::receiveCommand(cmdByte);

//         if (cmdPending) {
//             cmdPending = false;
//             uint8_t cmdByte = pendingCmdByte;

//             uint8_t team = cmdByte & 0x0F;
//             if (team != (TEAM_ID & 0x0F)) {
//                 Serial.print("[CMD] Not for us, team=");
//                 Serial.println(team);
//             } else {
//                 bool reqSci = (cmdByte & (1u << 4));
//                 bool reqTel = (cmdByte & (1u << 5));

//                 delay(60); // >= 50 ms before responding (spec)

//                 if (reqSci) lora::sendScience(s);
//                 // if (reqTel) lora::sendTelemetry(loc, b.altitude);

//                 lastCommandMs = now;
//                 flash_flushToSD();
//             }
//         }
                
//             }

//             // 1 Hz debug print (rich version + simple tick)
//             if (now - lastPrint >= 1000) {
//                 lastPrint = now;
//                 Serial.print("[DESCENT] tick | dt=");
//                 Serial.print(descentStartMs ? (now - descentStartMs) : 0);
//                 Serial.print(" ms | touchdown_enabled=");
//                 Serial.println(descentTimeOk ? "YES" : "NO");
//             }
// }

//TOUCHDOWN//
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
    uint8_t cmdByte = 0;
    bool command = lora::receiveCommand(cmdByte);

    if (command) {
        uint8_t team = cmdByte & 0x0F;
        if (team != (TEAM_ID & 0x0F)) {
            Serial.print("[CMD] Not for us, team="); Serial.println(team);
            return;  // or ignore
        }

        Serial.print("[CMD] cmdByte=0x"); Serial.println(cmdByte, HEX);

        bool reqSci = (cmdByte & (1u << 4));
        bool reqTel = (cmdByte & (1u << 5));

        delay(60); // >= 50 ms

        if (reqTel) {
            // lora::sendTelemetry(loc, bmpAlt);
        }
        if (reqSci) {
            lora::sendScience(s);
        }

        lastCommandMs = now;
        flash_flushToSD();
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

// Status
    switch (status) {
        case Status::ACTIVE:    handleActive();    break;
        case Status::ARMED:     handleArmed();     break;
        case Status::DESCENT:   handleDescent();   break;
        case Status::TOUCHDOWN: handleTouchdown(); break;
    }
}





