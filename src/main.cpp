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
#include <TinyGPS++.h>


// Path to SD log file
const char* SD_LOG_PATH = "/logs/descent.txt";
const char* SD_MIC_PATH = "/logs/acoustic.txt";
//SD file config
static File micRawFile;
static File micEventFile;

void micLogOpen() {
  micRawFile   = SD.open("/logs/mic_2khz.bin", FILE_WRITE);
  micEventFile = SD.open("/logs/acoustic.txt", FILE_WRITE);
}


// RAM buffer for samples
constexpr int SAMPLE_BUFFER_SIZE = 512;
Sample sampleBuffer[SAMPLE_BUFFER_SIZE];
int sampleBufferIndex = 0;

//STATE MACHINE DFINITION//
enum class Status {
    ACTIVE,
    // ARMED,
    DESCENT,
    TOUCHDOWN
};

Status status = Status::ACTIVE;


//GLOBAL VATIABLES AND THRESHOLDS//
// Timing
unsigned long armTimeMs = 0; // logging the time the armed button was pressed
unsigned long lastCommandMs = 0; // logging the exact time the command packet was last received
const unsigned long MIN_FREEFALL_TIME_MS = 0;    // example: 3 seconds after arming
const unsigned long MIN_DESCENT_TIME_MS = 150000; // 50000 ms
unsigned long descentStartMs = 0;
const unsigned long COMMAND_TIMEOUT_MS   = 22000;   // 22 seconds
static bool gnssEverValid = false; // variables for syncing the teensy internal clock and the GNSS
static uint32_t lastGnssUnix = 0; // variables for syncing the teensy internal clock and the GNSS
static uint32_t lastGnssMillis = 0; // variables for syncing the teensy internal clock and the GNSS
bool actuatorDeployed = false;
bool createNumberedDescentLog();
int findNextRunNumber();
// System-wide timestamp tracking
unsigned long systemStartMs = 0;  // When system powered on
unsigned long activeStartMs = 0;   // When entered ACTIVE state
unsigned long armedStartMs = 0;    // When ARM button pressed
unsigned long touchdownStartMs = 0; // When touchdown detected


// Freefall detection counters
int freefallAccelCount   = 0; 
int altitudeDropCount    = 0;

// Thresholds 
const float FREEFALL_ACCEL_THRESHOLD = 3.0f;   // m/s², magnitude below this = near freefall
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
    uint32_t eventMillis;
    uint8_t eventTime;   // seconds since ACTIVE start
    uint8_t  peak;
    uint8_t  rms;
    uint16_t duration;    // ms
};
static volatile uint8_t detEventCount = 0;
static DetonationEvent detEvents[4]; // store last 4 sound events in struct detEvents, when communication window communicate those

//HELPER FUNCTIONS

// FUNCTION TO GET SAMPLES N SECONDS AGO (used by lora.cpp science packet)
bool getSampleSecondsAgo(float secondsAgo, BmpSample& out) {
    uint32_t targetTime = millis() - (uint32_t)(secondsAgo * 1000.0f);

    for (int i = 0; i < BMP_HISTORY_SIZE; i++) {
        int idx = (bmpHistoryIndex - 1 - i + BMP_HISTORY_SIZE) % BMP_HISTORY_SIZE;

        if (bmpHistory[idx].timestampMs <= targetTime) {
            out = bmpHistory[idx];
            return true;
        }
    }
    return false;
}

static bool commandReceived = false;
static uint8_t lastCmdByte = 0;

static inline float vec3_mag(float x, float y, float z) {
    return sqrtf(x*x + y*y + z*z);
}

//TIMESTAMPING//
// Htime since system start
unsigned long getSystemTimeMs() {
    return millis() - systemStartMs;
}

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

//SD CARD DEBUG
// ============================================
// SD CARD DEBUG FUNCTIONS
// ============================================
void logEventSummary() {
    if (!sd::isDescentOpen()) {
        Serial.println("[EVENT LOG] Descent file not open");
        return;
    }
    
    Serial.println("[EVENT LOG] Writing event timeline to CSV");
    
    // Add blank lines to separate data from events
    sd::writeDescentLine("");
    sd::writeDescentLine("");
    sd::writeDescentLine("=== FLIGHT EVENT TIMELINE ===");
    
    char line[128];
    
    snprintf(line, sizeof(line), "System Start,0 ms");
    sd::writeDescentLine(line);
    
    if (activeStartMs > 0) {
        snprintf(line, sizeof(line), "Active State,%lu ms", activeStartMs - systemStartMs);
        sd::writeDescentLine(line);
    }
    
    if (armedStartMs > 0) {
        snprintf(line, sizeof(line), "Armed,%lu ms", armedStartMs - systemStartMs);
        sd::writeDescentLine(line);
    }
    
    if (descentStartMs > 0) {
        snprintf(line, sizeof(line), "Freefall Detected,%lu ms", descentStartMs - systemStartMs);
        sd::writeDescentLine(line);
        
        snprintf(line, sizeof(line), "Actuator Deployed,%lu ms", descentStartMs - systemStartMs);
        sd::writeDescentLine(line);
    }
    
    if (touchdownStartMs > 0) {
        snprintf(line, sizeof(line), "Touchdown,%lu ms", touchdownStartMs - systemStartMs);
        sd::writeDescentLine(line);
    }
    
    sd::flushDescentLog();
    
    Serial.println("[EVENT LOG] Event timeline written");
}


// Read and dump a specific file
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

// List ALL files on SD card (recursive)
void listAllFiles(File dir, int depth = 0) {
    while (true) {
        File entry = dir.openNextFile();
        if (!entry) break;
        
        // Print indentation
        for (int i = 0; i < depth; i++) Serial.print("  ");
        
        Serial.print(entry.name());
        
        if (entry.isDirectory()) {
            Serial.println("/");
            listAllFiles(entry, depth + 1);
        } else {
            Serial.print(" - ");
            Serial.print(entry.size());
            Serial.println(" bytes");
        }
        entry.close();
    }
}

void listAllFilesOnSD() {
    Serial.println("=== ALL FILES ON SD CARD ===");
    File root = SD.open("/");
    if (!root) {
        Serial.println("Failed to open root directory");
        return;
    }
    listAllFiles(root, 0);
    root.close();
    Serial.println("============================");
}

// List all run files in /logs directory
void listRunFiles() {
    Serial.println("=== RUN FILES ===");
    
    File dir = SD.open("/logs");
    if (!dir) {
        Serial.println("Failed to open /logs directory");
        return;
    }
    
    if (!dir.isDirectory()) {
        Serial.println("/logs is not a directory");
        dir.close();
        return;
    }
    
    int count = 0;
    while (true) {
        File entry = dir.openNextFile();
        if (!entry) break;
        
        // Only show files that start with "run_"
        String name = entry.name();
        if (name.startsWith("run_") && name.endsWith(".csv")) {
            Serial.print("  ");
            Serial.print(entry.name());
            Serial.print(" - ");
            Serial.print(entry.size());
            Serial.println(" bytes");
            count++;
        }
        entry.close();
    }
    
    Serial.print("Total run files: ");
    Serial.println(count);
    Serial.println("=================");
    
    dir.close();
}

// Get the current run filename
void getCurrentRunFilename(char* buffer, size_t bufferSize) {
    int currentRun = sd::findNextRunNumber() - 1;
    if (currentRun < 1) {
        buffer[0] = '\0';  // Empty string if no runs
        return;
    }
    snprintf(buffer, bufferSize, "/logs/run_%04d.csv", currentRun);
}

// Dump the current run file (the one created in this session)
void dumpCurrentRun() {
    char filename[64];
    getCurrentRunFilename(filename, sizeof(filename));
    
    if (filename[0] == '\0') {
        Serial.println("No current run file found");
        return;
    }
    
    Serial.print("Current run file: ");
    Serial.println(filename);
    dumpSdFile(filename);
}

// Complete debug - shows everything
void debugAfterRun() {
    Serial.println("\n\n### SD CARD DEBUG ###\n");
    
    // Show all files
    listAllFilesOnSD();
    
    Serial.println();
    
    // Show just run files
    listRunFiles();
    
    Serial.println();
    
    // Dump the current run
    dumpCurrentRun();
}

// ============================================
// END SD CARD DEBUG FUNCTIONS
// ============================================

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

    // Tunables
    const int SOUND_THRESHOLD = 583; // corresponds to 81dB SPL, 75dB SPL is 292
    const uint32_t COOLDOWN_MS = 50;

    int16_t tempBuffer[128];

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
                // write raw sample to already-open mic raw file
                sd::writeMicRaw(&sample, sizeof(sample));
                downsampleCounter = 0;
            }

            // 2) PEAK TRACKING
            if ((int16_t)absVal > eventMaxPeak) eventMaxPeak = (int16_t)absVal;

            // 3) LOUD EVENT DETECTION
            if (absVal > SOUND_THRESHOLD) {
                // Serial.println("----- EVENT DETECTED-----");
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

    // 4) LOG SUMMARY EVENT TO SD (after cooldown)
    uint32_t now = millis();
    if (isEventActive && (now - lastLoudSampleTime > COOLDOWN_MS)) {
        if (sampleCount == 0) {
            isEventActive = false;
            eventMaxPeak = 0;
            return;
        }

        DetonationEvent e;
        e.eventMillis = (uint32_t)(eventStartTime); //aboslute time reference in milliseconds
        e.eventTime = (uint8_t)((eventStartTime - activeStartMs) / 1000); // absolute time
        e.peak      = (uint8_t)(eventMaxPeak / 128);
        e.rms       = (uint8_t)(sqrt(sumSquares / (double)sampleCount) / 128);
        e.duration  = (uint16_t)(lastLoudSampleTime - eventStartTime);

        // Write event summary to already-open mic event file
        char line[64];
        snprintf(line, sizeof(line), "%u,%u,%u,%u", e.eventMillis, e.peak, e.rms, e.duration);
        sd::writeMicEventLine(line);

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
    if (sampleBufferIndex == 0) return;

    // Descent log must already be opened once in setup()
    if (!sd::isDescentOpen()) {
        Serial.println("Descent log not open!");
        return;
    }

    Serial.print("[FLASH] Writing ");
    Serial.print(sampleBufferIndex);
    Serial.println(" samples to SD...");

    // Write all buffered samples (or make this chunked if needed)
    for (int i = 0; i < sampleBufferIndex; i++) {
        const Sample& s = sampleBuffer[i];

        char line[128];
        snprintf(line, sizeof(line),
                 "%lu,%.2f,%.2f,%.2f,%.2f,%.2f",
                 s.timestampMs,
                 s.temperature,
                 s.pressure,
                 s.altitude,
                 s.pm2_5,
                 s.pm10_0);
        
        sd::writeDescentLine(line);  // Actually write the line!
    }

    // CRITICAL: Flush data to physical SD card
    sd::flushDescentLog();
    
    Serial.println("[FLASH] Flush complete");

    // Clear buffer after committing the batch
    sampleBufferIndex = 0;
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
// TinyGPSPlus gps;

//SETUP//
void setup() {
    Serial.begin(100000);
    //Serial1.begin(100000);
    delay(1000);
    Serial.println("hello");
    systemStartMs = millis();
    Teensy3Clock.set(1737460800); 
    bool ok = true;

    // Helper for consistent logging
    auto logStep = [](const char* name) {
        Serial.print("[SETUP] ");
        Serial.println(name);
    };

    auto logResult = [](const char* name, bool pass) {
        Serial.print("[SETUP] ");
        Serial.print(name);
        Serial.print(" -> ");
        Serial.println(pass ? "OK" : "FAIL");
    };

    logStep("interface::setup()");
    interface::setup();
    delay(100);
    logResult("interface::setup()", true);

    logStep("sd::setup()");
    ok &= (sd::setup() == 0);
    logResult("sd::setup()", ok);

    if (ok) {
        logStep("sd::openMicLogs()");
        bool micOk = sd::openMicLogs();
        ok &= micOk;
        logResult("sd::openMicLogs()", micOk);

        // Create numbered descent log file
        logStep("sd::createNumberedDescentLog()");
        bool descOk = sd::createNumberedDescentLog();
        ok &= descOk;
        logResult("sd::createNumberedDescentLog()", descOk);
    }

    logStep("lora::setup()");
    bool loraOk = (lora::setup() == 0);
    ok &= loraOk;
    delay(100);
    logResult("lora::setup()", loraOk);

    logStep("gnss::setup()");
    gnss::setup();
    delay(100);
    logResult("gnss::setup()", true);

    logStep("imu::setup()");
    imu::setup();
    delay(100);
    logResult("imu::setup()", true);

    logStep("bmp::setup()");
    bool bmpOk = (bmp::setup() == 0);
    ok &= bmpOk;
    delay(100);
    logResult("bmp::setup()", bmpOk);

    logStep("pm::setup()");
    pm::setup();
    delay(100);
    logResult("pm::setup()", true);

    logStep("actuator::setup()");
    actuator::setup();
    delay(100);
    logResult("actuator::setup()", true);

    logStep("actuator::undeploy()");
    // actuator::trigger();
    // delay(10000);
    actuator::undeploy();
    delay(100);
    logResult("actuator::undeploy()", true);

    logStep("mic::setup(16384)");
    mic::setup(16384);
    delay(100);
    logResult("mic::setup()", true);

    if (!ok) {
        Serial.println("[SETUP] ERROR: One or more subsystems failed.");
        Serial.println("[SETUP] Press RESET button to reboot, or wait indefinitely...");
        interface::startSystemBlinking();
        
        while (1) {
            interface::update();  // Update button states
            interface::serviceBlink();
            
            // Check for reset button
            if (interface::resetPressed()) {
                Serial.println("[SETUP] RESET button pressed - rebooting...");
                Serial.flush();
                delay(100);
                
                // Perform hardware reset
                SCB_AIRCR = 0x05FA0004;  // ARM Cortex-M7 reset command
            }
            
            delay(10);
        }
    }

    interface::stopSystemBlinking();
    Serial.println("[SETUP] setup finished OK");
}

//ACTIVE// 
//ACTIVE// 
void handleActive() {
    static bool armedLatched = false;   // prevents re-arming logic from running again
    unsigned long now = millis();

    static bool firstEntry = true;
    if (firstEntry) {
        activeStartMs = now;
        firstEntry = false;
        Serial.print("[ACTIVE] Entered at system time: ");
        Serial.print(getSystemTimeMs());
        Serial.println(" ms");
    }

    mic::discardBuffer();

    // If ARM button pressed -> go DESCENT and turn off yellow LED
    if (interface::armPressed()) {
        unsigned long now = millis();
        Serial.println("[ACTIVE] ARM button pressed -> DESCENT");
        status = Status::DESCENT;
        descentStartMs = millis();                 
        freefallAccelCount = 0;          
        altitudeDropCount = 0;
        hasLastAltitude = false;
        interface::setArmedLed(false);
    }
}

   
void handleDescent() {
    static uint32_t lastHzTick  = 0;
    static uint32_t lastPrint   = 0;
    static uint32_t lastFlushMs = 0;
    static const uint32_t FLUSH_PERIOD_MS = 20000; // 20s
    
    // Command mailbox
    static bool cmdPending = false;
    static uint8_t pendingCmdByte = 0;
    static uint32_t pendingCmdMs = 0;
    
    // PM sampling timer + last valid reading cache
    static uint32_t lastPmRead = 0;
    static pm::Reading lastPmReading;
    
    // DEPLOYMENT STATE
    static bool deploymentTriggered = false;
    static uint32_t freefallStartMs = 0;
    static const float FREEFALL_ACCEL_THRESHOLD = 3.0;
    static const uint32_t FREEFALL_DURATION_MS = 100;

    
    // ALTITUDE DROP RATE PROTECTION
    static float altitudeWindow[2] = {0};  // Store last 2 altitude readings (now and 1s ago)
    static uint32_t altitudeTimeWindow[2] = {0};  // Store timestamps
    static uint8_t altitudeIndex = 0;
    static const float MIN_ALTITUDE_DROP_RATE = 1.5;  // Must drop 1.5 meters in 1 second
    static const uint32_t ALTITUDE_WINDOW_TIME = 1000;  // Check drop over 1 second


    uint32_t now = millis();
    bool descentTimeOk = (descentStartMs != 0) && (now - descentStartMs >= MIN_DESCENT_TIME_MS);

    // HIGH-FREQUENCY POLLING (runs every loop)
    gnss::update();
    processSoundEvents();


    // FREEFALL DETECTION (only if not already deployed)
    static uint32_t lastAltitudeUpdate = 0;
    if (now - lastAltitudeUpdate >= 1000) {  // Every 1 second
        lastAltitudeUpdate = now;
        
        bmp::Reading b = bmp::read();
        if (b.valid) {
            altitudeWindow[altitudeIndex] = b.altitude;
            altitudeTimeWindow[altitudeIndex] = now;
            altitudeIndex = (altitudeIndex + 1) % 2;  // Toggle between 0 and 1
        }
    }

    
    // FREEFALL DETECTION (only if not already deployed)
if (!deploymentTriggered) {
    sensors_event_t accel, gyro, mag, temp;
    imu::read(accel, gyro, mag, temp);
    
    float ax = accel.acceleration.x;
    float ay = accel.acceleration.y;
    float az = accel.acceleration.z;
    float accelMag = sqrt(ax*ax + ay*ay + az*az);

    // Calculate altitude drop over 30-second window
    float oldestAltitude = 0;
    float newestAltitude = 0;
    uint32_t oldestTime = 0;
    uint32_t newestTime = 0;

    // Find oldest and newest valid readings within time window
    for (int i = 0; i < 2; i++) {
        if (altitudeTimeWindow[i] > 0 && now - altitudeTimeWindow[i] <= ALTITUDE_WINDOW_TIME) {
            if (altitudeTimeWindow[i] < oldestTime || oldestTime == 0) {
                oldestTime = altitudeTimeWindow[i];
                oldestAltitude = altitudeWindow[i];
            }
            if (altitudeTimeWindow[i] > newestTime) {
                newestTime = altitudeTimeWindow[i];
                newestAltitude = altitudeWindow[i];
            }
        }
    }
    
    float altitudeDrop = oldestAltitude - newestAltitude;
    uint32_t timeDelta = newestTime - oldestTime;
    bool altitudeDropOk = (altitudeDrop >= MIN_ALTITUDE_DROP_RATE) && (timeDelta >= 800);  // At least 800ms of data

    if (accelMag < FREEFALL_ACCEL_THRESHOLD) {
        if (freefallStartMs == 0) {
            freefallStartMs = now;
            Serial.println("[FREEFALL] Freefall detected, starting timer");
            Serial.print("[FREEFALL] Accel magnitude: ");
            Serial.print(accelMag);
            Serial.println(" m/s²");
        }
        
        // Check BOTH freefall duration AND altitude drop rate
        if (now - freefallStartMs >= FREEFALL_DURATION_MS) {
            if (altitudeDropOk) {
                deploymentTriggered = true;
                
                Serial.println("\n[DEPLOYMENT] ========================================");
                Serial.print("[DEPLOYMENT] Freefall duration: ");
                Serial.print(now - freefallStartMs);
                Serial.println(" ms");
                Serial.print("[DEPLOYMENT] Altitude drop: ");
                Serial.print(altitudeDrop);
                Serial.print(" m over ");
                Serial.print(timeDelta / 1000.0);
                Serial.println(" seconds");
                Serial.print("[DEPLOYMENT] Drop rate: ");
                Serial.print(altitudeDrop / (timeDelta / 1000.0));
                Serial.println(" m/s");
                Serial.print("[DEPLOYMENT] Final accel magnitude: ");
                Serial.print(accelMag);
                Serial.println(" m/s²");
                Serial.println("[DEPLOYMENT] TRIGGERING PARACHUTE DEPLOYMENT");
                Serial.println("[DEPLOYMENT] ========================================\n");

                actuator::trigger();
                
                Serial.println("ACTUATOR DEPLOYED");
                Serial.print("[ACTUATOR] Deployed at system time: ");
                Serial.print(getSystemTimeMs());
                Serial.println(" ms");
            } else {
                Serial.print("[DEPLOY BLOCKED] Altitude drop insufficient: ");
                Serial.print(altitudeDrop);
                Serial.print(" m over ");
                Serial.print(timeDelta / 1000.0);
                Serial.print(" s (need ");
                Serial.print(MIN_ALTITUDE_DROP_RATE);
                Serial.println(" m in 1s window)");
                freefallStartMs = 0; // Reset timer
            }
        }
    } else {
        // Not in freefall anymore - reset timer
        if (freefallStartMs != 0) {
            Serial.print("[FREEFALL] Ended after ");
            Serial.print(now - freefallStartMs);
            Serial.print(" ms (accel: ");
            Serial.print(accelMag);
            Serial.println(" m/s²)");
            freefallStartMs = 0;
        }
    }
}

        // ---------------- DEBUG: Deployment conditions ----------------
        static unsigned long lastCondPrint = 0;
        // if (now - lastCondPrint > 500) {   // 2 Hz, readable
        //     lastCondPrint = now;

        //     Serial.print("DEPLOY CHECK | ");

        //     Serial.print("FREEFALL_TIME=");
        //     Serial.print((freefallStartMs > 0) ? (now - freefallStartMs) : 0);
        //     Serial.print(" ms | ");

        //     Serial.print("DEPLOYED=");
        //     Serial.print(deploymentTriggered ? "YES" : "NO");

        //     Serial.println();
        // }
        // ---------------------------------------------------------
    }

    // FAST LoRa command polling (runs every loop)
    uint8_t cmd = 0;
    if (lora::receiveCommand(cmd)) {
        pendingCmdByte = cmd;
        pendingCmdMs = now;
        cmdPending = true;

        Serial.print("[CMD RX] cmdByte=0x");
        Serial.println(cmd, HEX);
    }

    // 1 Hz block
    if (now - lastHzTick >= 1000) {
        lastHzTick += 1000;

        bmp::Reading b = bmp::read();
        sensors_event_t accel, gyro, mag, temp;
        imu::read(accel, gyro, mag, temp);

        // PM SENSOR (2-second sampling)
        bool pmReady = false;
        pm::Reading newPmr;
        while (pm::poll(newPmr)) {            // will usually run 0 or 1 times
            lastPmReading = newPmr;           // keep newest
            pmReady = true;
            Serial.println("Received PM value");
        }
        /*
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
        }*/

        if (!b.valid) {
            Serial.println("[DESCENT] BMP read failed");
            return;
        }

        bmpHistory[bmpHistoryIndex] = { now, b.temperature, b.pressure, b.altitude };
        bmpHistoryIndex = (bmpHistoryIndex + 1) % BMP_HISTORY_SIZE;

        gnss::Location loc = getEnrichedLocation(b.altitude);

        Sample s;
        s.timestampMs = getSystemTimeMs();
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
        debugPrintGnss(loc);
        flash_storeSample(s);

        // Touchdown detection (only after minimum descent time)
        if (descentTimeOk && detectTouchdown(b.altitude)) {
            // Serial.println("[DESCENT] TOUCHDOWN detected");
            Serial.print("[TOUCHDOWN] Detected at system time: ");
            Serial.print(getSystemTimeMs());
            Serial.println(" ms");
            
            // touchdownStartMs = now;

            // Finalize microphone logging
            mic::stop();

            status = Status::TOUCHDOWN;
            return;
        }

        // Consume command (if one arrived since last second)
        if (cmdPending) {
            cmdPending = false;
            uint8_t cmdByte = pendingCmdByte;

            uint8_t team = cmdByte & 0x0F;
            if (team != (TEAM_ID & 0x0F)) {
                Serial.print("[CMD] Not for us, team=");
                Serial.println(team);
            } else {
                bool reqSci = (cmdByte & (1u << 4));
                bool reqTel = (cmdByte & (1u << 5));

                Serial.print("[CMD] our team. reqSci=");
                Serial.print(reqSci);
                Serial.print(" reqTel=");
                Serial.println(reqTel);

                // Spec: respond >= 50 ms after receiving command
                delay(60);

                // Send exactly what was requested
                if (reqTel) {
                    lora::sendTelemetry(loc, b.altitude);
                    delay(100);
                }
                if (reqSci) {
                    lora::sendScience(s);
                }

                lastCommandMs = now;
                flash_flushToSD();
            }
        } else {
            // COMMAND TIMEOUT - Debug what we would have sent
            if (now - lastCommandMs > COMMAND_TIMEOUT_MS) {
                Serial.print("[DESCENT] Command timeout (");
                Serial.print(COMMAND_TIMEOUT_MS);
                Serial.print(" ms) - flushing ");
                Serial.print(sampleBufferIndex);
                Serial.println(" samples to SD");
                
                // DEBUG: Show what packets would look like
                Serial.println("\n[TIMEOUT DEBUG] ========== PACKET DEBUG (no transmission) ==========");
                Serial.println("[TIMEOUT DEBUG] Current sample data:");
                Serial.print("  Timestamp: "); Serial.print(s.timestampMs); Serial.println(" ms");
                Serial.print("  Temperature: "); Serial.print(s.temperature); Serial.println(" °C");
                Serial.print("  Pressure: "); Serial.print(s.pressure); Serial.println(" Pa");
                Serial.print("  Altitude: "); Serial.print(s.altitude); Serial.println(" m");
                Serial.print("  PM2.5: "); Serial.println(s.pm2_5);
                Serial.print("  PM10.0: "); Serial.println(s.pm10_0);
                
                Serial.println("\n[TIMEOUT DEBUG] Science packet that would be sent:");
                // lora::debugSciencePacket(s);
                // lora::debugTelemetryPacket(loc, b.altitude);
                
                Serial.println("[TIMEOUT DEBUG] ================================================\n");
        
                
                Serial.println("\n[TIMEOUT DEBUG] Telemetry packet that would be sent:");
                Serial.print("  Lat: "); Serial.print(loc.latitude, 6);
                Serial.print(", Lon: "); Serial.print(loc.longitude, 6);
                Serial.print(", Alt: "); Serial.print(b.altitude);
                Serial.print(" m, Valid: "); Serial.println(loc.valid ? "YES" : "NO");
                Serial.println("[TIMEOUT DEBUG] ================================================\n");
                
                flash_flushToSD();
                lastCommandMs = now;
            }
        }
    }

    // 1 Hz debug print
    if (now - lastPrint >= 1000) {
        lastPrint = now;
        Serial.print("[DESCENT] tick | dt=");
        Serial.print(descentStartMs ? (now - descentStartMs) : 0);
        Serial.print(" ms | touchdown_enabled=");
        Serial.print(descentTimeOk ? "YES" : "NO");
        Serial.print(" | deployment=");
        Serial.print(deploymentTriggered ? "DONE" : "WAITING");
        if (freefallStartMs > 0 && !deploymentTriggered) {
            Serial.print(" | freefall_time=");
            Serial.print(now - freefallStartMs);
            Serial.print(" ms");
        }
        Serial.println();
    }

    mic::discardBuffer();
}

//TOUCHDOWN//
void handleTouchdown() {
    static unsigned long lastPrint = 0;
    static bool dataCollectionActive = true;
    static bool touchdownEntryLogged = false;

    unsigned long now = millis();

    if (touchdownStartMs == 0) touchdownStartMs = now;

    if (!touchdownEntryLogged) {
        touchdownEntryLogged = true;
        flash_flushToSD();
        logEventSummary();
        Serial.println("[TOUCHDOWN] Event timeline logged once on entry");
    }

    gnss::update();

    // Read sensors needed for LoRa response
    bmp::Reading b = bmp::read();
    sensors_event_t accel, gyro, mag, temp;
    imu::read(accel, gyro, mag, temp);

    // PM only while collecting (avoid timeouts spam)
    /*
    pm::Reading pmr;
    if (dataCollectionActive) {
        pm::update();
        pmr = pm::read();
    } else {
        pmr.valid = false;
        pmr.pm10_0 = -1;
        pmr.pm2_5  = -1;
    }*/
    pm::Reading pmr;
    bool pmReady = false;
    pm::Reading newPmr;
    while (pm::poll(newPmr)) {            // will usually run 0 or 1 times
        pmr = newPmr;           // keep newest
        pmReady = true;
        Serial.println("Received PM value");
    }

    gnss::Location loc = getEnrichedLocation(b.altitude);

    Sample s;
    s.timestampMs = getUnifiedTimestamp(loc.timestamp);
    s.temperature = b.temperature;
    s.pressure    = b.pressure;
    s.altitude    = b.altitude;

    if (pmr.valid) {
        s.pm10_0 = pmr.pm10_0;
        s.pm2_5  = pmr.pm2_5;
    } else {
        s.pm10_0 = -1;
        s.pm2_5  = -1;
    }

    if (dataCollectionActive) {
        // flash_storeSample(s);
    }

    uint8_t cmdByte = 0;
    bool command = lora::receiveCommand(cmdByte);

    if (command) {
        uint8_t team = cmdByte & 0x0F;
        if (team != (TEAM_ID & 0x0F)) {
            Serial.print("[CMD] Not for us, team="); Serial.println(team);
            return;
        }

        Serial.print("[CMD] cmdByte=0x"); Serial.println(cmdByte, HEX);

        bool reqSci = (cmdByte & (1u << 4));
        bool reqTel = (cmdByte & (1u << 5));

        delay(60);

        if (reqTel) lora::sendTelemetry(loc, b.altitude);
        if (reqSci) lora::sendScience(s);

        lastCommandMs = now;
        flash_flushToSD();

        // STOP data collection after command received
        dataCollectionActive = false;

        // OPTIONAL: stop PM hardware to prevent any further reads/timeouts
        pm::stop();

        Serial.println("[TOUCHDOWN] Data collection stopped after command");
    }

    if (now - lastPrint > 1000) {
        lastPrint = now;
        Serial.print("[TOUCHDOWN] tick");
        if (!dataCollectionActive) Serial.print(" (data collection STOPPED)");
        Serial.println();
    }
}


    void loop() {                                           
    for (int i = 0; i < 50; i++) {                     
        gnss::update();
    }                                                 

    actuator::update();
    interface::update();

    if (interface::resetPressed() && status == Status::ACTIVE) {  // Opening brace 3
        Serial.println("RESET BUTTON PRESSED - REBOOTING...");
        sd::closeDescentLog();
        sd::closeMicLogs();
        Serial.flush();
        delay(100);
        SCB_AIRCR = 0x05FA0004;
    }                                                    

    switch (status) {                                   
        case Status::ACTIVE:    handleActive();    break;
        case Status::DESCENT:   handleDescent();   break;
        case Status::TOUCHDOWN: handleTouchdown(); break;
    }                                                    
}                                                        


// void setup() {
//     Serial.begin(9600);
//     Serial.println("=== GNSS PASSTHROUGH TEST ===");

//     Serial1.begin(9600);   // try 38400 later if needed
// }

// void loop() {
//     // Serial.println("=== GNSS PASSTHROUGH TEST ===");
//     while (Serial1.available()) {
//         Serial.write(Serial1.read());
//     }
// }



// void loop() {
//   // GNSS -> PC + feed TinyGPS++
//   while (Serial1.available()) {
//     char c = (char)Serial1.read();
//     gps.encode(c);      // <-- required
//     Serial.write(c);    // raw passthrough
//   }

//   // PC -> GNSS (optional)
//   while (Serial.available()) {
//     Serial1.write(Serial.read());
//   }

//   // Print stats once per second
//   static uint32_t last = 0;
//   if (millis() - last >= 1000) {
//     last = millis();

//     Serial.print("\n[GPS] sat=");
//     Serial.print(gps.satellites.isValid() ? gps.satellites.value() : -1);

//     Serial.print(" hdop=");
//     Serial.print(gps.hdop.isValid() ? gps.hdop.hdop() : -1);

//     Serial.print(" locValid=");
//     Serial.print(gps.location.isValid());

//     Serial.print(" locAgeMs=");
//     Serial.print(gps.location.age());

//     Serial.print(" chars=");
//     Serial.print(gps.charsProcessed());

//     Serial.print(" cksumFail=");
//     Serial.print(gps.failedChecksum());

//     Serial.print(" sentencesFix=");
//     Serial.println(gps.sentencesWithFix());
//   }
// }

