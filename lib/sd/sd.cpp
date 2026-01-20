#include <Arduino.h>
#include <SD.h>
#include <string.h>

#include "sd_driver.h"
#include "config.h"

#ifndef SD_CS_PIN
#define SD_CS_PIN BUILTIN_SDCARD
#endif

namespace sd {

    static bool initialized = false;

    // Generic single "logFile" (your existing API: open/writeLine/close)
    static File logFile;

    // Mic logs kept open
    static File micRawFile;
    static File micEventFile;

    // Descent log kept open
    static File descentFile;

    // ---------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------

    static bool ensureParentDirs(const char* path) {
        if (!path) return false;
        const char* slash = strrchr(path, '/');
        if (!slash || slash == path) return true;

        size_t dirLen = static_cast<size_t>(slash - path);
        if (dirLen == 0) return true;

        char buf[512];
        if (dirLen >= sizeof(buf)) return false;

        memcpy(buf, path, dirLen);
        buf[dirLen] = '\0';

        size_t i = (buf[0] == '/') ? 1 : 0;
        while (true) {
            size_t j = i;
            while (buf[j] != '\0' && buf[j] != '/') j++;

            char saved = buf[j];
            buf[j] = '\0';

            if (buf[0] != '\0' && !SD.exists(buf)) {
                if (!SD.mkdir(buf)) {
                    buf[j] = saved;
                    return false;
                }
            }

            if (saved == '\0') break;
            buf[j] = saved;
            i = j + 1;
        }
        return true;
    }

    static bool ensureDirs(const char* dirPath) {
        if (!dirPath || !*dirPath) return false;

        char buf[512];
        size_t n = strnlen(dirPath, sizeof(buf));
        if (n == 0 || n >= sizeof(buf)) return false;
        memcpy(buf, dirPath, n);
        buf[n] = '\0';

        while (n > 1 && buf[n - 1] == '/') {
            buf[n - 1] = '\0';
            --n;
        }

        size_t i = (buf[0] == '/') ? 1 : 0;
        while (true) {
            size_t j = i;
            while (buf[j] != '\0' && buf[j] != '/') j++;

            char saved = buf[j];
            buf[j] = '\0';

            if (buf[0] != '\0' && !SD.exists(buf)) {
                if (!SD.mkdir(buf)) {
                    buf[j] = saved;
                    return false;
                }
            }

            if (saved == '\0') break;
            buf[j] = saved;
            i = j + 1;
        }
        return true;
    }

    // ---------------------------------------------------------
    // Base SD API
    // ---------------------------------------------------------

    int setup() {
        if (initialized) return 0;
        if (!SD.begin(SD_CS_PIN)) return 1;
        initialized = true;
        return 0;
    }

    bool open(const char* path, bool append) {
        if (!path || !*path) return false;
        if (!initialized && setup() != 0) return false;

        if (!ensureParentDirs(path)) return false;

        if (!append && SD.exists(path)) SD.remove(path);

        if (logFile) logFile.close();
        logFile = SD.open(path, FILE_WRITE);
        return static_cast<bool>(logFile);
    }

    size_t write(const void* data, size_t len) {
        if (!logFile || !data || len == 0) return 0;
        return logFile.write(static_cast<const uint8_t*>(data), len);
    }

    bool writeLine(const char* line) {
        if (!logFile || !line) return false;
        return logFile.println(line) > 0;
    }

    void flush() {
        if (logFile) logFile.flush();
    }

    void close() {
        if (logFile) {
            logFile.flush();
            logFile.close();
        }
    }

    bool isOpen() {
        return static_cast<bool>(logFile);
    }

    bool mkdirs(const char* path) {
        if (!initialized && setup() != 0) return false;
        if (!path || !*path) return false;
        return ensureDirs(path);
    }

    bool exists(const char* path) {
        if (!initialized && setup() != 0) return false;
        return path && SD.exists(path);
    }

    // ---------------------------------------------------------
    // Mic logging (open once, write often)
    // ---------------------------------------------------------

    bool openMicLogs() {
        if (!initialized && setup() != 0) return false;
        if (!mkdirs("/logs")) return false;

        if (!micRawFile)   micRawFile   = SD.open("/logs/mic_2khz.bin", FILE_WRITE);
        if (!micEventFile) micEventFile = SD.open("/logs/acoustic.txt", FILE_WRITE);

        return (bool)micRawFile && (bool)micEventFile;
    }

    size_t writeMicRaw(const void* data, size_t len) {
        if (!data || len == 0) return 0;
        if (!micRawFile) {
            if (!openMicLogs()) return 0;
        }
        return micRawFile.write((const uint8_t*)data, len);
    }

    bool writeMicEventLine(const char* line) {
        if (!line) return false;
        if (!micEventFile) {
            if (!openMicLogs()) return false;
        }
        return micEventFile.println(line) > 0;
    }

    void flushMicLogs() {
        if (micRawFile)   micRawFile.flush();
        if (micEventFile) micEventFile.flush();
    }

    void closeMicLogs() {
        if (micRawFile)   { micRawFile.flush(); micRawFile.close(); }
        if (micEventFile) { micEventFile.flush(); micEventFile.close(); }
        micRawFile = File();
        micEventFile = File();
    }

    // ---------------------------------------------------------
    // Descent log kept open (matches YOUR header)
    // ---------------------------------------------------------

    bool openDescentLog(const char* path, const char* headerLine) {
        if (!path || !*path) return false;
        if (!initialized && setup() != 0) return false;

        if (!ensureParentDirs(path)) return false;

        bool existed = SD.exists(path);

        if (!descentFile) descentFile = SD.open(path, FILE_WRITE);
        if (!descentFile) return false;

        // Write header only once (only if file didn't exist before)
        if (!existed && headerLine && headerLine[0]) {
            descentFile.println(headerLine);
            descentFile.flush(); // one-time cost
        }

        return true;
    }

    bool writeDescentLine(const char* line) {
        if (!line) return false;
        if (!descentFile) return false;
        return descentFile.println(line) > 0;
    }

    void flushDescentLog() {
        if (descentFile) descentFile.flush();
    }

    void closeDescentLog() {
        if (descentFile) { descentFile.flush(); descentFile.close(); }
        descentFile = File();
    }

    bool isDescentOpen() {
        return (bool)descentFile;
    }

} // namespace sd