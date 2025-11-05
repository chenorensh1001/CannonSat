#include <Arduino.h>
#include <SD.h>
#include <string.h>

#include "sd.h"
#include "config.h"

#ifndef SD_CS_PIN
#define SD_CS_PIN BUILTIN_SDCARD // Teensy 4.1 built-in SDIO interface
#endif

namespace sd {

    static bool initialized = false;
    static File logFile;

    // Create all parent directories for a path
    static bool ensureParentDirs(const char* path) {
        if (!path) return false;
        const char* slash = strrchr(path, '/');
        if (!slash || slash == path) return true; // no parent or root only

        size_t dirLen = static_cast<size_t>(slash - path);
        if (dirLen == 0) return true;

        // Simple fixed buffer to avoid heap use (allow deeper paths)
        char buf[512];
        if (dirLen >= sizeof(buf)) return false;

        memcpy(buf, path, dirLen);
        buf[dirLen] = '\0';

        // Walk and create nested dirs
        size_t i = (buf[0] == '/') ? 1 : 0;
        while (true) {
            // find next separator
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

            if (saved == '\0') break; // done
            buf[j] = saved;
            i = j + 1;
        }
        return true;
    }

    // Create all directories in the provided path (nested). Unlike ensureParentDirs,
    // this will attempt to create the full path.
    static bool ensureDirs(const char* dirPath) {
        if (!dirPath || !*dirPath) return false;

        // Copy dirPath into a local buffer we can mutate while walking
        char buf[512];
        size_t n = strnlen(dirPath, sizeof(buf));
        if (n == 0 || n >= sizeof(buf)) return false; // too long
        memcpy(buf, dirPath, n);
        buf[n] = '\0';

        // Remove trailing slashes (except when path is just "/")
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

            if (saved == '\0') break; // done
            buf[j] = saved;
            i = j + 1;
        }
        return true;
    }

    int setup() {
        if (initialized) return 0;
        if (!SD.begin(SD_CS_PIN)) {
            return 1;
        }
        initialized = true;
        return 0;
    }

    bool open(const char* path, bool append) {
        if (!path || !*path) return false;
        if (!initialized && setup() != 0) return false;

        if (!ensureParentDirs(path)) return false;

        if (!append && SD.exists(path)) {
            SD.remove(path); // emulate truncate
        }

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
        // Create full directory path (nested) without using heap String temporaries
        return ensureDirs(path);
    }

    bool exists(const char* path) {
        if (!initialized && setup() != 0) return false;
        return path && SD.exists(path);
    }
}