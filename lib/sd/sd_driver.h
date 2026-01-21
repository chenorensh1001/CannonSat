/**
 * @file sd_driver.h
 * @brief Simple SD logging utility built on Arduino SD.h for Teensy 4.1.
 *
 * @details
 * - Backend: Arduino SD.h.
 * - Chip select: Uses Teensy 4.1 BUILTIN_SDCARD (SDIO).
 * - Paths: nested directories are supported; internal path walker uses a 512-byte buffer
 *   (effective maximum path length is 511 characters).
 * - Threading: not ISR-safe and not re-entrant. Call these APIs from the main/application
 *   context only; do not use from interrupts.
 * - I/O semantics: open(path, append=false) truncates by removing the existing file, then
 *   recreating it; open(path, append=true) appends. Writes are buffered; call flush() or
 *   close() to ensure data is committed.
 *
 * Example:
 * @code{.cpp}
 * if (sd::setup() == 0 && sd::open("/logs/run.txt", true)) {
 *     sd::writeLine("hello, sd!");
 *     sd::flush();
 *     sd::close();
 * }
 * @endcode
 */

#pragma once
#include <Arduino.h>

namespace sd {
    /**
     * @brief Initialize the SD card interface.
     *
     * Uses Teensy 4.1 default BUILTIN_SDCARD (SDIO).
     *
     * @return int 0 on success, non-zero on failure (e.g., card not present or init error).
     */
    int setup();

    /**
     * @brief Open a file on the SD card for writing/logging.
     *
     * Creates parent directories if they do not exist. When append is false and the
     * file exists, it will be truncated (removed and recreated).
     *
     * @note Not ISR-safe. If truncation via remove() fails unexpectedly, the subsequent
     *       open() may still append depending on the filesystem state.
     *
     * @param path File path (e.g., "/logs/run.txt"). Can be absolute or relative.
     * @param append If true, appends to existing file; if false, truncates/overwrites.
     * @return true on success, false on failure.
     */
    bool open(const char* path, bool append = true);

    /**
     * @brief Write raw bytes to the currently open file.
     *
     * @note This does not automatically flush; call flush() or close() to ensure data is
     *       committed to the card.
     *
     * @param data Pointer to the data buffer to write.
     * @param len Number of bytes to write from the buffer.
     * @return size_t Number of bytes actually written (0 if no file is open or on error).
     */
    size_t write(const void* data, size_t len);

    /**
     * @brief Write a null-terminated C-string followed by a newline character.
     *
     * @note This does not automatically flush; call flush() or close() to ensure data is
     *       committed to the card.
     *
     * @param line C-string to write (must be non-null).
     * @return true if any characters were written, false otherwise.
     */
    bool writeLine(const char* line);

    /**
     * @brief Flush buffered data to the SD card for the open file.
     */
    void flush();

    /**
     * @brief Close the currently open file, flushing any pending data first.
     */
    void close();

    /**
     * @brief Check if a file is currently open.
     *
     * @return true if a file handle is open, false otherwise.
     */
    bool isOpen();

    /**
     * @brief Create a directory path, including all missing parent directories.
     *
     * @details Walks and creates every path segment without using dynamic String
     *          temporaries; safe for nested directories (e.g., "/logs/2025/11").
     *
     * @param path Directory path to create (e.g., "/logs/2025").
     * @return true on success (or if already exists), false on failure.
     */
    bool mkdirs(const char* path);

    /**
     * @brief Check whether a file or directory exists on the SD card.
     *
     * @param path File or directory path to check.
     * @return true if the path exists, false otherwise.
     */
    bool exists(const char* path);

    // -----------------------------
    // Microphone log helpers (open once, write often)
    // -----------------------------

    /**
     * @brief Open /logs/mic_2khz.bin and /logs/acoustic.txt once at startup (append mode).
     * @return true on success, false on failure.
     */
    bool openMicLogs();

    /**
     * @brief Write raw mic bytes to mic_2khz.bin (does NOT flush every call).
     * @param data Pointer to raw microphone data.
     * @param len Number of bytes to write.
     * @return size_t Number of bytes written.
     */
    size_t writeMicRaw(const void* data, size_t len);

    /**
     * @brief Write one CSV line to acoustic.txt (does NOT flush every call).
     * @param line C-string line to write.
     * @return true if written successfully, false otherwise.
     */
    bool writeMicEventLine(const char* line);

    /**
     * @brief Flush both microphone log files to SD card.
     */
    void flushMicLogs();

    /**
     * @brief Close both microphone log files.
     */
    void closeMicLogs();

    // -----------------------------
    // Descent log (open once, write many)
    // -----------------------------

    /**
     * @brief Open a descent log file with a header line (legacy method).
     * @param path File path for the descent log.
     * @param headerLine CSV header to write if file is new.
     * @return true on success, false on failure.
     */
    bool openDescentLog(const char* path, const char* headerLine);

    /**
     * @brief Find the next available run number for numbered log files.
     * @return int Next available run number (1-9998), or -1 on error.
     */
    int findNextRunNumber();

    /**
     * @brief Create a numbered descent log file (e.g., /logs/run_0001.csv).
     * 
     * Automatically finds the next available run number and creates the file
     * with a CSV header: timestamp,temperature,pressure,altitude,pm2_5,pm10_0
     * 
     * @return true on success, false on failure.
     */
    bool createNumberedDescentLog();

    /**
     * @brief Write one CSV line to the currently open descent log.
     * @param line C-string CSV line to write.
     * @return true if written successfully, false otherwise.
     */
    bool writeDescentLine(const char* line);

    /**
     * @brief Flush the descent log to SD card.
     */
    void flushDescentLog();

    /**
     * @brief Close the descent log file.
     */
    void closeDescentLog();

    /**
     * @brief Check if the descent log is currently open.
     * @return true if open, false otherwise.
     */
    bool isDescentOpen();
}