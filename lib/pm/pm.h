#pragma once

namespace pm {

/**
 * @brief PM sensor reading structure
 * Stores particulate matter concentrations measured by D9 Laser Dust Sensor
 */
struct Reading {
    float pm1_0   = 0.0;    /* PM1.0 concentration in µg/m³ */
    float pm2_5   = 0.0;    /* PM2.5 concentration in µg/m³ */
    float pm10_0  = 0.0;    /* PM10 concentration in µg/m³ */
    bool valid    = false;  /* True if reading is valid */
};

/**
 * @brief Initialize D9 Dust Sensor UART interface
 * 
 * Sets up UART communication with the D9 sensor.
 * The D9 operates in automatic streaming mode, continuously sending
 * 32-byte packets at 9600 baud, once per second.
 * 
 * @return 0 on success, non-zero on failure
 */
int setup();

/**
 * @brief Read a single measurement from D9 sensor stream
 * 
 * Parses the next complete 32-byte packet from the UART buffer.
 * D9 streams data continuously, so this function drains the buffer
 * and returns the most recent valid packet.
 * 
 * Packet structure (32 bytes):
 * - Bytes 0-1:  Header (0x42, 0x4D)
 * - Bytes 2-3:  Frame length (0x00, 0x1C)
 * - Bytes 4-5:  PM1.0 (big-endian uint16)
 * - Bytes 6-7:  PM2.5 (big-endian uint16)
 * - Bytes 8-9:  PM10  (big-endian uint16)
 * - Bytes 10-29: Duplicates and reserved
 * - Bytes 30-31: Checksum (sum of bytes 0-29)
 * 
 * @return Reading struct with .valid = true if successful
 */
Reading read();

/**
 * @brief Cleanup D9 sensor (flush buffers)
 * 
 * Called during shutdown or when reinitializing the sensor.
 */
void cleanup();

/**
 * @brief Stop PM sensor operation (physically disable UART)
 * 
 * Stops the UART communication and prevents further updates.
 * Called when measurements should stop (e.g., after touchdown).
 * This physically stops the sensor communication, not just sets values to zero.
 */
void stop();

/**
 * @brief Update PM sensor continuously from buffer
 * 
 * Call this frequently in your main loop to drain the UART buffer
 * and keep the sensor stream in sync. Similar to gnss::update().
 * Does nothing if sensor has been stopped via stop().
 */
void update();

}
