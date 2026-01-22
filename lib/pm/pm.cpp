#include <Arduino.h>
#include "pm.h"
#include "config.h"

namespace pm {

// Static UART object for D9 sensor communication
static HardwareSerial& pmSerial = Serial8;

// D9 Dust Sensor protocol constants
static constexpr uint8_t D9_HEADER_0 = 0x42;
static constexpr uint8_t D9_HEADER_1 = 0x4D;
static constexpr uint8_t D9_FRAME_LEN_0 = 0x00;
static constexpr uint8_t D9_FRAME_LEN_1 = 0x1C;
static constexpr int D9_PACKET_SIZE = 32;
static constexpr uint32_t D9_SENSOR_TIMEOUT_MS = 100;  // 2 seconds to get a packet

// Circular buffer for D9 packets
static constexpr int D9_BUFFER_SIZE = 2;
static Reading pmBuffer[D9_BUFFER_SIZE];
static int bufferWriteIdx = 0;
static int bufferReadIdx = 0;

// Sensor state flag
static bool sensorStopped = false;

/**
 * @brief Calculate checksum for D9 packet
 * Checksum = sum of first 30 bytes (stored in bytes 30-31)
 */
static uint16_t calculateChecksum(const uint8_t* data) {
    uint16_t sum = 0;
    for (int i = 0; i < 30; i++) {
        sum += data[i];
    }
    return sum;
}

/**
 * @brief Read a complete 32-byte D9 packet from UART
 * Returns true if valid packet received, false on timeout or error
 */
static bool readPacket(uint8_t* buffer) {
    uint32_t startTime = millis();
    int bytesRead = 0;
    
    while (bytesRead < D9_PACKET_SIZE && (millis() - startTime) < D9_SENSOR_TIMEOUT_MS) {
        if (pmSerial.available()) {
            uint8_t byte = pmSerial.read();
            
            // Look for start bytes (0x42, 0x4D)
            if (bytesRead == 0) {
                if (byte == D9_HEADER_0) {
                    buffer[bytesRead++] = byte;
                }
            } else if (bytesRead == 1) {
                if (byte == D9_HEADER_1) {
                    buffer[bytesRead++] = byte;
                } else {
                    // Reset if second byte doesn't match
                    bytesRead = 0;
                    buffer[0] = 0;  // Clear buffer to prevent stale data
                    if (byte == D9_HEADER_0) {
                        buffer[bytesRead++] = byte;
                    }
                }
            } else {
                buffer[bytesRead++] = byte;
            }
        }
        yield();
    }
    
    return (bytesRead == D9_PACKET_SIZE);
}

/**
 * @brief Parse D9 packet and extract PM values
 */
static Reading parsePacket(const uint8_t* packet) {
    Reading r;
    
    // Validate packet structure
    if (packet[0] != D9_HEADER_0 || packet[1] != D9_HEADER_1) {
        Serial.println("PM: Invalid header");
        r.valid = false;
        return r;
    }
    
    if (packet[2] != D9_FRAME_LEN_0 || packet[3] != D9_FRAME_LEN_1) {
        Serial.println("PM: Invalid frame length");
        r.valid = false;
        return r;
    }
    
    // Verify checksum (bytes 30-31 contain checksum, calculated from bytes 0-29)
    uint16_t calculatedChecksum = calculateChecksum(packet);
    uint16_t packetChecksum = ((uint16_t)packet[30] << 8) | packet[31];
    
    if (calculatedChecksum != packetChecksum) {
        Serial.print("PM: Checksum mismatch. Calculated: ");
        Serial.print(calculatedChecksum, HEX);
        Serial.print(", Packet: ");
        Serial.println(packetChecksum, HEX);
        r.valid = false;
        return r;
    }
    
    // Extract PM values (big-endian 16-bit integers)
    // Bytes 4-5: PM1.0
    uint16_t pm1_0_raw = ((uint16_t)packet[4] << 8) | packet[5];
    // Bytes 6-7: PM2.5
    uint16_t pm2_5_raw = ((uint16_t)packet[6] << 8) | packet[7];
    // Bytes 8-9: PM10
    uint16_t pm10_raw = ((uint16_t)packet[8] << 8) | packet[9];
    
    // Values are already in µg/m³
    r.pm1_0 = (float)pm1_0_raw;
    r.pm2_5 = (float)pm2_5_raw;
    r.pm10_0 = (float)pm10_raw;
    r.valid = true;
    
    // // Debug output
    Serial.print("PM: PM1.0=");
    Serial.print(r.pm1_0);
    Serial.print(" PM2.5=");
    Serial.print(r.pm2_5);
    Serial.print(" PM10=");
    Serial.print(r.pm10_0);
    Serial.println(" µg/m³");
    
    return r;
}

int setup() {
    // Reset stopped flag
    sensorStopped = false;
    
    // Initialize UART for D9 sensor (9600 baud, 8N1)
    pmSerial.begin(PM_SENSOR_BAUD_RATE, SERIAL_8N1);
    
    // Clear any stale data
    while (pmSerial.available()) {
        pmSerial.read();
    }
    
    Serial.println("D9 Dust Sensor initialized on Serial3 (9600 baud)");
    Serial.println("D9 will stream continuously: 32-byte packets every 1 second");
    
    bufferWriteIdx = 0;
    bufferReadIdx = 0;
    
    return 0;
}

Reading read() {
    Reading r;
    
    // Try to read a valid packet
    uint8_t buffer[D9_PACKET_SIZE];
    if (!readPacket(buffer)) {
        Serial.println("PM: Timeout reading packet");
        r.valid = false;
        return r;
    }
    
    // Parse the packet
    r = parsePacket(buffer);
    
    return r;
}

void update() {
    // If sensor is stopped, do nothing
    if (sensorStopped) {
        return;
    }
    
    // Continuously drain the UART buffer to prevent overflow
    // This mimics the GNSS update() pattern for streaming sensors
    
    uint8_t buffer[D9_PACKET_SIZE];
    
    // Try to read available packets (non-blocking)
    while (pmSerial.available() >= D9_PACKET_SIZE) {
        // Look for packet headers in stream
        if (pmSerial.peek() == D9_HEADER_0) {
            if (readPacket(buffer)) {
                // Parse and store in circular buffer
                Reading r = parsePacket(buffer);
                pmBuffer[bufferWriteIdx] = r;
                bufferWriteIdx = (bufferWriteIdx + 1) % D9_BUFFER_SIZE;
            }
        } else {
            // Skip junk bytes
            pmSerial.read();
        }
    }
}

void stop() {
    // Physically stop the UART communication
    pmSerial.end();
    
    // Set flag to prevent further updates
    sensorStopped = true;
    
    Serial.println("PM: Sensor stopped (UART disabled)");
}

void cleanup() {
    // Flush any pending data
    while (pmSerial.available()) {
        pmSerial.read();
    }
    
    // Stop the sensor if not already stopped
    if (!sensorStopped) {
        pmSerial.end();
        sensorStopped = true;
    }
    
    Serial.println("PM: Cleanup complete");
}

}
