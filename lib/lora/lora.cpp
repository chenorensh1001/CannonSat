#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <sample.h>
#include <gnss.h>
#include "lora_driver.h"
#include "config.h"

// Forward declaration for function implemented in main.cpp
// This is the function that retrieves BMP samples N seconds ago.
bool getSampleSecondsAgo(float secondsAgo, BmpSample& out);


namespace {


    uint8_t crc8_ccitt(const uint8_t *data, size_t len) {
        uint8_t crc = 0x00;
        while (len--) {
            uint8_t extract = *data++;
            for (uint8_t i = 8; i; i--) {
                uint8_t sum = (crc ^ extract) & 0x01;
                crc >>= 1;
                if (sum) crc ^= 0x8C;
                extract >>= 1;
            }
        }
        return crc;
    }

    // Pack 24-bit signed integer (for lat/lon)
    void packInt24(int32_t value, uint8_t *buf) {
        buf[0] = (value >> 16) & 0xFF;
        buf[1] = (value >> 8) & 0xFF;
        buf[2] = value & 0xFF;
    }

    // Pack 16-bit unsigned
    void packUInt16(uint16_t value, uint8_t *buf) {
        buf[0] = (value >> 8) & 0xFF;
        buf[1] = value & 0xFF;
    }

    // Pack 16-bit signed
    void packInt16(int16_t value, uint8_t *buf) {
        buf[0] = (value >> 8) & 0xFF;
        buf[1] = value & 0xFF;
    }

    // Pack 32-bit unsigned
    void packUInt32(uint32_t value, uint8_t *buf) {
        buf[0] = (value >> 24) & 0xFF;
        buf[1] = (value >> 16) & 0xFF;
        buf[2] = (value >> 8) & 0xFF;
        buf[3] = value & 0xFF;
    }
}

namespace lora {

    namespace {

    static int32_t lastLat = 99999;
    static int32_t lastLon = 99999;
    static uint32_t lastTs = 0;
    static int16_t lastVel = 0;

        // Cache size of the most recent packet so parsePacket() is not called twice.
        volatile int pendingPacketSize = 0;
    }

    // ---------------------------------------------------------------------
    // Basic radio control
    // ---------------------------------------------------------------------
    //Last known valid GNSS values for the telemery packet
    int setup() {
        LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT);  // CS, RST, DIO0

        if (!LoRa.begin(LORA_FREQ)) {
            Serial.println("Failed to start LoRa");
            return 1;
        }
        Serial.println("LoRa initialized");

        LoRa.setTxPower(LORA_TX_POWER);
        LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
        LoRa.setSignalBandwidth(LORA_BANDWIDTH);
        LoRa.setCodingRate4(LORA_CODING_RATE);

        Serial.println("LoRa ready and listening...");

        LoRa.receive();
        return 0;
    }

    void sleep() {
        LoRa.sleep();
        Serial.println("LoRa sleeping...");
    }

    void wake() {
        LoRa.receive();
        Serial.println("LoRa awake, listening...");
    }

    bool packetAvailable() {
        if (pendingPacketSize > 0) return true;

        int size = LoRa.parsePacket();
        if (size > 0) {
            pendingPacketSize = size;
            return true;
        }
        return false;
    }

    // ---------------------------------------------------------------------
    // Raw send/receive helpers used by higher-level packet builders
    // ---------------------------------------------------------------------

    bool send(const char* msg, size_t len) {
        if (len > MAX_MSG_LEN) return false;

        LoRa.beginPacket();
        LoRa.write((const uint8_t*)msg, len);
        int err = LoRa.endPacket(true);  // async
        if (err == 1) {
            LoRa.receive(); // back to RX
            return true;
        }

        LoRa.receive();
        return false;
    }

    bool receive(char* outBuf, size_t& outLen) {
        int packetSize = pendingPacketSize;
        if (packetSize == 0) {
            packetSize = LoRa.parsePacket();
        }
        if (packetSize == 0) return false;

        if ((size_t)packetSize > outLen) {
            packetSize = outLen; 
        }

        for (int i = 0; i < packetSize; i++) {
            int byteReceived = LoRa.read();
            if (byteReceived == -1) break;
            outBuf[i] = (char)byteReceived;
        }

        outLen = packetSize;
        pendingPacketSize = 0;
        return true;
    }

    // ---------------------------------------------------------------------
    // Telemetry Packet
    // ---------------------------------------------------------------------

    void buildTelemetryPacket(uint8_t* packet, const gnss::Location& loc, float bmpAltitude) {
        packet[0] = 0xA4;

        if (loc.valid) {
            lastLat = (int32_t)(loc.latitude * 100000.0);
            lastLon = (int32_t)(loc.longitude * 100000.0);
            lastTs  = (uint32_t)loc.timestamp;
            lastVel = (int16_t)(loc.verticalVelocity * 100.0);
        }

        packInt24(lastLat, &packet[1]);
        packInt24(lastLon, &packet[4]);
        packInt16(lastVel, &packet[7]);
        packUInt16((uint16_t)(bmpAltitude * 10.0), &packet[9]);
        packUInt32(lastTs, &packet[11]);
    }

    bool sendTelemetry(const gnss::Location& loc, float bmpAltitude) {
        uint8_t packet[16];

        buildTelemetryPacket(packet, loc, bmpAltitude);

        // debugPrintTelemetry(loc, bmpAltitude);
        // debugPrintPacket(packet, sizeof(packet));

        return lora::send((const char*)packet, sizeof(packet));
    }


    //DEBUG THE TELEMETRY PACKET
    // void debugPrintTelemetry(const gnss::Location& loc, float bmpAltitude) {
    //     Serial.println("=== TELEMETRY DEBUG ===");
    //     Serial.print("GNSS valid now: ");
    //     Serial.println(loc.valid ? "YES" : "NO");

    //     Serial.print("Current GNSS lat/lon: ");
    //     Serial.print(loc.latitude, 6);
    //     Serial.print(", ");
    //     Serial.println(loc.longitude, 6);

    //     Serial.print("Current GNSS Alt: ");
    //     Serial.println(loc.altitude);

    //     Serial.print("BMP Altitude: ");
    //     Serial.println(bmpAltitude);

    //     Serial.print("Vertical Velocity (current): ");
    //     Serial.println(loc.verticalVelocity);

    //     Serial.println("--- VALUES SENT IN PACKET ---");
    //     Serial.print("Sent Latitude: ");
    //     Serial.println(lastLat / 100000.0, 6);

    //     Serial.print("Sent Longitude: ");
    //     Serial.println(lastLon / 100000.0, 6);

    //     Serial.print("Sent Vertical Velocity: ");
    //     Serial.println(lastVel / 100.0);

    //     Serial.print("Sent Timestamp: ");
    //     Serial.println(lastTs);

    //     Serial.println("=============================");
    // }

    // void debugPrintPacket(uint8_t* packet, size_t len) {
    //     Serial.print("Packet bytes: ");
    //     for (size_t i = 0; i < len; i++) {
    //         if (packet[i] < 16) Serial.print("0");
    //         Serial.print(packet[i], HEX);
    //         Serial.print(" ");
    //     }
    //     Serial.println();
    // }

    // void debugTelemetry(const gnss::Location& loc, float bmpAltitude) {
    //     uint8_t packet[16];
    //     buildTelemetryPacket(packet, loc, bmpAltitude);

    //     debugPrintTelemetry(loc, bmpAltitude);
    //     debugPrintPacket(packet, sizeof(packet));
    // }



    // ---------------------------------------------------------------------
    // Science Packet
    // ---------------------------------------------------------------------

    bool sendScience(const Sample& s) {
    uint8_t packet[80];
    memset(packet, 0, 80);

    // Packet ID + Team ID (Team 4)
    packet[0] = 0x14;

    // Timestamp (Unix seconds)
    uint32_t unixTime = s.timestampMs / 1000;
    packUInt32(unixTime, &packet[1]);

    // Bitmap
    packet[5] = 0x0B;

    // Retrieve 3 BMP samples: 0s, 7s, 14s ago
    BmpSample s0, s7, s14;
    getSampleSecondsAgo(0,  s0);
    getSampleSecondsAgo(7,  s7);
    getSampleSecondsAgo(14, s14);

    // Secondary payload block (bytes 26–40)
    uint8_t* sec = &packet[26];

    auto encodeTemp = [](float tC) -> uint8_t {
        return (uint8_t)((tC + 25.0f) * 2.0f);   // 0.5°C resolution, compatible with -25 until 25 celsius, 
    };

    auto encodePressure = [](float pPa) -> int16_t {
        return (int16_t)((pPa - 101325.0f) / 50.0f);  // 50 Pa resolution
    };

    auto encodePM10 = [](float pm) -> uint16_t {
    if (pm < 0) return 0;
    if (pm > 6553.5f) pm = 6553.5f;
    return (uint16_t)(pm * 10.0f);   // 0.1 µg/m³ resolution
    };

    auto encodePM25 = [](float pm) -> uint16_t {
        if (pm < 0) return 0;
        if (pm > 6553.5f) pm = 6553.5f;
        return (uint16_t)(pm * 10.0f);
    };

    // PM samples (reuse current reading for all 3)
    float pm10_samples[3] = { s.pm10_0, s.pm10_0, s.pm10_0 };
    float pm25_samples[3] = { s.pm2_5,  s.pm2_5,  s.pm2_5  };


    BmpSample samples[3] = { s0, s7, s14 };

    for (int i = 0; i < 3; i++) {
        int offset = i * 5;   // 5 bytes per sample

        // PM2.5 (2 bytes)
        uint16_t pm25 = encodePM25(pm25_samples[i]);
        packUInt16(pm25, &sec[offset + 0]);

        // PM10 (2 bytes)
        uint16_t pm10 = encodePM10(pm10_samples[i]);
        packUInt16(pm10, &sec[offset + 2]);

        // Temperature (1 byte)
        sec[offset + 4] = encodeTemp(samples[i].temperature);

    }


    // ------------------------------------------------------
    // Telemetry block (bytes 71–78)
    // Altitudes (0s, 7s, 14s) + vertical velocity
    // ------------------------------------------------------
    uint8_t* tel = &packet[71];

    auto encodeAltitude = [](float altM) -> uint16_t {
        return (uint16_t)(altM * 10.0f);  // 0.1 m resolution
    };

    uint16_t a0  = encodeAltitude(s0.altitude);
    uint16_t a7  = encodeAltitude(s7.altitude);
    uint16_t a14 = encodeAltitude(s14.altitude);

    packUInt16(a0,  &tel[0]);
    packUInt16(a7,  &tel[2]);
    packUInt16(a14, &tel[4]);

    int16_t vel = (int16_t)(s0.verticalVelocity * 100.0f);
    packInt16(vel, &tel[6]);

    // CRC
    packet[79] = crc8_ccitt(packet, 79);



    // ------------------------------------------------------
    // DEBUG PRINT — EXACT PACKET BEFORE TRANSMISSION
    // ------------------------------------------------------

    // ------------------------------------------------------
    // DEBUG: Print actual values and encoded values
    // ------------------------------------------------------
    Serial.println("[DEBUG] Science payload values:");

    for (int i = 0; i < 3; i++) {
        Serial.printf("  Sample %d:\n", i);

        Serial.printf("    Temp raw: %.2f C\n", samples[i].temperature);
        Serial.printf("    Temp enc: %u\n", encodeTemp(samples[i].temperature));

        int16_t pEnc = encodePressure(samples[i].pressure);
        Serial.printf("    Pressure raw: %.2f Pa\n", samples[i].pressure);
        Serial.printf("    Pressure enc: %d\n", pEnc);

        Serial.printf("    Alt raw: %.2f m\n", samples[i].altitude);
        Serial.printf("    Alt enc (telemetry): %u\n", encodeAltitude(samples[i].altitude));
    }
    Serial.println();


    Serial.println("[DEBUG] Science packet (80 bytes):");
    for (int i = 0; i < 80; i++) {
        if (i % 16 == 0) Serial.println();
        Serial.printf("%02X ", packet[i]);
    }
    Serial.println();
    // ------------------------------------------------------

    return send((const char*)packet, 80);
}

} // namespace lora