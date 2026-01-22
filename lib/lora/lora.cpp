#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <sample.h>
#include <gnss.h>
#include "lora_driver.h"
#include "config.h"
#include <stdint.h>

struct DetonationEvent {
    uint32_t eventMillis;
    uint8_t  eventTime;   // seconds since ACTIVE start (1 byte per packet spec)
    uint8_t  peak;
    uint8_t  rms;
    uint16_t duration;    // ms
};

// Forward declaration for function implemented in main.cpp
// This is the function that retrieves BMP samples N seconds ago.
bool getSampleSecondsAgo(float secondsAgo, BmpSample& out);
uint8_t getDetonationEvents(DetonationEvent out[4]);

namespace {
    uint8_t crc8_ccitt_07_msb(const uint8_t* data, size_t len) {
        uint8_t crc = 0x00;
        for (size_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (uint8_t j = 0; j < 8; j++) {
                if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0x07);
                else           crc = (uint8_t)(crc << 1);
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
        // SPI1.begin();
        SPI1.setSCK(27);
        SPI1.setMOSI(26);
        SPI1.setMISO(39);
        SPI1.begin();
        LoRa.setSPI(SPI1);
        LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT);  // CS, RST, DIO0

        if (!LoRa.begin(LORA_FREQ)) {
            Serial.println("Failed to start LoRa");
            return 1;
        }
        Serial.println("LoRa initialized");
        LoRa.setTxPower(LORA_TX_POWER, PA_OUTPUT_PA_BOOST_PIN);
        LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
        LoRa.setSignalBandwidth(LORA_BANDWIDTH);
        LoRa.setCodingRate4(LORA_CODING_RATE);
        // LoRa.endPacket(false); 

        // Serial.println("LoRa ready and listening...");
        // Serial.println("=== LoRa CONFIG (probe) ===");
        // Serial.print("Freq (Hz): "); Serial.println((uint32_t)LORA_FREQ);
        // Serial.print("SF: "); Serial.println(LORA_SPREADING_FACTOR);
        // Serial.print("BW: "); Serial.println(LORA_BANDWIDTH);
        // Serial.print("CR4/: "); Serial.println(LORA_CODING_RATE);
        // Serial.print("TX pwr: "); Serial.println(LORA_TX_POWER);

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
        // Force RX mode every time (debug / robust)
        // LoRa.receive();

        if (pendingPacketSize > 0) return true;

        int size = LoRa.parsePacket();
        if (size > 0) {
            Serial.print("[LORA] parsePacket size="); Serial.println(size);
            Serial.print("[LORA] RSSI="); Serial.print(LoRa.packetRssi());
            Serial.print(" SNR="); Serial.println(LoRa.packetSnr());
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
        int err = LoRa.endPacket(false);  // async
        if (err == 1) {
            LoRa.receive(); // back to RX
            return true;
        }

        LoRa.receive();
        return false;
    }

    bool receive(char* outBuf, size_t& outLen) {
    int packetSize = pendingPacketSize;
    if (packetSize == 0) packetSize = LoRa.parsePacket();
    if (packetSize <= 0) return false;

    size_t toCopy = (size_t)packetSize;
    if (toCopy > outLen) toCopy = outLen;

    // Read full packet; copy what fits, discard the rest
    size_t i = 0;
    for (; i < (size_t)packetSize; i++) {
        int b = LoRa.read();
        if (b < 0) break;
        if (i < toCopy) outBuf[i] = (char)b;
        // else: discard
    }

    outLen = (size_t)packetSize;      // report REAL packet length
    pendingPacketSize = 0;

    // back to RX
    LoRa.receive();
    return true;
}


    //receivecommand//
    bool receiveCommand(uint8_t& cmdByteOut) {
            // Serial.println("receiveCommand called");
            if (!packetAvailable()) return false;
            Serial.println("command received");

            char buf[255];
            size_t len = sizeof(buf);

            // This uses your internal lora::receive() (currently in lora.cpp)
            if (!receive(buf, len)) return false;

            // Debug print (very useful while testing)
            Serial.print("[RX] len="); Serial.print(len);
            Serial.print(" bytes: ");
            // LoRa.receive();
            for (size_t i = 0; i < len; i++) {
                uint8_t b = (uint8_t)buf[i];
                if (b < 0x10) Serial.print("0");
                Serial.print(b, HEX);
                Serial.print(" ");
            }
            Serial.println();

            // Command packet is exactly 2 bytes: 0x00, cmd
            if (len != 2) return false;
            if ((uint8_t)buf[0] != 0x00) return false;

            cmdByteOut = (uint8_t)buf[1];
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
        Serial.println("Building Telemetry Packet");
        packInt24(lastLat, &packet[1]);
        packInt24(lastLon, &packet[4]);
        packInt16(lastVel, &packet[7]);
        packUInt16((uint16_t)(bmpAltitude * 10.0), &packet[9]);
        uint32_t timestamp = millis() / 1000;
        packUInt32(timestamp, &packet[11]);
        packet[15] = crc8_ccitt_07_msb(packet, 15);

        
    }

    bool sendTelemetry(const gnss::Location& loc, float bmpAltitude) {
        Serial.println("[TEL] === sendTelemetry START ===");
        
        uint8_t packet[16];
        buildTelemetryPacket(packet, loc, bmpAltitude);
        
        // Print the actual packet bytes
        Serial.print("[TEL] Packet bytes: ");
        for (int i = 0; i < 16; i++) {
            if (packet[i] < 0x10) Serial.print("0");
            Serial.print(packet[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
        
        // Print what we're sending
        Serial.print("[TEL] Lat: "); Serial.print(loc.latitude, 6);
        Serial.print(" Lon: "); Serial.print(loc.longitude, 6);
        Serial.print(" Alt: "); Serial.print(bmpAltitude);
        Serial.print(" Valid: "); Serial.println(loc.valid);
        
        bool result = lora::send((const char*)packet, sizeof(packet));
        
        Serial.print("[TEL] Send result: ");
        Serial.println(result ? "SUCCESS" : "FAILED");
        Serial.println("[TEL] === sendTelemetry END ===");
        
        return result;
    }


 

// ---------------------------------------------------------------------
// Science Packet
// ---------------------------------------------------------------------

    void buildSciencePacket(uint8_t* packet, const Sample& s) {
        memset(packet, 0, 80);
        Serial.println("Building Science Packet");
        // Packet ID + Team ID (Team 4)
        packet[0] = 0x14;

        // Timestamp (Unix seconds)
        uint32_t unixTime = (uint32_t)(s.timestampMs / 1000);
        packUInt32(unixTime, &packet[1]);

        // Bitmap (update later if you want it dynamic)
        packet[5] = 0x0B;

        // ---------------------------
        // Detonation events (bytes 6–25)
        // ---------------------------
        DetonationEvent ev[4] = {};
        uint8_t nEv = getDetonationEvents(ev);
        uint32_t now = millis();

        for (uint8_t i = 0; i < nEv && i < 4; i++) {
            uint8_t base = 6 + i * 5;

            // USE e[0] (eventMillis) only for the calculation of relative time
            uint32_t diffMs = now - ev[i].eventMillis;

            // 2. Convert to seconds. 
            uint32_t diffSec = diffMs / 1000;
            
            // Packing into LORA packet
            packet[base + 0] = (uint8_t)constrain(diffSec, 0, 255); // relative time
            packet[base + 1] = ev[i].peak;                 // e[2]
            packet[base + 2] = ev[i].rms;                  // e[3]
            packUInt16(ev[i].duration, &packet[base + 3]); // e[4]
        }

        // ---------------------------
        // Secondary payloads (bytes 26–40): 3 samples × 5 bytes = 15 bytes
        // Each sample: PM2.5(2) + Pressure(2) + Temp(1)
        // PM10 REMOVED
        // Bytes 41-45 remain zero (from memset)
        // ---------------------------
        BmpSample s0{}, s4{}, s14{};
        bool ok0  = getSampleSecondsAgo(0,  s0);
        bool ok4  = getSampleSecondsAgo(4,  s4);
        bool ok14 = getSampleSecondsAgo(14, s14);

        // Fallbacks if history not ready
        if (!ok0) {
            s0.temperature = -999;
            s0.pressure    = -999;
            s0.altitude    = -999;
            s0.verticalVelocity = 0;
        }
        if (!ok4)  s4  = s0;
        if (!ok14) s14 = s0;

        BmpSample samples[3] = { s0, s4, s14 };

        auto encodeTemp = [](float tC) -> uint8_t {
            // 0.5°C resolution: enc = (tC + 25) * 2
            if (tC < -25.0f) tC = -25.0f;
            if (tC > 102.5f) tC = 102.5f;
            return (uint8_t)((tC + 25.0f) * 2.0f);
        };

        auto encodePressure = [](float pPa) -> int16_t {
            // enc = (p - 101325) / 50
            float x = (pPa - 101325.0f) / 50.0f;
            if (x < -32768.0f) x = -32768.0f;
            if (x >  32767.0f) x =  32767.0f;
            return (int16_t)x;
        };

        auto encodePM = [](float pm) -> uint16_t {
            // 0.1 ug/m3 resolution
            if (pm < 0) return 0;
            if (pm > 6553.5f) pm = 6553.5f;
            return (uint16_t)(pm * 10.0f);
        };

        // 3 samples × 5 bytes = 15 bytes (26-40)
        // Bytes 41-45 will be padding (zeros from memset)
        for (int i = 0; i < 3; i++) {
            uint8_t base = 26 + i * 5;

            // Use latest PM2.5 reading for all 3 timestamps
            uint16_t pm25 = encodePM(s.pm2_5);

            packUInt16(pm25, &packet[base + 0]);   // bytes 0-1: PM2.5

            int16_t pEnc = encodePressure(samples[i].pressure);
            packInt16(pEnc, &packet[base + 2]);    // bytes 2-3: Pressure

            packet[base + 4] = encodeTemp(samples[i].temperature); // byte 4: Temperature
        }

        // ---------------------------
        // Telemetry (bytes 71–78)
        // ---------------------------
        auto encodeAltitude = [](float altM) -> uint16_t {
            if (altM < 0) altM = 0;
            float x = altM * 10.0f;            // 0.1 m
            if (x > 65535.0f) x = 65535.0f;
            return (uint16_t)x;
        };

        packUInt16(encodeAltitude(s0.altitude),  &packet[71]);
        packUInt16(encodeAltitude(s4.altitude),  &packet[73]);
        packUInt16(encodeAltitude(s14.altitude), &packet[75]);

        int16_t velEnc = (int16_t)(s0.verticalVelocity * 100.0f);
        packInt16(velEnc, &packet[77]);

        // CRC
        packet[79] = crc8_ccitt_07_msb(packet, 79);
    }

    bool sendScience(const Sample& s) {
        uint8_t packet[80];
        buildSciencePacket(packet, s);
        return send((const char*)packet, 80);
        
    }

    void debugSciencePacket(const Sample& s) {
        uint8_t packet[80];
        buildSciencePacket(packet, s);

        Serial.println("\n[DEBUG] ===== SCIENCE PACKET (RAW 80 BYTES) =====");
        for (int i = 0; i < 80; i++) {
            if (i % 16 == 0) Serial.println();
            Serial.printf("%02X ", packet[i]);
        }
        Serial.println("\n[DEBUG] =========================================");

        // Header info
        Serial.println("[DEBUG] Header:");
        Serial.printf("  Packet ID: 0x%02X (Team=%d)\n", packet[0], packet[0] & 0x0F);
        uint32_t timestamp = ((uint32_t)packet[1] << 24) | ((uint32_t)packet[2] << 16) | 
                            ((uint32_t)packet[3] << 8) | packet[4];
        Serial.printf("  Timestamp: %u seconds\n", timestamp);
        Serial.printf("  Bitmap: 0x%02X (binary: 0b", packet[5]);
        for (int i = 7; i >= 0; i--) {
            Serial.print((packet[5] >> i) & 1);
        }
        Serial.println(")");

        // Detonation events
        Serial.println("[DEBUG] Detonation events decoded:");
        for (int i = 0; i < 4; i++) {
            uint8_t base = 6 + i * 5;
            int8_t   t   = (int8_t)packet[base + 0];  // Signed for negative relative time
            uint8_t  pk  = packet[base + 1];
            uint8_t  rms = packet[base + 2];
            uint16_t dur = (uint16_t)((packet[base + 3] << 8) | packet[base + 4]);

            Serial.printf("  Ev%d: time=%d s ago, peak=%u, rms=%u, dur=%u ms\n",
                        i + 1, -t, pk, rms, dur);
        }

        // Secondary payloads - RAW VALUES
        Serial.println("\n[DEBUG] ===== SECONDARY PAYLOAD - RAW VALUES =====");
        
        // Get the samples used in encoding
        BmpSample s0{}, s4{}, s14{};
        getSampleSecondsAgo(0,  s0);
        getSampleSecondsAgo(4,  s4);
        getSampleSecondsAgo(14, s14);
        
        BmpSample samples[3] = { s0, s4, s14 };
        const char* sampleTimes[3] = { "0s ago", "4s ago", "14s ago" };

        for (int i = 0; i < 3; i++) {
            Serial.printf("\n[DEBUG] Sample %d (%s) - RAW:\n", i + 1, sampleTimes[i]);
            Serial.printf("  PM2.5:       %.2f µg/m³\n", s.pm2_5);
            Serial.printf("  Pressure:    %.2f Pa\n", samples[i].pressure);
            Serial.printf("  Temperature: %.2f °C\n", samples[i].temperature);
            Serial.printf("  Altitude:    %.2f m\n", samples[i].altitude);
        }

        // Secondary payloads - ENCODED VALUES
        Serial.println("\n[DEBUG] ===== SECONDARY PAYLOAD - ENCODED VALUES =====");
        
        auto encodeTemp = [](float tC) -> uint8_t {
            if (tC < -25.0f) tC = -25.0f;
            if (tC > 102.5f) tC = 102.5f;
            return (uint8_t)((tC + 25.0f) * 2.0f);
        };

        auto encodePressure = [](float pPa) -> int16_t {
            float x = (pPa - 101325.0f) / 50.0f;
            if (x < -32768.0f) x = -32768.0f;
            if (x >  32767.0f) x =  32767.0f;
            return (int16_t)x;
        };

        auto encodePM = [](float pm) -> uint16_t {
            if (pm < 0) return 0;
            if (pm > 6553.5f) pm = 6553.5f;
            return (uint16_t)(pm * 10.0f);
        };

        for (int i = 0; i < 3; i++) {
            uint8_t base = 26 + i * 5;
            uint16_t pm25 = (uint16_t)((packet[base + 0] << 8) | packet[base + 1]);
            int16_t  pEnc = (int16_t)((packet[base + 2] << 8) | packet[base + 3]);
            uint8_t  tEnc = packet[base + 4];

            // Calculate what the encoded values SHOULD be
            uint16_t pm25_expected = encodePM(s.pm2_5);
            int16_t  pEnc_expected = encodePressure(samples[i].pressure);
            uint8_t  tEnc_expected = encodeTemp(samples[i].temperature);

            Serial.printf("\n[DEBUG] Sample %d (%s) - ENCODED:\n", i + 1, sampleTimes[i]);
            Serial.printf("  PM2.5 encoded:       %u (0x%04X) [expected: %u]\n", 
                        pm25, pm25, pm25_expected);
            Serial.printf("  Pressure encoded:    %d (0x%04X) [expected: %d]\n", 
                        pEnc, (uint16_t)pEnc, pEnc_expected);
            Serial.printf("  Temperature encoded: %u (0x%02X) [expected: %u]\n", 
                        tEnc, tEnc, tEnc_expected);
            
            // Decode back to verify
            float pm25_decoded = pm25 / 10.0f;
            float pEnc_decoded = (pEnc * 50.0f) + 101325.0f;
            float tEnc_decoded = (tEnc / 2.0f) - 25.0f;
            
            Serial.printf("  Decoded back:\n");
            Serial.printf("    PM2.5:       %.2f µg/m³\n", pm25_decoded);
            Serial.printf("    Pressure:    %.2f Pa\n", pEnc_decoded);
            Serial.printf("    Temperature: %.2f °C\n", tEnc_decoded);
            
            // Check for padding bytes
            Serial.printf("  Bytes in packet: [%02X %02X] [%02X %02X] [%02X]\n",
                        packet[base + 0], packet[base + 1],
                        packet[base + 2], packet[base + 3],
                        packet[base + 4]);
        }

        // Show padding section
        Serial.println("\n[DEBUG] Padding (bytes 41-45):");
        Serial.printf("  [%02X %02X %02X %02X %02X]\n",
                    packet[41], packet[42], packet[43], packet[44], packet[45]);

        // Telemetry data
        Serial.println("\n[DEBUG] Telemetry Data (bytes 71-78):");
        uint16_t alt0  = (uint16_t)((packet[71] << 8) | packet[72]);
        uint16_t alt4  = (uint16_t)((packet[73] << 8) | packet[74]);
        uint16_t alt14 = (uint16_t)((packet[75] << 8) | packet[76]);
        int16_t  vel   = (int16_t)((packet[77] << 8) | packet[78]);

        Serial.printf("  Altitude 0s:  %u (0x%04X) → %.1f m\n", alt0, alt0, alt0 / 10.0f);
        Serial.printf("  Altitude 4s:  %u (0x%04X) → %.1f m\n", alt4, alt4, alt4 / 10.0f);
        Serial.printf("  Altitude 14s: %u (0x%04X) → %.1f m\n", alt14, alt14, alt14 / 10.0f);
        Serial.printf("  Velocity:     %d (0x%04X) → %.2f m/s\n", vel, (uint16_t)vel, vel / 100.0f);

        // CRC
        uint8_t crcCalc = crc8_ccitt_07_msb(packet, 79);
        Serial.printf("\n[DEBUG] CRC: RX=0x%02X | CALC=0x%02X ", packet[79], crcCalc);
        if (packet[79] == crcCalc) {
            Serial.println("✓ OK");
        } else {
            Serial.println("✗ MISMATCH!");
        }
    }

    void debugTelemetryPacket(const gnss::Location& loc, float bmpAltitude) {
        uint8_t packet[16];
        buildTelemetryPacket(packet, loc, bmpAltitude);

        Serial.println("\n[DEBUG] ===== TELEMETRY PACKET (RAW 16 BYTES) =====");
        for (int i = 0; i < 16; i++) {
            if (packet[i] < 0x10) Serial.print("0");
            Serial.print(packet[i], HEX);
            Serial.print(" ");
        }
        Serial.println("\n[DEBUG] =========================================");

        // Decode header
        Serial.println("[DEBUG] Header:");
        Serial.printf("  Packet ID: 0x%02X (Team=%d)\n", packet[0], packet[0] & 0x0F);

        // Decode latitude (bytes 1-3)
        int32_t lat = ((int32_t)packet[1] << 16) | ((int32_t)packet[2] << 8) | packet[3];
        if (lat & 0x800000) lat |= 0xFF000000; // Sign extend
        Serial.printf("  Latitude (raw): %d → %.5f°\n", lat, lat / 100000.0);

        // Decode longitude (bytes 4-6)
        int32_t lon = ((int32_t)packet[4] << 16) | ((int32_t)packet[5] << 8) | packet[6];
        if (lon & 0x800000) lon |= 0xFF000000; // Sign extend
        Serial.printf("  Longitude (raw): %d → %.5f°\n", lon, lon / 100000.0);

        // Decode velocity (bytes 7-8)
        int16_t vel = (int16_t)((packet[7] << 8) | packet[8]);
        Serial.printf("  Velocity (raw): %d → %.2f m/s\n", vel, vel / 100.0);

        // Decode altitude (bytes 9-10)
        uint16_t alt = (uint16_t)((packet[9] << 8) | packet[10]);
        Serial.printf("  Altitude (raw): %u → %.1f m\n", alt, alt / 10.0);

        // Decode timestamp (bytes 11-14)
        uint32_t timestamp = ((uint32_t)packet[11] << 24) | ((uint32_t)packet[12] << 16) | 
                            ((uint32_t)packet[13] << 8) | packet[14];
        Serial.printf("  Timestamp: %u seconds\n", timestamp);

        // CRC
        uint8_t crcCalc = crc8_ccitt_07_msb(packet, 15);
        Serial.printf("[DEBUG] CRC: RX=0x%02X | CALC=0x%02X ", packet[15], crcCalc);
        if (packet[15] == crcCalc) {
            Serial.println("✓ OK");
        } else {
            Serial.println("✗ MISMATCH!");
        }

        // Input values
        Serial.println("\n[DEBUG] Input values used:");
        Serial.printf("  GPS Latitude: %.6f° (valid=%d)\n", loc.latitude, loc.valid);
        Serial.printf("  GPS Longitude: %.6f° (valid=%d)\n", loc.longitude, loc.valid);
        Serial.printf("  GPS Velocity: %.2f m/s\n", loc.verticalVelocity);
        Serial.printf("  GPS Timestamp: %u\n", (uint32_t)loc.timestamp);
        Serial.printf("  BMP Altitude: %.2f m\n", bmpAltitude);
        Serial.println("[DEBUG] =========================================\n");
    }

} //namspace lora