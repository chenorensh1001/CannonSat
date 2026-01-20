#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <sample.h>
#include <gnss.h>
#include "lora_driver.h"
#include "config.h"
#include <stdint.h>

struct DetonationEvent {
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

    //receivecommand//
    bool receiveCommand(uint8_t& cmdByteOut) {
            // Serial.println("receiveCommand called");
            if (!packetAvailable()) return false;
            Serial.println("yes1");

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

    void buildSciencePacket(uint8_t* packet, const Sample& s) {
        memset(packet, 0, 80);
        Serial.println("startingtobuild");
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

        for (uint8_t i = 0; i < nEv && i < 4; i++) {
            uint8_t base = 6 + i * 5;
            packet[base + 0] = ev[i].eventTime;            // 1 byte
            packet[base + 1] = ev[i].peak;                 // 1 byte
            packet[base + 2] = ev[i].rms;                  // 1 byte
            packUInt16(ev[i].duration, &packet[base + 3]); // 2 bytes
        }

        // ---------------------------
        // Secondary payloads (bytes 26–46): 3 samples × 7 bytes
        // Each sample: PM2.5(2) + PM10(2) + Pressure(2) + Temp(1)
        // ---------------------------
        BmpSample s0{}, s7{}, s14{};
        bool ok0  = getSampleSecondsAgo(0,  s0);
        bool ok7  = getSampleSecondsAgo(7,  s7);
        bool ok14 = getSampleSecondsAgo(14, s14);

        // Fallbacks if history not ready
        if (!ok0) {
            s0.temperature = -999;
            s0.pressure    = -999;
            s0.altitude    = -999;
            s0.verticalVelocity = 0;
        }
        if (!ok7)  s7  = s0;
        if (!ok14) s14 = s0;

        BmpSample samples[3] = { s0, s7, s14 };

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

        for (int i = 0; i < 3; i++) {
            uint8_t base = 26 + i * 7;

            // Use latest PM reading for all 3 timestamps
            uint16_t pm25 = encodePM(s.pm2_5);
            uint16_t pm10 = encodePM(s.pm10_0);

            packUInt16(pm25, &packet[base + 0]);   // 26–27, 33–34, 40–41
            packUInt16(pm10, &packet[base + 2]);   // 28–29, 35–36, 42–43

            int16_t pEnc = encodePressure(samples[i].pressure);
            packInt16(pEnc, &packet[base + 4]);    // 30–31, 37–38, 44–45

            packet[base + 6] = encodeTemp(samples[i].temperature); // 32, 39, 46
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
        packUInt16(encodeAltitude(s7.altitude),  &packet[73]);
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

        Serial.println("[DEBUG] Detonation events decoded:");
        for (int i = 0; i < 4; i++) {
            uint8_t base = 6 + i * 5;
            uint8_t  t   = packet[base + 0];
            uint8_t  pk  = packet[base + 1];
            uint8_t  rms = packet[base + 2];
            uint16_t dur = (uint16_t)((packet[base + 3] << 8) | packet[base + 4]);

            Serial.printf("  Ev%d: time=%u s, peak=%u, rms=%u, dur=%u ms\n",
                        i + 1, t, pk, rms, dur);
        }

        Serial.println("[DEBUG] Secondary payloads decoded (PM2.5, PM10, PressureEnc, TempEnc):");
        for (int i = 0; i < 3; i++) {
            uint8_t base = 26 + i * 7;
            uint16_t pm25 = (uint16_t)((packet[base + 0] << 8) | packet[base + 1]);
            uint16_t pm10 = (uint16_t)((packet[base + 2] << 8) | packet[base + 3]);
            int16_t  pEnc = (int16_t)((packet[base + 4] << 8) | packet[base + 5]);
            uint8_t  tEnc = packet[base + 6];

            Serial.printf("  S%d: pm25=%u pm10=%u pEnc=%d tEnc=%u\n",
                        i + 1, pm25, pm10, pEnc, tEnc);
        }

        uint8_t crcCalc = crc8_ccitt_07_msb(packet, 79);
        Serial.printf("[DEBUG] CRC byte=0x%02X | CRC recalculated=0x%02X\n", packet[79], crcCalc);
    }
} //namspace lora