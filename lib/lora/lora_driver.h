#pragma once
#include <Arduino.h>
#include <LoRa.h>
#include <sample.h>
#include <gnss.h>

namespace lora {

    constexpr size_t MAX_MSG_LEN = 255;

    int setup();
    void sleep();
    void wake();
    bool packetAvailable();
    bool send(const char* msg, size_t len);
    bool commandReceived();

    // Build + send telemetry packet (GNSS + BMP altitude)
    bool sendTelemetry(const gnss::Location& loc, float bmpAltitude);

    // Build + send science packet
    bool sendScience(const Sample& s);

    // Debug: print raw telemetry values
    void debugTelemetry(const gnss::Location& loc, float bmpAltitude);

    // Debug: print packed bytes
    void debugPrintPacket(uint8_t* packet, size_t len);

    // Debug: print raw GNSS/BMP values (if separate)
    void debugPrintTelemetry(const gnss::Location& loc, float bmpAltitude);

    // Internal helper: build telemetry packet without sending
    void buildTelemetryPacket(uint8_t* packet, const gnss::Location& loc, float bmpAltitude);

}