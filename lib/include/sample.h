#pragma once
#include <stdint.h>

struct BmpSample {
    uint32_t timestampMs;
    float temperature;
    float pressure;
    float altitude;
    float verticalVelocity;  // used in science packet
};

struct Sample {
    uint32_t timestampMs;
    float temperature;
    float pressure;
    float altitude;
    float verticalVelocity;  // used in science packet
    float pm1_0;     // PM1.0 concentration (µg/m³)
    float pm2_5;     // PM2.5 concentration (µg/m³)
    float pm10_0;    // PM10 concentration (µg/m³)

};

