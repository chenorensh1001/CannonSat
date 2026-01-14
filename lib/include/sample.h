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
};

