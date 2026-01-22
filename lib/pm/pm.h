#pragma once
#include <Arduino.h>

namespace pm {

struct Reading {
    float pm1_0   = 0.0f;
    float pm2_5   = 0.0f;
    float pm10_0  = 0.0f;
    bool  valid   = false;
    uint32_t timestampMs = 0;   // millis() when parsed
};

int  setup();

// Non-blocking: consumes only bytes already available in Serial8.
// Returns true exactly when a NEW valid frame has been parsed into `out`.
bool poll(Reading& out);

void stop();
void cleanup();

} // namespace pm