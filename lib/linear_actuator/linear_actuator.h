#pragma once
#include <Arduino.h>

namespace actuator {
    void setup();
    void trigger();
    void update();
    bool isActive();
}
