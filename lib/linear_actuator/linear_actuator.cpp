#include <Arduino.h>
#include "linear_actuator.h"

namespace actuator {

    static bool actuatorDeployed = false;

    const int PIN_IN1 = 22;
    const int PIN_IN2 = 23;
    const unsigned long ACTUATOR_ON_TIME_MS = 1500;

    static bool active = false;
    static unsigned long startTime = 0;

    // ------------------------------------------------
    // Force actuator into SAFE / UNDEPLOYED state
    // ------------------------------------------------
    void undeploy() {
        digitalWrite(PIN_IN1, HIGH);
        digitalWrite(PIN_IN2, LOW);

        active = false;
        actuatorDeployed = false;
        startTime = 0;
    }

    void setup() {
        pinMode(PIN_IN1, OUTPUT);
        pinMode(PIN_IN2, OUTPUT);

        undeploy();  // <-- CRITICAL: force safe state on boot
    }

    // ------------------------------------------------
    // Deploy once
    // ------------------------------------------------
    void trigger() {
        if (actuatorDeployed) return;

        active = true;
        actuatorDeployed = true;
        startTime = millis();

        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, HIGH);
    }

    void update() {
        if (active && millis() - startTime > ACTUATOR_ON_TIME_MS) {
            digitalWrite(PIN_IN1, LOW);
            digitalWrite(PIN_IN2, LOW);
            active = false;
        }
    }

    bool isDeployed() {
        return actuatorDeployed;
    }

} // namespace actuator