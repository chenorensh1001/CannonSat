#include "linear_actuator.h"

namespace actuator {

    const int PIN_IN1 = 21;   // adjust to your wiring
    const int PIN_IN2 = 20;   // adjust to your wiring

    const unsigned long ACTUATOR_ON_TIME_MS = 1500;

    bool active = false;
    unsigned long startTime = 0;

    void setup() {
        pinMode(PIN_IN1, OUTPUT);
        pinMode(PIN_IN2, OUTPUT);

        // actuator off
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, LOW);
    }

    void trigger() {
        if (actuatorDeployed) return;   // already deployed → do nothing

        if (!active) {
            active = true;
            actuatorDeployed = true;    // mark as permanently deployed
            startTime = millis();

            digitalWrite(PIN_IN1, HIGH);
            digitalWrite(PIN_IN2, LOW);
        }
    }


    void update() {
        if (active && millis() - startTime > ACTUATOR_ON_TIME_MS) {
            // stop actuator
            digitalWrite(PIN_IN1, LOW);
            digitalWrite(PIN_IN2, LOW);
            active = false;
        }
    }

    bool isActive() {
        return active;
    }
}
