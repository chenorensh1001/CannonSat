#include "interface.h"

namespace interface {

    // Pins
    const int PIN_BTN_ARM   = 11; //11
    const int PIN_BTN_RESET = 28;
    const int PIN_LED_ARMED       = 20;
    const int PIN_LED_ARMED_GROUND       = 21;
    const int PIN_LED_ACTIVE         = 13; // green (active - 13) (HIGH side yellow - 20 on) (red active -- 15 ground -- 14)
    const int PIN_LED_ACTIVE_GROUND  = 12; // green LOW side (your "GND" pin)
    const int PIN_LED_OVERCURRENT = 15;
    const int PIN_LED_OVERCURRENT_GROUND = 14;

    // State
    static bool lastArm = false;
    static bool lastReset = false;
    static bool armEvent = false;
    static bool resetEvent = false;
    static bool systemBlinking = false;
    static unsigned long lastBlink = 0;
    static const unsigned long BLINK_INTERVAL = 250;
    static bool systemLedState = false;   // start OFF so blink is visible

    void setup() {
        pinMode(PIN_BTN_ARM, INPUT_PULLUP);
        pinMode(PIN_BTN_RESET, INPUT_PULLUP);
        pinMode(PIN_LED_ARMED, OUTPUT);
        pinMode(PIN_LED_ARMED_GROUND, OUTPUT);
        pinMode(PIN_LED_ACTIVE, OUTPUT);
        pinMode(PIN_LED_ACTIVE_GROUND, OUTPUT);   // <-- IMPORTANT
        pinMode(PIN_LED_OVERCURRENT, OUTPUT);
        digitalWrite(PIN_LED_ACTIVE_GROUND, LOW); // <-- IMPORTANT
        digitalWrite(PIN_LED_ARMED_GROUND, LOW); // ground reference
        digitalWrite(PIN_LED_ARMED, LOW);        // OFF for rail method (both sides LOW)
        digitalWrite(PIN_LED_OVERCURRENT, LOW);  // OFF (active-LOW assumption)
        digitalWrite(PIN_LED_ACTIVE, LOW);        // OFF if you are using HIGH/LOW pair as rails? (see note below)
        lastArm   = !digitalRead(PIN_BTN_ARM);
        lastReset = !digitalRead(PIN_BTN_RESET);
        armEvent   = false;
        resetEvent = false;
    }

    void update() {
        bool a = !digitalRead(PIN_BTN_ARM);
        bool r = !digitalRead(PIN_BTN_RESET);

        armEvent = a && !lastArm;
        resetEvent = r && !lastReset;

        lastArm = a;
        lastReset = r;

        // Ensure the "ground" reference stays LOW in normal operation
        digitalWrite(PIN_LED_ACTIVE_GROUND, LOW);

        // Blink GREEN while booting/initializing using the HIGH-side pin
        // ON = HIGH (since the other side is forced LOW)
        if (systemBlinking) {
            unsigned long now = millis();
            if (now - lastBlink >= BLINK_INTERVAL) {
                lastBlink = now;
                systemLedState = !systemLedState;
                digitalWrite(PIN_LED_ACTIVE, systemLedState ? HIGH : LOW);
            }
        }
    }

    bool armPressed() {
        bool e = armEvent;
        armEvent = false;
        return e;
    }

    bool resetPressed() {
        bool e = resetEvent;
        resetEvent = false;
        return e;
    }

    void startSystemBlinking() {
        systemBlinking = true;
        lastBlink = millis();
        systemLedState = false;

        digitalWrite(PIN_LED_ACTIVE_GROUND, LOW); // keep reference low
        digitalWrite(PIN_LED_ACTIVE, LOW);        // start OFF
    }

    void stopSystemBlinking() {
        systemBlinking = false;
        systemLedState = true;

        digitalWrite(PIN_LED_ACTIVE_GROUND, LOW); // keep reference low
        digitalWrite(PIN_LED_ACTIVE, HIGH);       // steady ON (high vs low rail)
    }

    void setArmedLed(bool on) {
        digitalWrite(PIN_LED_ARMED_GROUND, LOW);      // keep reference low
        digitalWrite(PIN_LED_ARMED, on ? HIGH : LOW); // rail method: HIGH=ON
    }

    // DEBUG: two-pin "rail" test: one forced LOW, one forced HIGH, then freeze
    void debugGreenLedOn() {
        pinMode(PIN_LED_ACTIVE, OUTPUT);
        pinMode(PIN_LED_ACTIVE_GROUND, OUTPUT);

        digitalWrite(PIN_LED_ACTIVE_GROUND, LOW); // "GND"
        digitalWrite(PIN_LED_ACTIVE, HIGH);       // "VCC"

        while (1) {
            // hold the rails forever
        }
    }

    void serviceBlink() {
        // Ensure the "ground" reference stays LOW
        digitalWrite(PIN_LED_ACTIVE_GROUND, LOW);

        if (!systemBlinking) return;

        unsigned long now = millis();
        if (now - lastBlink >= BLINK_INTERVAL) {
            lastBlink = now;
            systemLedState = !systemLedState;
            digitalWrite(PIN_LED_ACTIVE, systemLedState ? HIGH : LOW);
        }
    }

    void setActiveLedOn() {
    systemBlinking = false;
    digitalWrite(PIN_LED_ACTIVE_GROUND, LOW);
    digitalWrite(PIN_LED_ACTIVE, HIGH);  // ON for your two-pin rail
    }    
} // namespace interface
