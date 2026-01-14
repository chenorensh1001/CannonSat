#include "interface.h"

namespace interface {

    // Pins
    const int PIN_BTN_ARM   = 28;
    const int PIN_BTN_RESET = 11;
    const int PIN_LED_SYSTEM = 12;   // always on when system has power
    const int PIN_LED_ARMED  = 30;

    // Debounce
    bool lastArm   = false;
    bool lastReset = false;
    bool armEvent   = false;
    bool resetEvent = false;

    void setup() {
        pinMode(PIN_BTN_ARM,   INPUT_PULLUP);
        pinMode(PIN_BTN_RESET, INPUT_PULLUP);

        pinMode(PIN_LED_SYSTEM, OUTPUT);
        pinMode(PIN_LED_ARMED,  OUTPUT);

        // System LED always on when powered
        digitalWrite(PIN_LED_SYSTEM, HIGH);
        digitalWrite(PIN_LED_ARMED,  LOW);
    }

    void update() {
        bool a = !digitalRead(PIN_BTN_ARM);
        bool r = !digitalRead(PIN_BTN_RESET);

        armEvent   = (a && !lastArm);
        resetEvent = (r && !lastReset);

        lastArm   = a;
        lastReset = r;
    }

    bool armPressed()   { return armEvent; }
    bool resetPressed() { return resetEvent; }

    void setSystemLed(bool on) {
        digitalWrite(PIN_LED_SYSTEM, on ? HIGH : LOW);
    }

    void setArmedLed(bool on) {
        digitalWrite(PIN_LED_ARMED, on ? HIGH : LOW);
    }
}
