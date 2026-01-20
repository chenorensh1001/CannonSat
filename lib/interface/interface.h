#pragma once
#include <Arduino.h>

namespace interface {

    // Pins (single source of truth)
    extern const int PIN_BTN_ARM;
    extern const int PIN_BTN_RESET;

    extern const int PIN_LED_SYSTEM;
    extern const int PIN_LED_ARMED;
    extern const int PIN_LED_ACTIVE;
    extern const int PIN_LED_OVERCURRENT;

    // Public API
    void setup();
    void update();

    // Buttons (edge events)
    bool armPressed();
    bool resetPressed();

    // LEDs
    void startSystemBlinking();
    void stopSystemBlinking();
    void setArmedLed(bool on);
    void debugGreenLedOn();
    void serviceBlink();
    void setActiveLedOn();

}
