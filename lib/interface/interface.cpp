#include "interface.h"

namespace interface {

    // Pins
    const int PIN_BTN_ARM   = 11; //11
    const int PIN_BTN_RESET = 36;
    const int PIN_LED_ARMED       = 20;
    const int PIN_LED_ARMED_GROUND       = 21;
    const int PIN_LED_ACTIVE         = 13; // green (active - 13) (HIGH side yellow - 20 on) (red active -- 15 ground -- 14)
    const int PIN_LED_ACTIVE_GROUND  = 12; // green LOW side (your "GND" pin)
    const int PIN_LED_OVERCURRENT = 14;
    const int PIN_LED_OVERCURRENT_GROUND = 15;

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
        pinMode(33, INPUT_PULLUP);  // ADD THIS - Overcurrent optocoupler with pull-up
        pinMode(PIN_LED_ARMED, OUTPUT);
        pinMode(PIN_LED_ARMED_GROUND, OUTPUT);
        pinMode(PIN_LED_ACTIVE, OUTPUT);
        pinMode(PIN_LED_ACTIVE_GROUND, OUTPUT);
        pinMode(PIN_LED_OVERCURRENT, OUTPUT);
        pinMode(PIN_LED_OVERCURRENT_GROUND, OUTPUT);
        digitalWrite(PIN_LED_ACTIVE_GROUND, HIGH);
        digitalWrite(PIN_LED_ARMED_GROUND, HIGH);
        digitalWrite(PIN_LED_OVERCURRENT_GROUND, LOW);
        digitalWrite(PIN_LED_ARMED, LOW);
        digitalWrite(PIN_LED_OVERCURRENT, LOW);
        digitalWrite(PIN_LED_ACTIVE, LOW);
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

        // Check overcurrent optocoupler (pin 35 with pull-up)
        bool pin33Value = digitalRead(33);
        
        // Debug: print every second
        // static unsigned long lastDebug = 0;
        // // if (millis() - lastDebug > 1000) {
        // //     Serial.print("Pin 33 value: ");
        // //     Serial.println(pin33Value ? "HIGH" : "LOW");
        // //     lastDebug = millis();
        // // }
        
        // LOW = power OK (optocoupler pulling to ground)
        // HIGH = no power (pulled up by resistor)
        if (pin33Value == HIGH) {  // no power
            digitalWrite(PIN_LED_OVERCURRENT_GROUND, LOW);
            digitalWrite(PIN_LED_OVERCURRENT, HIGH);  // turn RED LED ON
        } else {  // power OK
            digitalWrite(PIN_LED_OVERCURRENT_GROUND, LOW);
            digitalWrite(PIN_LED_OVERCURRENT, LOW);   // turn RED LED OFF
        }

        // Blink GREEN while booting/initializing
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
