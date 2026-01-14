#pragma once
#include <Arduino.h>

namespace interface {

    void setup();
    void update();

    bool armPressed();
    bool resetPressed();

    void setSystemLed(bool on);
    void setArmedLed(bool on);
}