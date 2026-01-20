#pragma once

namespace actuator {
    void setup();
    void trigger();
    void update();

    void undeploy();     // <-- ADD THIS
    bool isDeployed();
}