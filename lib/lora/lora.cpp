#include <lora.h>
#include "config.h"

#include <Arduino.h>
#include <RH_RF69.h>

namespace lora {
    RH_RF69 rf69(RFM69_CS, RFM69_INT);

    int setup() {
        if(!rf69.init()) {
            Serial.println("Failed to find RFM95W");
            return 1; 
        }
        Serial.println("RFM95W found!");

        return 0;
    }
}