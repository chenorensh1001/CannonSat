#include <Arduino.h>
#include <RH_RF95.h>

#include <lora.h>
#include "config.h"

namespace lora {
    RH_RF95 rf95(RFM95_CS, RFM95_INT);

    volatile bool packetReceived = false;

    // IRQ interrupt service routine
    void onRadioInterrupt() {
        packetReceived = true;
    }

    int setup() {
        if(!rf95.init()) {
            Serial.println("Failed to find RFM95W");
            return 1; 
        }
        Serial.println("RFM95W found!");

        rf95.setFrequency(LORA_FREQ);
        rf95.setTxPower(LORA_TX_POWER);
        rf95.setSpreadingFactor(LORA_SPREADING_FACTOR);
        rf95.setCodingRate4(LORA_CODING_RATE);
        rf95.setSignalBandwidth(LORA_BANDWIDTH);

        pinMode(RFM95_INT, INPUT);
        attachInterrupt(digitalPinToInterrupt(RFM95_INT), onRadioInterrupt, RISING);

        return 0;
    }

    bool send(const String &msg) {
        uint8_t data[msg.length() + 1];
        msg.getBytes(data, sizeof(data));

        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
        return true;
    }

    bool receive(String &out) {
        if (!packetReceived) return false;

        packetReceived = false;

        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);

        if (rf95.recv(buf, &len)) {
            out = String((char*)buf);
            return true;
        }
        return false;
    }
}