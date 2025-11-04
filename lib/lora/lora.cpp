#include <Arduino.h>
#include <RH_RF95.h>

#include <lora.h>
#include "config.h"

namespace lora {
    RH_RF95 rf95(RFM95_CS, RFM95_INT);
    volatile bool packetReceivedFlag = false;

    void onRadioInterrupt() {
        packetReceivedFlag = true;
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
        attachInterrupt(digitalPinToInterrupt(RFM95_INT), onRadioInterrupt, FALLING);

        rf95.setModeRx();
        delay(2);
        Serial.println("RFM95W ready and listening...");

        return 0;
    }

    void sleep() {
        detachInterrupt(digitalPinToInterrupt(RFM95_INT));
        rf95.sleep();
        Serial.println("RFM95W sleeping...");
    }
    
    void wake() {
        rf95.setModeRx();
        attachInterrupt(digitalPinToInterrupt(RFM95_INT), onRadioInterrupt, FALLING);
        delay(2);
        Serial.println("RFM95W awake, listening...");
    }

    bool packetAvailable() {
        return packetReceivedFlag;
    }

    bool send(const char* msg, size_t len, uint8_t retries) {
        if (len > MAX_MSG_LEN) return false;

        uint8_t buf[MAX_MSG_LEN];
        memcpy(buf, msg, len);

        for (uint8_t attempt = 0; attempt < retries; ++attempt) {
            rf95.send(buf, len);
            if (rf95.waitPacketSent(2000)) { // 2s timeout
                rf95.setModeRx();
                return true;
            }
        }
        rf95.setModeRx();
        return false;
    }

    bool receive(char* outBuf, size_t& outLen) {
        noInterrupts();
        bool flag = packetReceivedFlag;
        packetReceivedFlag = false;
        interrupts();

        if (!flag) return false;

        uint8_t buf[MAX_MSG_LEN];
        uint8_t len = sizeof(buf);

        if (rf95.recv(buf, &len)) {
            if (len > outLen) len = outLen;
            memcpy(outBuf, buf, len);
            outLen = len; 
            return true;
        }
        return false;
    }
}