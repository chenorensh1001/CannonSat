#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

#include "lora_driver.h"
#include "config.h"

namespace lora {
    namespace {
        // Cache size of the most recent packet so parsePacket() is not called twice.
        volatile int pendingPacketSize = 0;
    }
    
    int setup() {
        LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT);  // CS, RST, DIO0

        if (!LoRa.begin(LORA_FREQ)) {
            Serial.println("Failed to start LoRa");
            return 1;
        }
        Serial.println("LoRa initialized");

        LoRa.setTxPower(LORA_TX_POWER);
        LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
        LoRa.setSignalBandwidth(LORA_BANDWIDTH);
        LoRa.setCodingRate4(LORA_CODING_RATE);

        Serial.println("LoRa ready and listening...");

        return 0;
    }

    void sleep() {
        LoRa.sleep();
        Serial.println("LoRa sleeping...");
    }

    void wake() {
        LoRa.receive();
        Serial.println("LoRa awake, listening...");
    }

    bool packetAvailable() {
        if (pendingPacketSize > 0) return true;

        // LoRa.parsePacket() returns packet size if available, 0 otherwise
        int size = LoRa.parsePacket();
        if (size > 0) {
            pendingPacketSize = size;
            return true;
        }
        return false;
    }

    bool send(const char* msg, size_t len) {
        if (len > MAX_MSG_LEN) return false;

        LoRa.beginPacket();
        LoRa.write((const uint8_t*)msg, len);
        int err = LoRa.endPacket(true);  // async
        if (err == 1) { // success
            LoRa.receive(); // switch to receive
            return true;
        }

        LoRa.receive();
        return false;
    }

    bool receive(char* outBuf, size_t& outLen) {
        int packetSize = pendingPacketSize;
        if (packetSize == 0) {
            packetSize = LoRa.parsePacket();
        }
        if (packetSize == 0) return false;

        if ((size_t)packetSize > outLen) {
            // If incoming packet is larger than buffer, truncate
            packetSize = outLen;
        }

        for (int i = 0; i < packetSize; i++) {
            int byteReceived = LoRa.read();
            if (byteReceived == -1) break;
            outBuf[i] = (char)byteReceived;
        }

        outLen = packetSize;
        pendingPacketSize = 0;
        return true;
    }
}
