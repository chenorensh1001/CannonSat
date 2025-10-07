#pragma once
#include <Arduino.h>
#include <RH_RF95.h>

namespace lora {

    // Maximum message length
    constexpr size_t MAX_MSG_LEN = RH_RF95_MAX_MESSAGE_LEN;

    /**
     * @brief Initialize the LoRa module
     * @return 0 on success, 1 on failure
     */
    int setup();

    /**
     * @brief Put the LoRa module into sleep mode
     */
    void sleep();

    /**
     * @brief Wake the LoRa module from sleep mode
     */
    void wake();

    /**
     * @brief Check if a new packet has been received
     * @return true if a packet is available
     */
    bool packetAvailable();

    /**
     * @brief Send a message over LoRa
     * @param msg Pointer to the message buffer
     * @param len Length of the message in bytes
     * @param retries Number of send attempts (default 3)
     * @return true if message was sent successfully
     */
    bool send(const char* msg, size_t len, uint8_t retries = 3);

    /**
     * @brief Receive a message from LoRa
     * 
     * @param outBuf Pointer to a pre-allocated buffer where the received data will be stored
     * @param outLen On input: size of the buffer (in bytes); on output: actual length of the received message
     * @return true if a message was successfully received, false otherwise
     */
    bool receive(char* outBuf, size_t& outLen);

}
