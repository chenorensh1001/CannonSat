#pragma once
#include <Arduino.h>
#include <LoRa.h>
#include <sample.h>
#include <gnss.h>


namespace lora {

    // Maximum message length - Arduino LoRa lib doesn't define this, so define as needed
    constexpr size_t MAX_MSG_LEN = 255; // can be up to 255 bytes with Arduino LoRa lib

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
     * @return true if message was sent successfully
     */
    bool send(const char* msg, size_t len);

    /**
     * @brief Receive a message from LoRa
     * 
     * @param outBuf Pointer to a pre-allocated buffer where the received data will be stored
     * @param outLen On input: size of the buffer (in bytes); on output: actual length of the received message
     * @return true if a message was successfully received, false otherwise
     */
    //bool receive(char* outBuf, size_t& outLen);
    bool commandReceived();
    bool sendTelemetry(const gnss::Location& loc);
    bool sendScience(const Sample& s);


}
