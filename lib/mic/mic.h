#pragma once
#include <Arduino.h>
#include <Audio.h>

namespace mic {

    constexpr size_t DEFAULT_BUFFER_SIZE = 512; // samples per circular buffer

    /**
     * @brief Initialize the I²S microphone system
     *
     * @param sampleRate Desired sample rate in Hz (unused in this API, configure hardware separately if needed)
     * @param bufSize Number of samples for circular buffer (must be > 0)
     * @return 0 on success, non-zero on failure
     */
    int setup(size_t bufSize = DEFAULT_BUFFER_SIZE);

    /**
     * @brief How many unread samples are currently available in the ring buffer
     * @return number of samples available to read
     */
    size_t availableSamples();

    /**
     * @brief Read up to len samples from the ring buffer.
     *
     * The function copies up to min(len, availableSamples()) samples into the provided buffer,
     * advances the read index, and returns the number of samples copied.
     *
     * This function is non-blocking and safe to call from the main thread.
     *
     * @param[out] buffer pre-allocated buffer to receive samples (int16_t)
     * @param[in] len maximum samples to read
     * @return number of samples actually copied
     */
    size_t readBuffer(int16_t* buffer, size_t len);

    /**
     * @brief Clear all unread samples (advance read pointer to write pointer).
     *
     * Use when you decide you no longer need the buffered audio (e.g. reset).
     */
    void discardBuffer();

}
