#include <Audio.h>
#include "mic.h"

AudioInputI2S2            i2sMic;      
AudioConnection* patchCord = nullptr;

namespace mic {

    // Internal ring buffer state 
    static int16_t* circularBuffer = nullptr;
    static size_t ringSize = DEFAULT_BUFFER_SIZE;

    // Indices and count
    static volatile size_t writeIndex = 0;
    static volatile size_t readIndex = 0;
    static volatile size_t sampleCount = 0; 

    // --- Filter State Variables ---
    static float prev_raw = 0.0f;
    static float prev_filtered = 0.0f;
    const float pole = 0.999f; // DC Blocker pole

    // A-Weighting Biquad States (Memory for the filter)
    // We use three stages (6 poles/zeros) to approximate the A-Weighting curve
    static float z1_1 = 0, z1_2 = 0;
    static float z2_1 = 0, z2_2 = 0;
    static float z3_1 = 0, z3_2 = 0;

    class MicStream : public AudioStream {
    public:
        MicStream() : AudioStream(1, inputQueueArray) {}
        
        virtual void update() override {
            audio_block_t* block = receiveReadOnly(0);
            if (!block) return;

            for (int i = 0; i < AUDIO_BLOCK_SAMPLES; ++i) {
                float x = (float)block->data[i];

                // --- 1. DC BLOCKER ---
                float highPassed = x - prev_raw + (pole * prev_filtered);
                prev_raw = x;
                prev_filtered = highPassed;

                // --- 2. A-WEIGHTING BIQUADS (at 44.1kHz) ---
                // Stage 1
                float out1 = highPassed * 0.1691f + z1_1;
                z1_1 = highPassed * -0.2116f - (-1.8669f) * out1 + z1_2;
                z1_2 = highPassed * 0.0425f - (0.8752f) * out1;

                // Stage 2
                float out2 = out1 * 1.0000f + z2_1;
                z2_1 = out1 * -2.0000f - (-1.8845f) * out2 + z2_2;
                z2_2 = out1 * 1.0000f - (0.8878f) * out2;

                // Stage 3
                float out3 = out2 * 1.0000f + z3_1;
                z3_1 = out2 * -2.0000f - (-1.9904f) * out3 + z3_2;
                z3_2 = out2 * 1.0000f - (0.9905f) * out3;

                // Final gain adjustment to normalize the filter
                float finalOutput = out3 * 1.15f; 

                // --- 3. STORE RESULT ---
                if (finalOutput > 32767.0f) finalOutput = 32767.0f;
                if (finalOutput < -32768.0f) finalOutput = -32768.0f;
                
                circularBuffer[writeIndex] = (int16_t)finalOutput;
                writeIndex++;
                if (writeIndex >= ringSize) writeIndex = 0;

                if (sampleCount < ringSize) {
                    sampleCount++;
                } else {
                    readIndex++;
                    if (readIndex >= ringSize) readIndex = 0;
                }
            }
            release(block);
        }
    private:
        audio_block_t* inputQueueArray[1];
    };

    static MicStream micStream;

    int setup(size_t bufSize) {
        if (bufSize == 0) return 1;
        ringSize = bufSize;

        circularBuffer = new (std::nothrow) int16_t[ringSize];
        if (!circularBuffer) return 2;

        writeIndex = 0; readIndex = 0; sampleCount = 0;
        prev_raw = 0.0f; prev_filtered = 0.0f;
        
        // Reset Filter States
        z1_1 = z1_2 = z2_1 = z2_2 = z3_1 = z3_2 = 0.0f;

        AudioMemory(12);
        patchCord = new AudioConnection(i2sMic, micStream);

        return 0;
    }

    // (availableSamples, readBuffer, and discardBuffer remain unchanged)
    size_t availableSamples() {
        noInterrupts();
        size_t count = sampleCount;
        interrupts();
        return count;
    }

    size_t readBuffer(int16_t* buffer, size_t len) {
        if (!buffer || len == 0) return 0;
        noInterrupts();
        size_t avail = sampleCount;
        size_t toCopy = (len < avail) ? len : avail;
        if (toCopy == 0) { interrupts(); return 0; }
        size_t firstChunk = ringSize - readIndex;
        if (firstChunk > toCopy) firstChunk = toCopy;
        memcpy(buffer, &circularBuffer[readIndex], firstChunk * sizeof(int16_t));
        size_t secondChunk = toCopy - firstChunk;
        if (secondChunk > 0) {
            memcpy(buffer + firstChunk, &circularBuffer[0], secondChunk * sizeof(int16_t));
            readIndex = secondChunk;
        } else {
            readIndex += firstChunk;
            if (readIndex >= ringSize) readIndex = 0;
        }
        sampleCount -= toCopy;
        interrupts();
        return toCopy;
    }

    void discardBuffer() {
        noInterrupts();
        readIndex = writeIndex;
        sampleCount = 0;
        interrupts();
    }
}