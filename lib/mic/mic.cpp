#include "mic.h"
#include <Audio.h>

AudioInputI2S            i2sMic;      // I2S microphone input
AudioConnection*         patchCord = nullptr;

namespace mic {

    // Internal ring buffer state 
    static int16_t* circularBuffer = nullptr;
    static size_t ringSize = DEFAULT_BUFFER_SIZE;

    // Indices and count are volatile because they're modified in audio interrupt context
    static volatile size_t writeIndex = 0;
    static volatile size_t readIndex = 0;
    static volatile size_t sampleCount = 0; // number of unread samples in buffer

    // AudioStream subclass to receive audio blocks
    class MicStream : public AudioStream {
    public:
        MicStream() : AudioStream(1, inputQueueArray) {}
        virtual void update() override {
            audio_block_t* block = receiveReadOnly(0);
            if (!block) return;

            // AUDIO_BLOCK_SAMPLES is typically 128, iterate and push into ring buffer
            for (int i = 0; i < AUDIO_BLOCK_SAMPLES; ++i) {
                int16_t s = block->data[i]; // 16-bit sample

                // Write sample
                circularBuffer[writeIndex] = s;
                writeIndex++;
                if (writeIndex >= ringSize) writeIndex = 0;

                // Update sampleCount atomically-ish:
                if (sampleCount < ringSize) {
                    // there's space - just increment count
                    sampleCount++;
                } else {
                    // buffer full -> overwrite oldest: advance readIndex to drop 1 sample
                    readIndex++;
                    if (readIndex >= ringSize) readIndex = 0;
                    // sampleCount stays at ringSize
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

        // allocate ring buffer
        circularBuffer = new (std::nothrow) int16_t[ringSize];
        if (!circularBuffer) return 2;

        // initialize indices
        writeIndex = 0;
        readIndex = 0;
        sampleCount = 0;

        // allocate audio memory; tune depending on AUDIO_BLOCK_SAMPLES and usage
        AudioMemory(12);

        // connect I2S source to our MicStream so update() gets called
        patchCord = new AudioConnection(i2sMic, micStream);

        return 0;
    }

    size_t availableSamples() {
        // sampleCount is volatile and updated in ISR; reading is atomic for size_t on 32-bit,
        // but to be safe across architectures, protect with noInterrupts().
        noInterrupts();
        size_t count = sampleCount;
        interrupts();
        return count;
    }

    size_t readBuffer(int16_t* buffer, size_t len) {
        if (!buffer || len == 0) return 0;

        // Atomically snapshot available samples
        noInterrupts();
        size_t avail = sampleCount;
        // determine how many we will copy
        size_t toCopy = (len < avail) ? len : avail;

        // If nothing to copy, release interrupts and return
        if (toCopy == 0) {
            interrupts();
            return 0;
        }

        // Copy in up to two contiguous regions to handle wrap-around
        size_t firstChunk = ringSize - readIndex;
        if (firstChunk > toCopy) firstChunk = toCopy;

        // copy first chunk
        memcpy(buffer, &circularBuffer[readIndex], firstChunk * sizeof(int16_t));

        // copy second chunk if needed
        size_t secondChunk = toCopy - firstChunk;
        if (secondChunk > 0) {
            memcpy(buffer + firstChunk, &circularBuffer[0], secondChunk * sizeof(int16_t));
            readIndex = secondChunk;
        } else {
            readIndex += firstChunk;
            if (readIndex >= ringSize) readIndex = 0;
        }

        // update sampleCount
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
