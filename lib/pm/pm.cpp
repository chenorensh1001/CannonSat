#include <Arduino.h>
#include "pm.h"
#include "config.h"

namespace pm {

static HardwareSerial& pmSerial = Serial8;

// D9 / Plantower-like packet format
static constexpr uint8_t D9_HEADER_0     = 0x42;
static constexpr uint8_t D9_HEADER_1     = 0x4D;
static constexpr uint8_t D9_FRAME_LEN_0  = 0x00;
static constexpr uint8_t D9_FRAME_LEN_1  = 0x1C;
static constexpr int     D9_PACKET_SIZE  = 32;

static bool sensorStopped = false;

static uint16_t calculateChecksum(const uint8_t* data) {
    uint16_t sum = 0;
    for (int i = 0; i < 30; i++) sum += data[i];
    return sum;
}

static bool parsePacket(const uint8_t* packet, Reading& out) {
    if (packet[0] != D9_HEADER_0 || packet[1] != D9_HEADER_1) return false;
    if (packet[2] != D9_FRAME_LEN_0 || packet[3] != D9_FRAME_LEN_1) return false;

    const uint16_t calc = calculateChecksum(packet);
    const uint16_t got  = ((uint16_t)packet[30] << 8) | packet[31];
    if (calc != got) return false;

    const uint16_t pm1_0_raw = ((uint16_t)packet[4] << 8) | packet[5];
    const uint16_t pm2_5_raw = ((uint16_t)packet[6] << 8) | packet[7];
    const uint16_t pm10_raw  = ((uint16_t)packet[8] << 8) | packet[9];

    out.pm1_0  = (float)pm1_0_raw;
    out.pm2_5  = (float)pm2_5_raw;
    out.pm10_0 = (float)pm10_raw;
    out.valid  = true;
    out.timestampMs = millis();
    return true;
}

int setup() {
    sensorStopped = false;

    pmSerial.begin(PM_SENSOR_BAUD_RATE, SERIAL_8N1);

    // Drain stale bytes
    while (pmSerial.available()) pmSerial.read();

    Serial.println("PM: D9 sensor init on Serial8 (poll-based, non-blocking)");
    return 0;
}

// Non-blocking incremental parser.
// IMPORTANT: This function never waits for bytes.
// It will read and process ONLY bytes currently in the UART RX buffer.
bool poll(Reading& out) {
    if (sensorStopped) return false;

    static uint8_t frame[D9_PACKET_SIZE];
    static uint8_t idx = 0;

    enum State : uint8_t { WAIT_42, WAIT_4D, COLLECT };
    static State st = WAIT_42;

    out.valid = false;

    while (pmSerial.available()) {
        const uint8_t b = pmSerial.read();

        switch (st) {
            case WAIT_42:
                if (b == D9_HEADER_0) {
                    frame[0] = b;
                    idx = 1;
                    st = WAIT_4D;
                }
                break;

            case WAIT_4D:
                if (b == D9_HEADER_1) {
                    frame[1] = b;
                    idx = 2;
                    st = COLLECT;
                } else if (b == D9_HEADER_0) {
                    // re-sync on 0x42
                    frame[0] = b;
                    idx = 1;
                    st = WAIT_4D;
                } else {
                    st = WAIT_42;
                }
                break;

            case COLLECT:
                frame[idx++] = b;

                // early reject if length bytes wrong
                if (idx == 4) {
                    if (frame[2] != D9_FRAME_LEN_0 || frame[3] != D9_FRAME_LEN_1) {
                        idx = 0;
                        st = WAIT_42;
                        break;
                    }
                }

                if (idx == D9_PACKET_SIZE) {
                    idx = 0;
                    st = WAIT_42;

                    Reading tmp;
                    tmp.valid = false;
                    if (parsePacket(frame, tmp)) {
                        out = tmp;
                        return true;  // exactly one NEW parsed frame
                    }
                    // if invalid frame, keep scanning for the next header in the same call
                }
                break;
        }
    }

    return false; // no complete valid frame parsed this call
}

void stop() {
    pmSerial.end();
    sensorStopped = true;
    Serial.println("PM: Sensor stopped (UART disabled)");
}

void cleanup() {
    while (pmSerial.available()) pmSerial.read();
    if (!sensorStopped) {
        pmSerial.end();
        sensorStopped = true;
    }
    Serial.println("PM: Cleanup complete");
}

} // namespace pm