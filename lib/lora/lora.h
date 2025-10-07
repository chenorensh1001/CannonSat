#pragma once

namespace lora {
    int setup();
    bool send(const String &msg);
    bool receive(String &out);
}