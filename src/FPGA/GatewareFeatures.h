#pragma once

namespace lime {

struct GatewareFeatures {
    uint32_t databusWidth{ 16 };
    bool hasConfigurableStreamPacketSize{ 0 };
};

} // namespace lime