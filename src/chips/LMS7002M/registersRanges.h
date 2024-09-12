#pragma once

#include <array>
#include <cstdint>

#include "AddressValuePair.h"

namespace lime::lms7002m {

struct RegisterRange {
    std::uint16_t first;
    std::uint16_t last;
    const char* name;
};

constexpr std::array<RegisterRange, 33> registersRanges{ {
    { 0x0006, 0x0006, "mSPI" },
    { 0x0020, 0x002F, "LimeLight" },
    { 0x0081, 0x0081, "EN_DIR" },
    { 0x0082, 0x0082, "AFE" },
    { 0x0084, 0x0084, "BIAS" },
    { 0x0085, 0x0085, "XBUF" },
    { 0x0086, 0x008C, "CGEN" },
    { 0x0092, 0x00A7, "LDO" },
    { 0x00A8, 0x00AC, "BIST" },
    { 0x00AD, 0x00AE, "CDS" },
    { 0x0100, 0x0104, "TRF" },
    { 0x0105, 0x010B, "TBB" },
    { 0x010C, 0x0114, "RFE" },
    { 0x0115, 0x011A, "RBB" },
    { 0x011C, 0x0124, "SX" },
    { 0x0125, 0x0126, "TRX_GAIN" },
    { 0x0200, 0x020C, "TxTSP" },
    { 0x0240, 0x0261, "TxNCO" },
    { 0x0280, 0x02A7, "TxGFIR1" },
    { 0x02C0, 0x02E7, "TxGFIR2" },
    { 0x0300, 0x0327, "TxGFIR3a" },
    { 0x0340, 0x0367, "TxGFIR3b" },
    { 0x0380, 0x03A7, "TxGFIR3c" },
    { 0x0400, 0x040F, "RxTSP" },
    { 0x0440, 0x0461, "RxNCO" },
    { 0x0480, 0x04A7, "RxGFIR1" },
    { 0x04C0, 0x04E7, "RxGFIR2" },
    { 0x0500, 0x0527, "RxGFIR3a" },
    { 0x0540, 0x0567, "RxGFIR3b" },
    { 0x0580, 0x05A7, "RxGFIR3c" },
    { 0x05C0, 0x05CC, "RSSI_DC_CALIBRATION" },
    { 0x0600, 0x0606, "RSSI_PDET_TEMP_CONFIG" },
    { 0x0640, 0x0641, "RSSI_DC_CONFIG" },
} };

} // namespace lime::lms7002m