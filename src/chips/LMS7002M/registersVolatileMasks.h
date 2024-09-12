#pragma once

#include <array>

#include "AddressValuePair.h"

namespace lime::lms7002m {

/// Addresses and masks of bits that can change as side effect of other register manipulation
constexpr std::array<AddressValuePair, 23> registersVolatileMasks{ {
    { 0x0001, 0xFFFF }, // MCU P1 port output
    { 0x0005, 0xFFFF }, // data from MCU
    { 0x008C, 0xF000 }, // CGEN VCO comparators
    { 0x00A8, 0x0100 }, // BIST BSTATE
    { 0x00A9, 0xFFFF }, // BIST BSIGT
    { 0x00AA, 0xFFFF }, // BIST BSIGR
    { 0x00AB, 0xFFFF }, // BIST BSIGR/BSIGC
    { 0x00AC, 0xFFFF }, // BIST BSIGC
    { 0x0123, 0xF000 }, // SX VCO comparators
    { 0x0209, 0xFFFF }, // TxTSP BIST
    { 0x020A, 0xFFFF }, // TxTSP BIST
    { 0x020B, 0xFFFF }, // TxTSP BIST
    { 0x040E, 0xFFFF }, // RxTSP adc, bist, rssi
    { 0x040F, 0xFFFF }, // RxTSP adc, bist, rssi
    { 0x05C3, 0x03FF }, // TX DAC shadow register
    { 0x05C4, 0x03FF }, // TX DAC shadow register
    { 0x05C5, 0x03FF }, // TX DAC shadow register
    { 0x05C6, 0x03FF }, // TX DAC shadow register
    { 0x05C7, 0x03FF }, // RX DAC shadow register
    { 0x05C8, 0x03FF }, // RX DAC shadow register
    { 0x05C9, 0x03FF }, // RX DAC shadow register
    { 0x05CA, 0x03FF }, // RX DAC shadow register
    { 0x0601, 0x003F }, // RSSI PDET comparators
} };

} // namespace lime::lms7002m
