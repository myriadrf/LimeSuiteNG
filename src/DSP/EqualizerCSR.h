#pragma once
#include "limesuiteng/Register.h"

namespace lime {
namespace EqualizerCSR {
struct CSRegister : public lime::Register {
    constexpr CSRegister()
        : lime::Register(0, 15, 0)
        , defaultValue(0)
        , twocomplement(false){};

    constexpr CSRegister(uint16_t address, uint8_t msb, uint8_t lsb, uint16_t defaultValue, bool twocomplement)
        : lime::Register(address, msb, lsb)
        , defaultValue(defaultValue)
        , twocomplement(twocomplement){};

    uint16_t defaultValue;
    bool twocomplement;
};

extern CSRegister EN_RXTSP;
extern CSRegister EN_TXTSP;

extern CSRegister RX_DCCORR_BYP;
extern CSRegister RX_PHCORR_BYP;
extern CSRegister RX_GCORR_BYP;

extern CSRegister RX_EQU_BYP;

extern CSRegister RX_DCLOOP_BYP;
extern CSRegister RX_DCLOOP_AVG;

extern CSRegister TX_HB_BYP;
extern CSRegister TX_HB_DEL;
extern CSRegister SLEEP_CFR;
extern CSRegister BYPASS_CFR;
extern CSRegister ODD_CFR;
extern CSRegister BYPASSGAIN_CFR;
extern CSRegister SLEEP_FIR;
extern CSRegister BYPASS_FIR;

extern CSRegister ODD_FIR;
extern CSRegister TX_PHCORR_BYP;
extern CSRegister TX_GCORR_BYP;
extern CSRegister TX_DCCORR_BYP;
extern CSRegister TX_ISINC_BYP;
extern CSRegister TX_EQU_BYP;
extern CSRegister TX_INVERTQ;

extern CSRegister TX_GCORRQ;
extern CSRegister TX_GCORRI;
extern CSRegister TX_PHCORR;
extern CSRegister TX_DCCORRI;
extern CSRegister TX_DCCORRQ;
extern CSRegister thresholdSpin;
extern CSRegister thresholdGain;
extern CSRegister CFR_ORDER; // dummy CSRegister

extern CSRegister RX_GCORRQ;
extern CSRegister RX_GCORRI;
extern CSRegister RX_PHCORR;
extern CSRegister cmbInsel;
extern CSRegister MAC;
} // namespace EqualizerCSR
} // namespace lime