#include "EqualizerCSR.h"

namespace lime {
namespace EqualizerCSR {
CSRegister EN_RXTSP = { 0x00A0, 0, 0, 0, 0 };
CSRegister EN_TXTSP = { 0x0080, 0, 0, 0, 0 };

CSRegister RX_DCCORR_BYP = { 0x00AC, 2, 2, 0, 0 };
CSRegister RX_PHCORR_BYP = { 0x00AC, 0, 0, 0, 0 };
CSRegister RX_GCORR_BYP = { 0x00AC, 1, 1, 0, 0 };

CSRegister RX_EQU_BYP = { 0x00AC, 5, 5, 0, 0 };

CSRegister RX_DCLOOP_BYP = { 0x00AC, 8, 8, 0, 0 };
CSRegister RX_DCLOOP_AVG = { 0x00A4, 2, 0, 0, 0 };

CSRegister TX_HB_BYP = { 0x0088, 0, 0, 1, 0 };
CSRegister TX_HB_DEL = { 0x0088, 1, 1, 0, 0 };
CSRegister SLEEP_CFR = { 0x0088, 2, 2, 1, 0 };
CSRegister BYPASS_CFR = { 0x0088, 3, 3, 1, 0 };
CSRegister ODD_CFR = { 0x0088, 4, 4, 1, 0 };
CSRegister BYPASSGAIN_CFR = { 0x0088, 5, 5, 1, 0 };
CSRegister SLEEP_FIR = { 0x0088, 6, 6, 1, 0 };
CSRegister BYPASS_FIR = { 0x0088, 7, 7, 1, 0 };

CSRegister ODD_FIR = { 0x0088, 8, 8, 0, 0 };
CSRegister TX_PHCORR_BYP = { 0x0088, 9, 9, 0, 0 };
CSRegister TX_GCORR_BYP = { 0x0088, 10, 10, 0, 0 };
CSRegister TX_DCCORR_BYP = { 0x0088, 11, 11, 0, 0 };
CSRegister TX_ISINC_BYP = { 0x0088, 12, 12, 1, 0 };
CSRegister TX_EQU_BYP = { 0x0088, 13, 13, 1, 0 };
CSRegister TX_INVERTQ = { 0x0088, 15, 15, 0, 0 };

CSRegister TX_GCORRQ = { 0x0081, 11, 0, 2047, 0 };
CSRegister TX_GCORRI = { 0x0082, 11, 0, 2047, 0 };
CSRegister TX_PHCORR = { 0x0083, 11, 0, 0, 1 };
CSRegister TX_DCCORRI = { 0x0084, 15, 0, 0, 0 };
CSRegister TX_DCCORRQ = { 0x0085, 15, 0, 0, 0 };
CSRegister thresholdSpin = { 0x0086, 15, 0, 0, 0 };
CSRegister thresholdGain = { 0x0087, 15, 0, 0, 0 };
CSRegister CFR_ORDER = { 0x008C, 7, 0, 0, 0 }; // dummy CSRegister

CSRegister RX_GCORRQ = { 0x00A1, 11, 0, 2047, 0 };
CSRegister RX_GCORRI = { 0x00A2, 11, 0, 2047, 0 };
CSRegister RX_PHCORR = { 0x00A3, 11, 0, 0, 1 };

CSRegister cmbInsel = { 0x0080, 2, 2, 0, 0 };

CSRegister MAC = { 0xFFFF, 1, 0, 0, 0 };
} // namespace EqualizerCSR
} // namespace lime
