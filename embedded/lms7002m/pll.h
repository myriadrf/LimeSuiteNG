#ifndef LIME_LMS7002M_PLL_H
#define LIME_LMS7002M_PLL_H

#include <stdint.h>

#include "limesuiteng/embedded/result.h"

#ifdef __cplusplus
extern "C" {
#endif

struct lms7002m_context;

typedef enum lms7002m_VCO { VCOL = 0, VCOM = 1, VCOH = 2 } lms7002m_VCO;

typedef struct pll_coefficients {
    uint32_t fractional;
    uint16_t integer;
    uint8_t sel_vco;
    uint8_t csw_vco;
    uint8_t div_loch;
    uint8_t en_div2;
} pll_coefficients;

lime_Result lms7002m_tune_vco_sx_fast(struct lms7002m_context* self, uint16_t* reg0121_value);
lime_Result lms7002m_calculate_pll_coefficients(struct pll_coefficients* pll, uint64_t refClk_Hz, uint64_t LO_freq_hz);
lime_Result lms7002m_set_lo_sx(struct lms7002m_context* self, uint64_t LO_freq_hz);

#ifdef __cplusplus
}
#endif

#endif // LIME_LMS7002M_PLL_H
