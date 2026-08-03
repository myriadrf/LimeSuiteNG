#ifndef LIME_LMS7002M_CONTEXT_H
#define LIME_LMS7002M_CONTEXT_H

#include "limesuiteng/embedded/lms7002m/lms7002m.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct lms7002m_context {
    lms7002m_hooks hooks;

    uint32_t reference_clock_hz; ///< Common reference clock for CGEN, SXR, SXT

    // calibration save/restore scratch, per chip so contexts don't clobber each other
    uint16_t chip_state_0x0020;
    uint16_t chip_state_data[359]; // UPDATE IF THE SAVED RANGES CHANGE
} lms7002m_context;

#ifdef __cplusplus
} // extern C
#endif

#endif // LIME_LMS7002M_CONTEXT_H
