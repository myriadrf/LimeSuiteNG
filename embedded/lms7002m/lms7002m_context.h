#ifndef LIME_LMS7002M_CONTEXT_H
#define LIME_LMS7002M_CONTEXT_H

#include "limesuiteng/embedded/lms7002m/lms7002m.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct lms7002m_context {
    lms7002m_hooks hooks;

    uint32_t reference_clock_hz; ///< Common reference clock for CGEN, SXR, SXT
} lms7002m_context;

#ifdef __cplusplus
} // extern C
#endif

#endif // LIME_LMS7002M_CONTEXT_H
