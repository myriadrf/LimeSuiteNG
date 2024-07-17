#ifndef LMS7002M_CSR_H
#define LMS7002M_CSR_H

#include "limesuiteng/embedded/types.h"

#ifdef __cplusplus
extern "C" {
#endif

struct lms7002m_csr {
    uint16_t address;
    uint8_t msb;
    uint8_t lsb;
};

#ifdef __cplusplus
}
#endif

#endif
