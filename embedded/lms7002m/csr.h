#ifndef LMS7002M_CSR_H
#define LMS7002M_CSR_H

#include "limesuiteng/embedded/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct lms7002m_csr {
    uint16_t address;
    uint8_t msb;
    uint8_t lsb;
} lms7002m_csr;

static inline uint32_t csr_mask(const struct lms7002m_csr csr)
{
    return (~(~0u << (csr.msb - csr.lsb + 1))) << (csr.lsb);
}

static inline uint32_t csr_clear_bits(uint32_t reg_value, const struct lms7002m_csr csr)
{
    return reg_value & (~csr_mask(csr));
}

static inline uint32_t csr_set_bits(uint32_t reg_value, const struct lms7002m_csr csr, uint32_t csr_value)
{
    const uint32_t mask = csr_mask(csr);
    uint32_t value = reg_value & (~mask);
    value |= (csr_value << csr.lsb) & mask;
    return value;
}

static inline uint32_t csr_get_bits(uint32_t reg_value, const struct lms7002m_csr csr)
{
    const uint32_t mask = csr_mask(csr);
    return (reg_value & mask) >> csr.lsb;
}

#ifdef __cplusplus
}
#endif

#endif
