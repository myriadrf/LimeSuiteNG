#ifndef LMS7002M_SPI_H
#define LMS7002M_SPI_H

#include "limesuiteng/embedded/result.h"
#include "limesuiteng/embedded/types.h"

#ifdef __cplusplus
extern "C" {
#endif

struct lms7002m_context;
struct lms7002m_csr;

void lms7002m_spi_write(struct lms7002m_context* self, uint16_t address, uint16_t value);
uint16_t lms7002m_spi_read(struct lms7002m_context* self, uint16_t address);
lime_Result lms7002m_spi_modify(struct lms7002m_context* self, uint16_t address, uint8_t msb, uint8_t lsb, uint16_t value);
lime_Result lms7002m_spi_modify_csr(struct lms7002m_context* self, const struct lms7002m_csr csr, uint16_t value);
uint16_t lms7002m_spi_read_bits(struct lms7002m_context* self, uint16_t address, uint8_t msb, uint8_t lsb);
uint16_t lms7002m_spi_read_csr(struct lms7002m_context* self, const struct lms7002m_csr csr);

#ifdef __cplusplus
} // extern C
#endif

#endif // LMS7002M_SPI_H
