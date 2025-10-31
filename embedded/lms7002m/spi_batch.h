#ifndef LMS7002M_SPI_BATCH_H
#define LMS7002M_SPI_BATCH_H

#include "limesuiteng/embedded/result.h"
#include "limesuiteng/embedded/types.h"

#define SPI_BATCH_MAX_LEN 16

#ifdef __cplusplus
extern "C" {
#endif

struct lms7002m_context;
struct lms7002m_csr;

typedef struct spi_batch_t {
    uint16_t addr[SPI_BATCH_MAX_LEN];
    uint16_t mask[SPI_BATCH_MAX_LEN];
    uint16_t value[SPI_BATCH_MAX_LEN];
    uint8_t length;
} spi_batch_t;

// optimizes common SPI registers read/modify/write operations into single read/write batches
// Registers write order is not guaranteed.
// Does not take into account side effects of register value changes

void spi_batch_init(struct spi_batch_t* batch);
int spi_batch_modify(struct spi_batch_t* batch, uint16_t address, uint8_t msb, uint8_t lsb, uint16_t value);
int spi_batch_modify_csr(struct spi_batch_t* batch, const struct lms7002m_csr csr, uint16_t value);

int lms7002m_spi_batch_flush(struct spi_batch_t* batch, struct lms7002m_context* context);

#ifdef __cplusplus
} // extern C
#endif

#endif // LMS7002M_SPI_BATCH_H
