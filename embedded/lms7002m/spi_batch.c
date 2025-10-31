#include "spi_batch.h"

#include "csr.h"
#include "lms7002m_context.h"

#include <string.h>

void spi_batch_init(struct spi_batch_t* batch)
{
    memset(batch, 0, sizeof(struct spi_batch_t));
}

int spi_batch_modify(struct spi_batch_t* batch, uint16_t address, uint8_t msb, uint8_t lsb, uint16_t value)
{
    const uint16_t spiMask = (~(~0u << (msb - lsb + 1))) << (lsb);
    uint8_t i = 0;
    for (; i < batch->length; ++i)
        if (address == batch->addr[i])
            break;
    if (i >= SPI_BATCH_MAX_LEN)
        return -1;
    if (i >= batch->length)
    {
        batch->length = i + 1;
        batch->addr[i] = address;
    }

    batch->mask[i] |= spiMask;
    batch->value[i] &= ~spiMask;
    batch->value[i] |= (value << lsb) & spiMask;
    return 0;
}

int spi_batch_modify_csr(struct spi_batch_t* batch, const struct lms7002m_csr csr, uint16_t value)
{
    return spi_batch_modify(batch, csr.address, csr.msb, csr.lsb, value);
}

int lms7002m_spi_batch_flush(struct spi_batch_t* batch, struct lms7002m_context* context)
{
    uint32_t mosi[SPI_BATCH_MAX_LEN];
    uint32_t miso[SPI_BATCH_MAX_LEN];
    uint8_t mosi_len = 0;
    for (uint8_t i = 0; i < batch->length; ++i)
    {
        if (batch->mask[i] != 0xFFFF)
            mosi[mosi_len++] = batch->addr[i] << 16;
    }
    {
        int status = context->hooks.spi16_transact(mosi, miso, mosi_len, context->hooks.spi16_userData);
        if (status != 0)
            return status;
    }

    uint8_t miso_i = 0;
    mosi_len = batch->length;
    for (uint8_t i = 0; i < batch->length; ++i)
    {
        mosi[i] = (1 << 31) | (batch->addr[i] << 16);
        if (batch->mask[i] != 0xFFFF)
            mosi[i] |= (miso[miso_i++] & (~batch->mask[i])) | batch->value[i];
        else
            mosi[i] |= batch->value[i];
    }
    batch->length = 0;
    return context->hooks.spi16_transact(mosi, 0, mosi_len, context->hooks.spi16_userData);
}
