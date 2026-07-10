/**
 * @file limesuiteng/spi_c.h
 * @author Lime Microsystems
 * @brief SPI bus access subinterface.
 */
#ifndef LIMESUITENG_SPI_C_H
#define LIMESUITENG_SPI_C_H

#include "limesuiteng/types_c.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Opaque SPI subinterface handle. Obtained via lime_sdrdevice_get_spi(). */
typedef struct lime_SPI lime_SPI;

/**
 * @brief Performs a full-duplex SPI transaction on an internal bus.
 * @param spi The SPI subinterface.
 * @param bus_address The chip-select id of the internal bus.
 * @param mosi Data to write, or NULL for a read-only transfer.
 * @param[out] miso Buffer for data read, or NULL for a write-only transfer.
 * @param count The number of 32-bit words to transfer.
 * @return The status of the operation.
 */
LIME_C_API lime_OpStatus lime_spi_transact(
    lime_SPI* spi, uint32_t bus_address, const uint32_t* mosi, uint32_t* miso, uint32_t count);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* LIMESUITENG_SPI_C_H */
