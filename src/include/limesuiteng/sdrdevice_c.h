/**
 * @file limesuiteng/sdrdevice_c.h
 * @author Lime Microsystems
 * @brief SDR device control: channel configuration and capability subinterfaces.
 */
#ifndef LIMESUITENG_SDRDEVICE_C_H
#define LIMESUITENG_SDRDEVICE_C_H

#include "limesuiteng/types_c.h"
#include "limesuiteng/sdrdescriptor_c.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Opaque SDR device handle. Obtained via lime_device_as_sdr(). */
typedef struct lime_SDRDevice lime_SDRDevice;

/**
 * @brief Sets the RF center frequency of a channel.
 * @param dev The device to configure.
 * @param module The RF SoC index.
 * @param dir The direction to configure.
 * @param channel The channel index within the RF SoC.
 * @param hz The frequency to set (in Hz).
 * @return The status of the operation.
 */
LIME_C_API lime_OpStatus lime_sdrdevice_set_frequency(
    lime_SDRDevice* dev, uint32_t module, lime_TRXDir dir, uint32_t channel, double hz);

/**
 * @brief Enables or disables a channel.
 * @param dev The device to configure.
 * @param module The RF SoC index.
 * @param dir The direction to configure.
 * @param channel The channel index within the RF SoC.
 * @param enable True to enable, false to disable.
 * @return The status of the operation.
 */
LIME_C_API lime_OpStatus lime_sdrdevice_enable_channel(
    lime_SDRDevice* dev, uint32_t module, lime_TRXDir dir, uint32_t channel, bool enable);

/**
 * @brief Selects the antenna path by name.
 * Antenna names are listed by lime_descriptor_antenna_name().
 * @param dev The device to configure.
 * @param module The RF SoC index.
 * @param dir The direction to configure.
 * @param channel The channel index within the RF SoC.
 * @param name The antenna name to select.
 * @return lime_OpStatus_InvalidValue when the name is unknown.
 */
LIME_C_API lime_OpStatus lime_sdrdevice_set_antenna(
    lime_SDRDevice* dev, uint32_t module, lime_TRXDir dir, uint32_t channel, const char* name);

/**
 * @brief Gets the device capability descriptor.
 * @param dev The device to read.
 * @return Library-owned descriptor, valid for the device's lifetime; NULL if @p dev is NULL.
 */
LIME_C_API const lime_SDRDescriptor* lime_sdrdevice_get_descriptor(lime_SDRDevice* dev);

/** @brief Opaque GPIO subinterface handle. */
typedef struct lime_GPIO lime_GPIO;

/**
 * @brief Gets the GPIO subinterface of a device.
 * @param dev The device to query.
 * @return The GPIO handle, or NULL if the capability is absent.
 */
LIME_C_API lime_GPIO* lime_sdrdevice_get_gpio(lime_SDRDevice* dev);

/**
 * @brief Sets the state of a single GPIO pin.
 * @param gpio The GPIO subinterface.
 * @param pin The pin index.
 * @param value The pin state to set.
 * @return The status of the operation.
 */
LIME_C_API lime_OpStatus lime_gpio_set_value(lime_GPIO* gpio, uint32_t pin, bool value);

/**
 * @brief Reads the state of a single GPIO pin.
 * @param gpio The GPIO subinterface.
 * @param pin The pin index.
 * @param[out] value The pin state read.
 * @return The status of the operation.
 */
LIME_C_API lime_OpStatus lime_gpio_get_value(lime_GPIO* gpio, uint32_t pin, bool* value);

/** @brief Opaque SPI subinterface handle. */
typedef struct lime_SPI lime_SPI;

/**
 * @brief Gets the SPI subinterface of a device.
 * @param dev The device to query.
 * @return The SPI handle, or NULL if the capability is absent.
 */
LIME_C_API lime_SPI* lime_sdrdevice_get_spi(lime_SDRDevice* dev);

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

#endif /* LIMESUITENG_SDRDEVICE_C_H */
