/**
 * @file limesuiteng/sdrdevice_c.h
 * @author Lime Microsystems
 * @brief SDR device control: channel configuration and capability subinterfaces.
 */
#ifndef LIMESUITENG_SDRDEVICE_C_H
#define LIMESUITENG_SDRDEVICE_C_H

#include "limesuiteng/types_c.h"
#include "limesuiteng/sdrdescriptor_c.h"
#include "limesuiteng/gpio_c.h"
#include "limesuiteng/spi_c.h"

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
LIME_API lime_OpStatus lime_sdrdevice_set_frequency(
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
LIME_API lime_OpStatus lime_sdrdevice_enable_channel(
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
LIME_API lime_OpStatus lime_sdrdevice_set_antenna(
    lime_SDRDevice* dev, uint32_t module, lime_TRXDir dir, uint32_t channel, const char* name);

/**
 * @brief Gets the name of the currently selected antenna path.
 * @param dev The device to read.
 * @param module The RF SoC index.
 * @param dir The direction to read.
 * @param channel The channel index within the RF SoC.
 * @return Library-owned antenna name, valid for the device's lifetime; NULL when unavailable.
 */
LIME_API const char* lime_sdrdevice_get_antenna(lime_SDRDevice* dev, uint32_t module, lime_TRXDir dir, uint32_t channel);

/**
 * @brief Gets the device capability descriptor.
 * @param dev The device to read.
 * @return Library-owned descriptor, valid for the device's lifetime; NULL if @p dev is NULL.
 */
LIME_API const lime_SDRDescriptor* lime_sdrdevice_get_descriptor(lime_SDRDevice* dev);

/**
 * @brief Gets the GPIO subinterface of a device.
 * @param dev The device to query.
 * @return The GPIO handle, or NULL if the capability is absent.
 */
LIME_API lime_GPIO* lime_sdrdevice_get_gpio(lime_SDRDevice* dev);

/**
 * @brief Gets the SPI subinterface of a device.
 * @param dev The device to query.
 * @return The SPI handle, or NULL if the capability is absent.
 */
LIME_API lime_SPI* lime_sdrdevice_get_spi(lime_SDRDevice* dev);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* LIMESUITENG_SDRDEVICE_C_H */
