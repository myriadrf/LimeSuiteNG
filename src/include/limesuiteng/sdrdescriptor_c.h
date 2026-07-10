/**
 * @file limesuiteng/sdrdescriptor_c.h
 * @author Lime Microsystems
 * @brief Read-only access to static device capability information.
 */
#ifndef LIMESUITENG_SDRDESCRIPTOR_C_H
#define LIMESUITENG_SDRDESCRIPTOR_C_H

#include "limesuiteng/types_c.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Opaque device descriptor handle. Library-owned: pointers and strings
 * obtained through the accessors stay valid for the device's lifetime. */
typedef struct lime_SDRDescriptor lime_SDRDescriptor;

/**
 * @brief Gets the displayable name of the device.
 * @param d The descriptor to read.
 * @return The device name, or NULL if @p d is NULL.
 */
LIME_C_API const char* lime_descriptor_name(const lime_SDRDescriptor* d);

/**
 * @brief Gets the unique serial number of the device.
 * @param d The descriptor to read.
 * @return The serial number, or 0 if @p d is NULL.
 */
LIME_C_API uint64_t lime_descriptor_serial(const lime_SDRDescriptor* d);

/**
 * @brief Gets the number of RF SoC modules in the device.
 * @param d The descriptor to read.
 * @return The RF SoC count, or 0 if @p d is NULL.
 */
LIME_C_API size_t lime_descriptor_rfsoc_count(const lime_SDRDescriptor* d);

/**
 * @brief Gets the name of an RF SoC module.
 * @param d The descriptor to read.
 * @param soc The RF SoC index.
 * @return The module name, or NULL when out of range.
 */
LIME_C_API const char* lime_descriptor_rfsoc_name(const lime_SDRDescriptor* d, size_t soc);

/**
 * @brief Gets the channel count of an RF SoC module.
 * @param d The descriptor to read.
 * @param soc The RF SoC index.
 * @return The channel count, or 0 when out of range.
 */
LIME_C_API uint32_t lime_descriptor_channel_count(const lime_SDRDescriptor* d, size_t soc);

/**
 * @brief Gets the number of antenna paths of an RF SoC module for a direction.
 * @param d The descriptor to read.
 * @param soc The RF SoC index.
 * @param dir The direction to query.
 * @return The antenna count, or 0 when out of range.
 */
LIME_C_API size_t lime_descriptor_antenna_count(const lime_SDRDescriptor* d, size_t soc, lime_TRXDir dir);

/**
 * @brief Gets an antenna path name; the vocabulary for lime_sdrdevice_set_antenna().
 * @param d The descriptor to read.
 * @param soc The RF SoC index.
 * @param dir The direction to query.
 * @param index The antenna path index.
 * @return The antenna name, or NULL when out of range.
 */
LIME_C_API const char* lime_descriptor_antenna_name(
    const lime_SDRDescriptor* d, size_t soc, lime_TRXDir dir, size_t index);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* LIMESUITENG_SDRDESCRIPTOR_C_H */
