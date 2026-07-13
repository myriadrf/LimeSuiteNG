/**
 * @file limesuiteng/device_registry_c.h
 * @author Lime Microsystems
 * @brief Device enumeration, connection, and the device tree.
 */
#ifndef LIMESUITENG_DEVICE_REGISTRY_C_H
#define LIMESUITENG_DEVICE_REGISTRY_C_H

#include "limesuiteng/types_c.h"
#include "limesuiteng/sdrdevice_c.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Opaque generic device handle: an SDR, an RF front-end, or a utility device. */
typedef struct lime_device lime_device;

/** @brief Serialized identifier of a connectable device. */
typedef struct {
    char str[256]; ///< The device identifier as a key=value string
} lime_DeviceHandle;

/**
 * @brief Enumerates the connectable devices.
 * @param[out] out Buffer for the found device handles, or NULL to only count.
 * @param max The capacity of @p out.
 * @return The number of devices found (may exceed @p max), or a negative lime_OpStatus on failure.
 */
LIME_API int lime_enumerate(lime_DeviceHandle* out, size_t max);

/**
 * @brief Opens a connection to a device.
 * @param handle The device to connect to, as returned by lime_enumerate().
 * @return The connected device, or NULL on failure. Release with lime_device_close().
 */
LIME_API lime_device* lime_device_open(const lime_DeviceHandle* handle);

/**
 * @brief Closes a device connection and frees the handle.
 * @param dev The device to close; NULL is allowed and ignored.
 */
LIME_API void lime_device_close(lime_device* dev);

/** @brief The kinds of devices in the device tree. */
typedef enum {
    lime_Kind_SDR = 0, ///< Software-defined radio
    lime_Kind_RFE = 1, ///< RF front-end
    lime_Kind_Utility = 2 ///< Auxiliary device
} lime_Kind;

/**
 * @brief Gets the kind of a device.
 * @param dev The device to query.
 * @return The device kind.
 */
LIME_API lime_Kind lime_device_kind(lime_device* dev);

/**
 * @brief Gets the number of subdevices of a device.
 * @param dev The device to query.
 * @return The subdevice count.
 */
LIME_API size_t lime_device_child_count(lime_device* dev);

/**
 * @brief Gets a subdevice of a device.
 * @param dev The device to query.
 * @param index The subdevice index.
 * @return The subdevice, or NULL when out of range. Owned by the parent; do not close.
 */
LIME_API lime_device* lime_device_child(lime_device* dev, size_t index);

/**
 * @brief Gets the SDR interface of a device.
 * @param dev The device to query.
 * @return The SDR interface, or NULL if the device is not an SDR.
 */
LIME_API lime_SDRDevice* lime_device_as_sdr(lime_device* dev);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* LIMESUITENG_DEVICE_REGISTRY_C_H */
