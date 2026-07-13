/**
 * @file limesuiteng/gpio.h
 * @author Lime Microsystems
 * @brief GPIO pin control subinterface.
 */
#ifndef LIMESUITENG_GPIO_C_H
#define LIMESUITENG_GPIO_C_H

#include "limesuiteng/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Opaque GPIO subinterface handle. Obtained via lime_sdrdevice_get_gpio(). */
typedef struct lime_GPIO lime_GPIO;

/**
 * @brief Sets the state of a single GPIO pin.
 * @param gpio The GPIO subinterface.
 * @param pin The pin index.
 * @param value The pin state to set.
 * @return The status of the operation.
 */
LIME_API lime_OpStatus lime_gpio_set_value(lime_GPIO* gpio, uint32_t pin, bool value);

/**
 * @brief Reads the state of a single GPIO pin.
 * @param gpio The GPIO subinterface.
 * @param pin The pin index.
 * @param[out] value The pin state read.
 * @return The status of the operation.
 */
LIME_API lime_OpStatus lime_gpio_get_value(lime_GPIO* gpio, uint32_t pin, bool* value);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* LIMESUITENG_GPIO_C_H */
