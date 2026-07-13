/* C wrapper for the GPIO subinterface, over lime::SDRDevice::GPIORead/GPIOWrite. */
#include "limesuiteng/gpio.h"
#include "private.h"

using lime::SDRDevice;

extern "C" {

lime_OpStatus lime_gpio_set_value(lime_GPIO* gpio, uint32_t pin, bool value)
{
    if (gpio == nullptr || pin >= 8)
        return lime_OpStatus_InvalidValue;
    SDRDevice* dev = reinterpret_cast<SDRDevice*>(gpio);
    uint8_t reg = 0;
    if (dev->GPIORead(&reg, 1) != lime::OpStatus::Success)
        return lime_OpStatus_IOFailure;
    reg = value ? static_cast<uint8_t>(reg | (1u << pin)) : static_cast<uint8_t>(reg & ~(1u << pin));
    return static_cast<lime_OpStatus>(dev->GPIOWrite(&reg, 1));
}

lime_OpStatus lime_gpio_get_value(lime_GPIO* gpio, uint32_t pin, bool* value)
{
    if (gpio == nullptr || value == nullptr || pin >= 8)
        return lime_OpStatus_InvalidValue;
    SDRDevice* dev = reinterpret_cast<SDRDevice*>(gpio);
    uint8_t reg = 0;
    const lime::OpStatus st = dev->GPIORead(&reg, 1);
    if (st != lime::OpStatus::Success)
        return static_cast<lime_OpStatus>(st);
    *value = (reg >> pin) & 1u;
    return lime_OpStatus_Success;
}

} /* extern "C" */
