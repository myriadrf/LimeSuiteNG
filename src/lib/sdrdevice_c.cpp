/* C wrapper over lime::SDRDevice: channel control and the GPIO / SPI subinterfaces. */
#include "limesuiteng/sdrdevice_c.h"
#include "capi_private.h"

#include <string>

using lime::SDRDevice;

extern "C" {

lime_OpStatus lime_sdrdevice_set_frequency(
    lime_SDRDevice* dev, uint32_t module, lime_TRXDir d, uint32_t channel, double hz)
{
    if (dev == nullptr || narrows(module, channel))
        return lime_OpStatus_InvalidValue;
    return static_cast<lime_OpStatus>(
        sdr(dev)->SetFrequency(static_cast<uint8_t>(module), dir(d), static_cast<uint8_t>(channel), hz));
}

lime_OpStatus lime_sdrdevice_enable_channel(
    lime_SDRDevice* dev, uint32_t module, lime_TRXDir d, uint32_t channel, bool enable)
{
    if (dev == nullptr || narrows(module, channel))
        return lime_OpStatus_InvalidValue;
    return static_cast<lime_OpStatus>(
        sdr(dev)->EnableChannel(static_cast<uint8_t>(module), dir(d), static_cast<uint8_t>(channel), enable));
}

lime_OpStatus lime_sdrdevice_set_antenna(
    lime_SDRDevice* dev, uint32_t module, lime_TRXDir d, uint32_t channel, const char* name)
{
    if (dev == nullptr || name == nullptr || narrows(module, channel))
        return lime_OpStatus_InvalidValue;
    const lime::ChannelId c{ static_cast<uint8_t>(module), dir(d), static_cast<uint8_t>(channel) };
    return static_cast<lime_OpStatus>(sdr(dev)->SetAntenna(c, std::string(name)));
}

const lime_SDRDescriptor* lime_sdrdevice_get_descriptor(lime_SDRDevice* dev)
{
    if (dev == nullptr)
        return nullptr;
    return reinterpret_cast<const lime_SDRDescriptor*>(&sdr(dev)->GetDescriptor());
}

/* Transitional: GPIO capability lives on SDRDevice; the handle is the device. */
lime_GPIO* lime_sdrdevice_get_gpio(lime_SDRDevice* dev)
{
    return reinterpret_cast<lime_GPIO*>(dev);
}

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

/* Transitional: SPI capability lives on SDRDevice; the handle is the device. */
lime_SPI* lime_sdrdevice_get_spi(lime_SDRDevice* dev)
{
    return reinterpret_cast<lime_SPI*>(dev);
}

lime_OpStatus lime_spi_transact(lime_SPI* spi, uint32_t bus_address, const uint32_t* mosi, uint32_t* miso, uint32_t count)
{
    if (spi == nullptr || (mosi == nullptr && miso == nullptr) || count == 0)
        return lime_OpStatus_InvalidValue;
    SDRDevice* dev = reinterpret_cast<SDRDevice*>(spi);
    return static_cast<lime_OpStatus>(dev->SPI(bus_address, mosi, miso, count));
}

} /* extern "C" */
