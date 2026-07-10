/* C wrapper over lime::SDRDevice: channel control and capability subinterface getters. */
#include "limesuiteng/sdrdevice_c.h"
#include "private.h"

#include <string_view>

using lime::SDRDevice;

extern "C" {

lime_OpStatus lime_sdrdevice_set_frequency(lime_SDRDevice* dev, uint32_t module, lime_TRXDir d, uint32_t channel, double hz)
{
    if (dev == nullptr || narrows(module, channel))
        return lime_OpStatus_InvalidValue;
    return static_cast<lime_OpStatus>(
        sdr(dev)->SetFrequency(static_cast<uint8_t>(module), dir(d), static_cast<uint8_t>(channel), hz));
}

lime_OpStatus lime_sdrdevice_enable_channel(lime_SDRDevice* dev, uint32_t module, lime_TRXDir d, uint32_t channel, bool enable)
{
    if (dev == nullptr || narrows(module, channel))
        return lime_OpStatus_InvalidValue;
    return static_cast<lime_OpStatus>(
        sdr(dev)->EnableChannel(static_cast<uint8_t>(module), dir(d), static_cast<uint8_t>(channel), enable));
}

lime_OpStatus lime_sdrdevice_set_antenna(lime_SDRDevice* dev, uint32_t module, lime_TRXDir d, uint32_t channel, const char* name)
{
    if (dev == nullptr || name == nullptr || narrows(module, channel))
        return lime_OpStatus_InvalidValue;
    const auto& soc = sdr(dev)->GetDescriptor().rfSOC;
    if (module >= soc.size())
        return lime_OpStatus_InvalidValue;
    const auto it = soc[module].pathNames.find(dir(d));
    if (it == soc[module].pathNames.end())
        return lime_OpStatus_InvalidValue;
    const auto& names = it->second;
    const std::string_view wanted{ name };
    for (uint8_t i = 0; i < names.size(); ++i)
        if (names[i] == wanted)
            return static_cast<lime_OpStatus>(
                sdr(dev)->SetAntenna(static_cast<uint8_t>(module), dir(d), static_cast<uint8_t>(channel), i));
    return lime_OpStatus_InvalidValue;
}

const char* lime_sdrdevice_get_antenna(lime_SDRDevice* dev, uint32_t module, lime_TRXDir d, uint32_t channel)
{
    if (dev == nullptr || narrows(module, channel))
        return nullptr;
    const auto& soc = sdr(dev)->GetDescriptor().rfSOC;
    if (module >= soc.size())
        return nullptr;
    const auto it = soc[module].pathNames.find(dir(d));
    if (it == soc[module].pathNames.end())
        return nullptr;
    const uint8_t path = sdr(dev)->GetAntenna(static_cast<uint8_t>(module), dir(d), static_cast<uint8_t>(channel));
    return path < it->second.size() ? it->second[path].c_str() : nullptr;
}

const lime_SDRDescriptor* lime_sdrdevice_get_descriptor(lime_SDRDevice* dev)
{
    if (dev == nullptr)
        return nullptr;
    return reinterpret_cast<const lime_SDRDescriptor*>(&sdr(dev)->GetDescriptor());
}

/* Transitional: the GPIO and SPI capabilities live on SDRDevice; the handles are the device. */
lime_GPIO* lime_sdrdevice_get_gpio(lime_SDRDevice* dev)
{
    return reinterpret_cast<lime_GPIO*>(dev);
}

lime_SPI* lime_sdrdevice_get_spi(lime_SDRDevice* dev)
{
    return reinterpret_cast<lime_SPI*>(dev);
}

} /* extern "C" */
