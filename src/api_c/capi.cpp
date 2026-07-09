/*
 * LimeSuiteNG public C API -- transitional implementation.
 *
 * A thin extern "C" wrapper over the C++ core. During the transition the
 * generic device tree collapses onto SDRDevice (the only device class so
 * far): lime_device and lime_SDRDevice are the same underlying pointer.
 * Later phases introduce the real generic device / RFE / Utility split.
 */
#include "limesuiteng/c/limesuiteng.h"

#include "limesuiteng/DeviceRegistry.h"
#include "limesuiteng/DeviceHandle.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/types.h"
#include "limesuiteng/VersionInfo.h"

#include <cstring>
#include <string>
#include <vector>

using lime::SDRDevice;

static inline SDRDevice* sdr(lime_SDRDevice* d) { return reinterpret_cast<SDRDevice*>(d); }
static inline lime::TRXDir dir(lime_TRXDir d) { return static_cast<lime::TRXDir>(d != lime_TRXDir_Rx); }

extern "C" {

int lime_enumerate(lime_DeviceHandle* out, size_t max)
{
    const std::vector<lime::DeviceHandle> handles = lime::DeviceRegistry::enumerate();
    if (out != nullptr)
    {
        const size_t n = handles.size() < max ? handles.size() : max;
        for (size_t i = 0; i < n; ++i)
        {
            const std::string s = handles[i].Serialize();
            std::strncpy(out[i].str, s.c_str(), sizeof(out[i].str) - 1);
            out[i].str[sizeof(out[i].str) - 1] = '\0';
        }
    }
    return static_cast<int>(handles.size());
}

lime_device* lime_device_open(const lime_DeviceHandle* handle)
{
    if (handle == nullptr)
        return nullptr;
    const lime::DeviceHandle h{ std::string(handle->str) };
    return reinterpret_cast<lime_device*>(lime::DeviceRegistry::makeDevice(h));
}

void lime_device_close(lime_device* dev)
{
    if (dev != nullptr)
        lime::DeviceRegistry::freeDevice(reinterpret_cast<SDRDevice*>(dev));
}

/* Transitional: the only device kind is an SDR, and it is the same pointer. */
lime_Kind lime_device_kind(lime_device*) { return lime_Kind_SDR; }
size_t lime_device_child_count(lime_device*) { return 0; }
lime_device* lime_device_child(lime_device*, size_t) { return nullptr; }
lime_SDRDevice* lime_device_as_sdr(lime_device* dev) { return reinterpret_cast<lime_SDRDevice*>(dev); }

lime_OpStatus lime_sdrdevice_set_frequency(
    lime_SDRDevice* dev, uint8_t module, lime_TRXDir d, uint8_t channel, double hz)
{
    if (dev == nullptr)
        return lime_OpStatus_InvalidValue;
    return static_cast<lime_OpStatus>(sdr(dev)->SetFrequency(module, dir(d), channel, hz));
}

lime_OpStatus lime_sdrdevice_enable_channel(
    lime_SDRDevice* dev, uint8_t module, lime_TRXDir d, uint8_t channel, bool enable)
{
    if (dev == nullptr)
        return lime_OpStatus_InvalidValue;
    return static_cast<lime_OpStatus>(sdr(dev)->EnableChannel(module, dir(d), channel, enable));
}

lime_OpStatus lime_sdrdevice_set_antenna(
    lime_SDRDevice* dev, uint8_t module, lime_TRXDir d, uint8_t channel, const char* name)
{
    if (dev == nullptr || name == nullptr)
        return lime_OpStatus_InvalidValue;
    const lime::ChannelId c{ module, dir(d), channel };
    return static_cast<lime_OpStatus>(sdr(dev)->SetAntenna(c, std::string(name)));
}

const lime_SDRDescriptor* lime_sdrdevice_get_descriptor(lime_SDRDevice* dev)
{
    if (dev == nullptr)
        return nullptr;
    return reinterpret_cast<const lime_SDRDescriptor*>(&sdr(dev)->GetDescriptor());
}

/* Transitional: GPIO capability lives on SDRDevice; the handle is the device. */
lime_GPIO* lime_sdrdevice_get_gpio(lime_SDRDevice* dev) { return reinterpret_cast<lime_GPIO*>(dev); }

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

const char* lime_get_library_version(void)
{
    static const std::string v = lime::GetLibraryVersion();
    return v.c_str();
}
const char* lime_get_api_version(void)
{
    static const std::string v = lime::GetAPIVersion();
    return v.c_str();
}
const char* lime_get_abi_version(void)
{
    static const std::string v = lime::GetABIVersion();
    return v.c_str();
}

} /* extern "C" */
