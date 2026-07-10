/* C wrapper over lime::DeviceRegistry / lime::DeviceHandle. */
#include "limesuiteng/device_registry_c.h"
#include "capi_private.h"

#include "limesuiteng/DeviceRegistry.h"
#include "limesuiteng/DeviceHandle.h"

#include <cstring>
#include <string>
#include <vector>

using lime::SDRDevice;

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
lime_Kind lime_device_kind(lime_device*)
{
    return lime_Kind_SDR;
}

size_t lime_device_child_count(lime_device*)
{
    return 0;
}

lime_device* lime_device_child(lime_device*, size_t)
{
    return nullptr;
}

lime_SDRDevice* lime_device_as_sdr(lime_device* dev)
{
    return reinterpret_cast<lime_SDRDevice*>(dev);
}

} /* extern "C" */
