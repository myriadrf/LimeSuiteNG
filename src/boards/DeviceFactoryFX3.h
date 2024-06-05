#pragma once

#include "limesuiteng/DeviceRegistry.h"

namespace lime {

class DeviceHandle;

/** @brief A class for a LimeSDR-USB device entry. */
class DeviceFactoryFX3 : DeviceRegistryEntry
{
  public:
    DeviceFactoryFX3();
    std::vector<DeviceHandle> enumerate(const DeviceHandle& hint) override;
    SDRDevice* make(const DeviceHandle& handle) override;

  private:
    SDRDevice* make_LimeSDR(const DeviceHandle& handle, const uint16_t& vid, const uint16_t& pid);
};

} // namespace lime
