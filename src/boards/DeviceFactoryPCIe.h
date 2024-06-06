#pragma once

#include "limesuiteng/DeviceRegistry.h"

namespace lime {

class DeviceHandle;

/** @brief A class for enumerating and instantiating PCIe based devices. */
class DeviceFactoryPCIe : public DeviceRegistryEntry
{
  public:
    DeviceFactoryPCIe();
    std::vector<DeviceHandle> enumerate(const DeviceHandle& hint) override;
    SDRDevice* make(const DeviceHandle& handle) override;
};

} // namespace lime
