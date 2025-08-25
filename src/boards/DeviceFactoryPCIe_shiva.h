#pragma once

#include "limesuiteng/DeviceRegistry.h"

namespace lime {

class DeviceHandle;

/** @brief A class for enumerating and instantiating PCIe based devices. */
class DeviceFactoryPCIe_shiva : public DeviceRegistryEntry
{
  public:
    DeviceFactoryPCIe_shiva();
    std::vector<DeviceHandle> enumerate(const DeviceHandle& hint) override;
    SDRDevice* make(const DeviceHandle& handle) override;
};

} // namespace lime
