#pragma once

#include "limesuiteng/DeviceRegistry.h"

namespace lime {

class DeviceHandle;

/** @brief A class for enumerating and instantiating Xillybus based devices. */
class DeviceFactoryXillybus : public DeviceRegistryEntry
{
  public:
    DeviceFactoryXillybus();
    std::vector<DeviceHandle> enumerate(const DeviceHandle& hint) override;
    SDRDevice* make(const DeviceHandle& handle) override;
};

} // namespace lime
