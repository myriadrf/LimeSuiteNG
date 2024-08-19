#pragma once

#include <cstdint>

#include "limesuiteng/DeviceRegistry.h"

namespace lime {

class DeviceHandle;

/** @brief A class for LimeSDR Mini device registry entry. */
class DeviceFactoryFTDI : public DeviceRegistryEntry
{
  public:
    DeviceFactoryFTDI();
    std::vector<DeviceHandle> enumerate(const DeviceHandle& hint) override;
    SDRDevice* make(const DeviceHandle& handle) override;
};

} // namespace lime
