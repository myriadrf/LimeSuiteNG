#pragma once

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

  private:
    SDRDevice* make_LimeSDR_Mini(const DeviceHandle& handle, const uint16_t& vid, const uint16_t& pid);
};

} // namespace lime
