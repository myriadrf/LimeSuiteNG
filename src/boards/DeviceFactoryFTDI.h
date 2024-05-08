#pragma once

#include "USBEntry.h"
#include "limesuiteng/DeviceHandle.h"

namespace lime {

/** @brief A class for LimeSDR Mini device registry entry. */
class DeviceFactoryFTDI : public USBEntry
{
  public:
    DeviceFactoryFTDI();

#ifndef __unix__
    std::vector<DeviceHandle> enumerate(const DeviceHandle& hint) override;
#endif

    SDRDevice* make(const DeviceHandle& handle) override;

  private:
    SDRDevice* make_LimeSDR_Mini(const DeviceHandle& handle, const uint16_t& vid, const uint16_t& pid);
};

} // namespace lime
