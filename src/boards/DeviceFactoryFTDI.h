#pragma once

#include "USBEntry.h"
#include "limesuite/DeviceHandle.h"

namespace lime {

/** @brief A class for LimeSDR Mini device registry entry. */
class DeviceFactoryFTDI : public USBEntry
{
  public:
    DeviceFactoryFTDI();

#ifndef __unix__
    virtual std::vector<DeviceHandle> enumerate(const DeviceHandle& hint) override;
#endif

    virtual SDRDevice* make(const DeviceHandle& handle) override;

  private:
    SDRDevice* make_LimeSDR_Mini(const DeviceHandle& handle, const uint16_t& vid, const uint16_t& pid);
};

} // namespace lime
