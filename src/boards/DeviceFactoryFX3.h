#pragma once

#include "USBEntry.h"
#include "limesuiteng/DeviceHandle.h"

namespace lime {

/** @brief A class for a LimeSDR-USB device entry. */
class DeviceFactoryFX3 : public USBEntry
{
  public:
    DeviceFactoryFX3();

#ifndef __unix__
    virtual std::vector<DeviceHandle> enumerate(const DeviceHandle& hint) override;
#endif
    virtual SDRDevice* make(const DeviceHandle& handle) override;

  private:
    SDRDevice* make_LimeSDR(const DeviceHandle& handle, const uint16_t& vid, const uint16_t& pid);
};

} // namespace lime
