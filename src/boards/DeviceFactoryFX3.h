#pragma once

#include "USBEntry.h"
#include "limesuite/DeviceHandle.h"

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
};

} // namespace lime
