#ifndef LIME_WINDOWS_HOTPLUG_H
#define LIME_WINDOWS_HOTPLUG_H

#include "Windows.h"
#include "cfgmgr32.h"

#include "comms/USB/IUSB.h"

#include <cstdint>
#include <string>
#include <vector>

namespace lime {

class WindowsHotplug
{
  public:
    ~WindowsHotplug();

    void AddOnHotplugDisconnectCallback(const IUSB::HotplugDisconnectCallbackType& function, void* userData);

    void AddDeviceToReceiveHotplugDisconnectEvents(uint16_t vid, uint16_t pid, const std::string& serial);

  private:
    std::vector<IUSB::CallbackInfo<IUSB::HotplugDisconnectCallbackType>> hotplugDisconnectCallbacks{};
    HCMNOTIFICATION deviceDisconnectCallbackHandle{};

    static DWORD callback(
        HCMNOTIFICATION hNotify, PVOID Context, CM_NOTIFY_ACTION Action, PCM_NOTIFY_EVENT_DATA EventData, DWORD EventDataSize);
};

} // namespace lime

#endif // LIME_WINDOWS_HOTPLUG_H
