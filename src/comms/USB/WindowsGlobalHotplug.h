#ifndef LIME_WINDOWS_GLOBAL_HOTPLUG_H
#define LIME_WINDOWS_GLOBAL_HOTPLUG_H

#include "Windows.h"
#include "cfgmgr32.h"

namespace lime {
class WindowsGlobalHotplug
{
  public:
    WindowsGlobalHotplug();
    ~WindowsGlobalHotplug();

  private:
    HCMNOTIFICATION globalDeviceDisconnectCallbackHandle{};

    static DWORD CALLBACK globalCallback(
        HCMNOTIFICATION hNotify, PVOID Context, CM_NOTIFY_ACTION Action, PCM_NOTIFY_EVENT_DATA EventData, DWORD EventDataSize);
};
} // namespace lime

#endif // LIME_WINDOWS_GLOBAL_HOTPLUG_H
