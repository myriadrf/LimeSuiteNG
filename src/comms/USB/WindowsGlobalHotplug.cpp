#include "WindowsGlobalHotplug.h"

#include "Windows.h"
#include "cfgmgr32.h"

#include "comms/USB/IUSB.h"
#include "comms/USB/GlobalHotplugEvents.h"
#include "CommonFunctions.h"

#include <stdexcept>
#include <string>
#include <string_view>

using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

namespace lime {

namespace {

uint16_t StringHexToInt(std::wstring_view input)
{
    std::wstringstream ss{ std::wstring(input) };

    uint16_t value;
    ss >> std::hex >> value;

    return value;
}

IUSB::VendorProductId ParseVidPid(std::wstring_view input)
{
    const auto splits = SplitString(input, L"\\"sv);

    const auto separated = SplitString(splits[1], L"&"sv);

    const auto vidAsString = SplitString(separated[0], L"VID_"sv);
    const auto pidAsString = SplitString(separated[1], L"PID_"sv);

    const auto vID = StringHexToInt(vidAsString[0]);
    const auto pID = StringHexToInt(pidAsString[0]);

    return { vID, pID };
}

} // namespace

WindowsGlobalHotplug::WindowsGlobalHotplug()
{
    CM_NOTIFY_FILTER filter{};
    filter.cbSize = sizeof(filter);
    filter.Flags = CM_NOTIFY_FILTER_FLAG_ALL_DEVICE_INSTANCES;
    filter.FilterType = CM_NOTIFY_FILTER_TYPE_DEVICEINSTANCE;
    filter.u.DeviceInstance.InstanceId[0] = 0;
    auto returnValue = CM_Register_Notification(&filter, nullptr, globalCallback, &globalDeviceDisconnectCallbackHandle);

    if (returnValue != CR_SUCCESS)
    {
        throw std::runtime_error("CM_Register_Notification failed with error code "s + intToHex(returnValue));
    }
}

WindowsGlobalHotplug::~WindowsGlobalHotplug()
{
    CM_Unregister_Notification(globalDeviceDisconnectCallbackHandle);
}

DWORD CALLBACK WindowsGlobalHotplug::globalCallback(
    HCMNOTIFICATION hNotify, PVOID Context, CM_NOTIFY_ACTION Action, PCM_NOTIFY_EVENT_DATA EventData, DWORD EventDataSize)
{
    if (EventData->FilterType != CM_NOTIFY_FILTER_TYPE_DEVICEINSTANCE)
    {
        return ERROR_SUCCESS;
    }

    const auto& matchTo = GlobalHotplugEvents::GetVidPids();
    const auto vidPid = ParseVidPid(EventData->u.DeviceInstance.InstanceId);

    // Figure out if it's a device we care about
    if (matchTo.find(vidPid) == matchTo.end())
    {
        // It's not something we care about, this is the only value we can return here though
        return ERROR_SUCCESS;
    }

    if (CM_NOTIFY_ACTION_DEVICEINSTANCESTARTED == Action)
    {
        const auto& callbacks = GlobalHotplugEvents::GetConnectCallbacks();
        for (auto iter = callbacks.rbegin(); iter != callbacks.rend(); ++iter)
        {
            (*iter)();
        }
    }
    else if (CM_NOTIFY_ACTION_DEVICEINSTANCEREMOVED == Action)
    {
        const auto& callbacks = GlobalHotplugEvents::GetDisconnectCallbacks();
        for (auto iter = callbacks.rbegin(); iter != callbacks.rend(); ++iter)
        {
            (*iter)();
        }
    }

    return ERROR_SUCCESS;
}

} // namespace lime
