#include "WindowsHotplug.h"

#include <Windows.h>
#include <cfgmgr32.h>

#include "CommonFunctions.h"
#include "IUSB.h"

#include <algorithm>
#include <codecvt>
#include <locale>
#include <stdexcept>
#include <string>
#include <string_view>

using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

namespace lime {

namespace {

template<typename strView> std::vector<strView> SplitString(strView string, strView delimiter)
{
    std::vector<strView> ret;

    auto position{ string.find(delimiter) };

    while (position != strView::npos)
    {
        if (position != 0)
        {
            ret.push_back(string.substr(0, position));
        }

        string = string.substr(position + delimiter.size());
        position = string.find(delimiter);
    }

    if (string.size() > 0)
    {
        ret.push_back(string);
    }
    return ret;
}

std::string_view GetSpecificDeviceIDFromDeviceIDListBySerial(const std::vector<std::string_view>& list, const std::string& serial)
{
    for (const auto& deviceID : list)
    {
        if (deviceID.find(serial) != std::string_view::npos)
        {
            return deviceID;
        }
    }

    throw std::runtime_error("Serial not found"s);
}

std::string GetDeviceID(uint16_t vid, uint16_t pid, const std::string& serial)
{
    ULONG bufferSize = 0;
    const std::string filter = "USB\\VID_"s + intToHex(vid, true) + "&PID_"s + intToHex(pid, true);
    const ULONG flags = CM_GETIDLIST_FILTER_ENUMERATOR;

    CONFIGRET status = CM_Get_Device_ID_List_Size(&bufferSize, filter.c_str(), flags);
    if (status == CR_NO_SUCH_VALUE) // no device like that exists, return
    {
        return ""s;
    }
    else if (status != CR_SUCCESS)
    {
        throw std::runtime_error("CM_Get_Device_ID_List_Size failed"s);
    }

    const PZZSTR buffer = new CHAR[bufferSize];

    status = CM_Get_Device_ID_List(filter.c_str(), buffer, bufferSize, flags);
    if (status != CR_SUCCESS)
    {
        throw std::runtime_error("CM_Get_Device_ID_List failed"s);
    }

    const std::string_view allDeviceIDs{ buffer, bufferSize };
    const auto& allDeviceIDsSplits = SplitString(allDeviceIDs, "\0"sv);

    const auto deviceID = std::string{ GetSpecificDeviceIDFromDeviceIDListBySerial(allDeviceIDsSplits, serial) };

    delete[] buffer;
    return deviceID;
}

} // namespace

WindowsHotplug ::~WindowsHotplug()
{
    CM_Unregister_Notification(deviceDisconnectCallbackHandle);
}

void WindowsHotplug::AddOnHotplugDisconnectCallback(const IUSB::HotplugDisconnectCallbackType& function, void* userData)
{
    hotplugDisconnectCallbacks.push_back({ function, userData });
}

void WindowsHotplug::AddDeviceToReceiveHotplugDisconnectEvents(uint16_t vid, uint16_t pid, const std::string& serial)
{
    const auto deviceID = GetDeviceID(vid, pid, serial);
    const auto wideDeviceID = std::wstring_convert<std::codecvt_utf8<wchar_t>>().from_bytes(deviceID);

    CM_NOTIFY_FILTER filter{};
    filter.cbSize = sizeof(filter);
    filter.FilterType = CM_NOTIFY_FILTER_TYPE_DEVICEINSTANCE;
    std::wcsncpy(filter.u.DeviceInstance.InstanceId,
        wideDeviceID.c_str(),
        std::min(sizeof(filter.u.DeviceInstance.InstanceId), wideDeviceID.size()));

    auto returnValue = CM_Register_Notification(&filter, this, callback, &deviceDisconnectCallbackHandle);
    if (returnValue != CR_SUCCESS)
    {
        throw std::runtime_error("CM_Register_Notification failed with error code "s + intToHex(returnValue));
    }
}

DWORD WindowsHotplug::callback(
    HCMNOTIFICATION hNotify, PVOID Context, CM_NOTIFY_ACTION Action, PCM_NOTIFY_EVENT_DATA EventData, DWORD EventDataSize)
{
    if (CM_NOTIFY_ACTION_DEVICEINSTANCEREMOVED == Action)
    {
        auto* hotplug = reinterpret_cast<WindowsHotplug*>(Context);

        for (auto iter = hotplug->hotplugDisconnectCallbacks.rbegin(); iter != hotplug->hotplugDisconnectCallbacks.rend(); ++iter)
        {
            (*iter)();
        }
    }

    return ERROR_SUCCESS;
}

} // namespace lime
