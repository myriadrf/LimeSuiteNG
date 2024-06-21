#include "GlobalHotplugEvents.h"

#include "comms/USB/IUSB.h"

#include <algorithm>
#include <set>

namespace lime {

std::set<IUSB::VendorProductId> GlobalHotplugEvents::vidPidPairsToLookFor{};

std::vector<SDRDevice::CallbackInfo<GlobalHotplugEvents::GlobalHotplugConnectCallbackType>>
    GlobalHotplugEvents::globalHotplugConnectCallbacks{};
std::vector<SDRDevice::CallbackInfo<GlobalHotplugEvents::GlobalHotplugDisconnectCallbackType>>
    GlobalHotplugEvents::globalHotplugDisconnectCallbacks{};

#ifdef __unix__
USBGeneric GlobalHotplugEvents::hotplug{};
#else
WindowsGlobalHotplug GlobalHotplugEvents::hotplug{};
#endif

void GlobalHotplugEvents::AddVidPids(const std::set<IUSB::VendorProductId>& ids)
{
    vidPidPairsToLookFor.insert(ids.begin(), ids.end());
}

std::set<IUSB::VendorProductId> GlobalHotplugEvents::GetVidPids()
{
    return vidPidPairsToLookFor;
}

std::vector<SDRDevice::CallbackInfo<GlobalHotplugEvents::GlobalHotplugConnectCallbackType>>
GlobalHotplugEvents::GetConnectCallbacks()
{
    return globalHotplugConnectCallbacks;
}

std::vector<SDRDevice::CallbackInfo<GlobalHotplugEvents::GlobalHotplugDisconnectCallbackType>>
GlobalHotplugEvents::GetDisconnectCallbacks()
{
    return globalHotplugDisconnectCallbacks;
}

std::size_t GlobalHotplugEvents::AddGlobalHotplugConnectCallback(const GlobalHotplugConnectCallbackType& function, void* userData)
{
    return AddCallback(globalHotplugConnectCallbacks, function, userData);
}

void GlobalHotplugEvents::RemoveGlobalHotplugConnectCallback(std::size_t id)
{
    RemoveCallback(globalHotplugConnectCallbacks, id);
}

std::size_t GlobalHotplugEvents::AddGlobalHotplugDisconnectCallback(
    const GlobalHotplugDisconnectCallbackType& function, void* userData)
{
    return AddCallback(globalHotplugDisconnectCallbacks, function, userData);
}

void GlobalHotplugEvents::RemoveGlobalHotplugDisconnectCallback(std::size_t id)
{
    RemoveCallback(globalHotplugDisconnectCallbacks, id);
}

template<typename T>
std::size_t GlobalHotplugEvents::AddCallback(std::vector<SDRDevice::CallbackInfo<T>>& vector, const T& function, void* userData)
{
    std::size_t id = 0;

    if (!vector.empty())
    {
        // As long as elements are not out of order this guarantees a unique ID in the array.
        id = vector.back().id + 1;
    }

    vector.push_back({ function, userData, id });

    return id;
}

template<typename T> void GlobalHotplugEvents::RemoveCallback(std::vector<SDRDevice::CallbackInfo<T>>& vector, std::size_t id)
{
    vector.erase(
        std::remove_if(vector.begin(), vector.end(), [&id](const SDRDevice::CallbackInfo<T>& info) { return id == info.id; }),
        vector.end());
}

} // namespace lime
