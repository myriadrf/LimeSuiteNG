#ifndef LIME_GLOBALHOTPLUGEVENTS_H
#define LIME_GLOBALHOTPLUGEVENTS_H

#include "comms/USB/IUSB.h"
#include "limesuiteng/config.h"
#include "limesuiteng/SDRDevice.h"

#ifdef __unix__
    #include "comms/USB/USBGeneric.h"
#else
    #include "comms/USB/WindowsGlobalHotplug.h"
#endif

namespace lime {

class GlobalHotplugEvents
{
  public:
    static void AddVidPids(const std::set<IUSB::VendorProductId>& ids);
    static std::set<IUSB::VendorProductId> GetVidPids();

    typedef std::function<void(void* userData)> GlobalHotplugConnectCallbackType;
    typedef std::function<void(void* userData)> GlobalHotplugDisconnectCallbackType;

    static std::vector<SDRDevice::CallbackInfo<GlobalHotplugConnectCallbackType>> GetConnectCallbacks();
    static std::vector<SDRDevice::CallbackInfo<GlobalHotplugDisconnectCallbackType>> GetDisconnectCallbacks();

    /// @brief Adds a callback to run when a global hotplug connect event happens (currently USB only).
    /// @param function The function to run when a connect happens.
    /// @param userData The data to pass into the function when it runs.
    /// @return The ID of the callback for removal later.
    static LIME_API std::size_t AddGlobalHotplugConnectCallback(const GlobalHotplugConnectCallbackType& function, void* userData);

    /// @brief Removes the given global hotplug connect callback by ID.
    /// @param id The ID of the callback to remove.
    static LIME_API void RemoveGlobalHotplugConnectCallback(std::size_t id);

    /// @brief Adds a callback to run when a global hotplug disconnect event happens (currently USB only).
    /// @param function The function to run when a disconnect happens.
    /// @param userData The data to pass into the function when it runs.
    /// @return The ID of the callback for removal later.
    static LIME_API std::size_t AddGlobalHotplugDisconnectCallback(
        const GlobalHotplugDisconnectCallbackType& function, void* userData);

    /// @brief Removes the given global hotplug disconnect callback by ID.
    /// @param id The ID of the callback to remove.
    static LIME_API void RemoveGlobalHotplugDisconnectCallback(std::size_t id);

  private:
    static std::set<IUSB::VendorProductId> vidPidPairsToLookFor;

    static std::vector<SDRDevice::CallbackInfo<GlobalHotplugConnectCallbackType>> globalHotplugConnectCallbacks;
    static std::vector<SDRDevice::CallbackInfo<GlobalHotplugDisconnectCallbackType>> globalHotplugDisconnectCallbacks;

#ifdef __unix__
    static USBGeneric hotplug;
#else
    static WindowsGlobalHotplug hotplug;
#endif

    template<typename T>
    static std::size_t AddCallback(std::vector<SDRDevice::CallbackInfo<T>>& vector, const T& function, void* userData);
    template<typename T> static void RemoveCallback(std::vector<SDRDevice::CallbackInfo<T>>& vector, std::size_t id);
};

} // namespace lime

#endif // LIME_GLOBALHOTPLUGEVENTS_H
