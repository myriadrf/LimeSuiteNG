#include "DeviceFactoryFX3.h"
#include "LimeSDR.h"
#include "FX3/FX3.h"
#include "USB_CSR_Pipe_SDR.h"
#include "LMS64C_LMS7002M_Over_USB.h"
#include "LMS64C_FPGA_Over_USB.h"
#include "limesuiteng/Logger.h"
#include "CommonFunctions.h"

#include <memory>
#include <string_view>
#include <stdexcept>

#ifndef __unix__
    #include "CyAPI.h"
    #include <codecvt>
#else
    #ifdef __GNUC__
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wpedantic"
    #endif
    #include <libusb.h>
    #ifdef __GNUC__
        #pragma GCC diagnostic pop
    #endif
    #include <mutex>
#endif

using namespace lime;
using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

void __loadFX3(void) //TODO fixme replace with LoadLibrary/dlopen
{
    static DeviceFactoryFX3 FX3Support; // self register on initialization
}

// Device identifier vendor ID and product ID pairs.
static const std::set<VidPid> ids{ { 1204, 241 }, { 1204, 243 }, { 7504, 24840 } };

DeviceFactoryFX3::DeviceFactoryFX3()
    : USBEntry("FX3"s, ids)
{
}

#ifndef __unix__
std::vector<DeviceHandle> DeviceFactoryFX3::enumerate(const DeviceHandle& hint)
{
    std::vector<DeviceHandle> handles;
    if (!hint.media.empty() && hint.media.find("USB"sv) == std::string::npos)
    {
        return handles;
    }

    CCyUSBDevice device;
    if (device.DeviceCount() <= 0)
    {
        return handles;
    }

    for (int i = 0; i < device.DeviceCount(); ++i)
    {
        if (device.IsOpen())
            device.Close();
        device.Open(i);
        DeviceHandle handle;
        if (device.bSuperSpeed == true)
            handle.media = "USB 3.0"s;
        else if (device.bHighSpeed == true)
            handle.media = "USB 2.0"s;
        else
            handle.media = "USB"s;

        uint16_t vendorId = device.VendorID;
        uint16_t productId = device.ProductID;
        handle.name = device.DeviceName;
        handle.serial = std::wstring_convert<std::codecvt_utf8<wchar_t>>().to_bytes(device.SerialNumber);
        handle.addr = intToHex(vendorId) + ':' + intToHex(productId);

        if (hint.serial.empty() or handle.serial.find(hint.serial) != std::string::npos)
            handles.push_back(handle); //filter on serial
        device.Close();
    }

    return handles;
}
#endif

SDRDevice* DeviceFactoryFX3::make_LimeSDR(const DeviceHandle& handle, const uint16_t& vid, const uint16_t& pid)
{
    auto usbComms = std::make_shared<FX3>(
#ifdef __unix__
        ctx
#endif
    );
    if (!usbComms->Connect(vid, pid, handle.serial))
    {
        const std::string reason = "Unable to connect to device using handle ("s + handle.Serialize() + ")"s;
        throw std::runtime_error(reason);
    }

    auto usbPipe = std::make_shared<USB_CSR_Pipe_SDR>(*usbComms);

    // protocol layer
    auto route_lms7002m = std::make_shared<LMS64C_LMS7002M_Over_USB>(usbPipe);
    auto route_fpga = std::make_shared<LMS64C_FPGA_Over_USB>(usbPipe);

    return new LimeSDR(route_lms7002m, route_fpga, usbComms, usbPipe);
}

SDRDevice* DeviceFactoryFX3::make(const DeviceHandle& handle)
{
    const auto splitPos = handle.addr.find(':');
    uint16_t vid = 0;
    uint16_t pid = 0;

    if (splitPos != std::string::npos)
    {
        vid = std::stoi(handle.addr.substr(0, splitPos), nullptr, 16);
        pid = std::stoi(handle.addr.substr(splitPos + 1), nullptr, 16);
    }

    if (ids.find({ vid, pid }) != ids.end())
        return make_LimeSDR(handle, vid, pid);

    lime::ReportError(OpStatus::InvalidValue, "Unrecognized device ID (%s)", handle.addr.c_str());
    return nullptr;
}
