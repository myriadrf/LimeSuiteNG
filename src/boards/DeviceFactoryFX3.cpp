#include "DeviceFactoryFX3.h"

#include <memory>
#include <string_view>

#include "limesuiteng/DeviceHandle.h"
#include "LimeSDR/LimeSDR.h"
#include "FX3/FX3.h"
#include "LimeSDR/USB_CSR_Pipe_SDR.h"
#include "comms/USB/LMS64C_LMS7002M_Over_USB.h"
#include "comms/USB/LMS64C_FPGA_Over_USB.h"
#include "CommonFunctions.h"

using namespace lime;
using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

void __loadFX3(void) //TODO fixme replace with LoadLibrary/dlopen
{
    static DeviceFactoryFX3 FX3Support; // self register on initialization
}

// Device identifier vendor ID and product ID pairs.
static const std::set<IUSB::VendorProductId> ids{ { 0x04B4, 0x00F1 }, { 0x04B4, 0x00F3 }, { 0x1D50, 0x6108 } };

DeviceFactoryFX3::DeviceFactoryFX3()
    : DeviceRegistryEntry("FX3"s)
{
}

std::vector<DeviceHandle> DeviceFactoryFX3::enumerate(const DeviceHandle& hint)
{
    std::vector<DeviceHandle> handles;
    if (!hint.media.empty() && hint.media.find("USB"sv) == std::string::npos)
        return handles;

    FX3 usb;
    std::vector<USBDescriptor> deviceList = usb.enumerateDevices(ids);

    if (deviceList.empty())
        return handles;

    for (auto& dev : deviceList)
    {
        if (ids.find({ dev.vid, dev.pid }) == ids.end())
            continue;
        DeviceHandle handle;
        handle.name = dev.product;
        handle.serial = dev.serial;
        handle.addr = intToHex(dev.vid) + ':' + intToHex(dev.pid);

        if (handle.IsEqualIgnoringEmpty(hint))
            handles.push_back(handle);
    }

    return handles;
}

SDRDevice* DeviceFactoryFX3::make_LimeSDR(const DeviceHandle& handle, uint16_t vid, uint16_t pid)
{
    auto usbComms = std::make_shared<FX3>();
    if (!usbComms->Connect(vid, pid, handle.serial.c_str()))
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

    return nullptr;
}
