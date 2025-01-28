#include "DeviceFactoryFTDI.h"

#include <string_view>

#include "LimeSDR_Mini/LimeSDR_Mini.h"
#include "LimeSDR_Mini/LimeNET_Micro.h"
#include "limesuiteng/DeviceHandle.h"
#include "LimeSDR_Mini/USB_CSR_Pipe_Mini.h"
#include "comms/USB/LMS64C_LMS7002M_Over_USB.h"
#include "comms/USB/LMS64C_FPGA_Over_USB.h"
#include "CommonFunctions.h"
#include "comms/USB/FT601/FT601.h"

using namespace lime;
using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

void __loadFTDI(void) // TODO: fixme replace with LoadLibrary/dlopen
{
    static DeviceFactoryFTDI FTDISupport; // Self register on initialization
}

// Device identifier vendor ID and product ID pairs.
static const std::set<IUSB::VendorProductId> ids{ { 0x0403, 0x601F } };

DeviceFactoryFTDI::DeviceFactoryFTDI()
    : DeviceRegistryEntry("FTDI"s)
{
}

std::vector<DeviceHandle> DeviceFactoryFTDI::enumerate(const DeviceHandle& hint)
{
    std::vector<DeviceHandle> handles;
    if (!hint.media.empty() && hint.media.find("USB"sv) == std::string::npos)
        return handles;

    FT601 usb;
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

static SDRDevice* make_LimeSDR_Mini(const DeviceHandle& handle, uint16_t vid, uint16_t pid)
{
    auto usbComms = std::make_shared<FT601>();
    if (!usbComms->Connect(vid, pid, handle.serial.c_str()))
    {
        const std::string reason = "Unable to connect to device using handle ("s + handle.Serialize() + ")"s;
        throw std::runtime_error(reason);
    }

    auto usbPipe = std::make_shared<USB_CSR_Pipe_Mini>(*usbComms);

    // protocol layer
    auto route_lms7002m = std::make_shared<LMS64C_LMS7002M_Over_USB>(usbPipe);
    auto route_fpga = std::make_shared<LMS64C_FPGA_Over_USB>(usbPipe);

    auto board = new LimeSDR_Mini(route_lms7002m, route_fpga, usbComms, usbPipe);
    // LimeSDR-Mini serial number is taken from USB chip's descriptor.
    // TODO: add serial number getter into UnixUsb, and use it inside LimeSDR_mini
    board->SetSerialNumber(handle.serial);
    return board;
}

static SDRDevice* make_LimeNET_Micro(const DeviceHandle& handle, uint16_t vid, uint16_t pid)
{
    auto usbComms = std::make_shared<FT601>();
    if (!usbComms->Connect(vid, pid, handle.serial.c_str()))
    {
        const std::string reason = "Unable to connect to device using handle ("s + handle.Serialize() + ")"s;
        throw std::runtime_error(reason);
    }

    auto usbPipe = std::make_shared<USB_CSR_Pipe_Mini>(*usbComms);

    // protocol layer
    auto route_lms7002m = std::make_shared<LMS64C_LMS7002M_Over_USB>(usbPipe);
    auto route_fpga = std::make_shared<LMS64C_FPGA_Over_USB>(usbPipe);

    auto board = new LimeNET_Micro(route_lms7002m, route_fpga, usbComms, usbPipe);
    // LimeNET Micro serial number is taken from USB chip's descriptor.
    // TODO: add serial number getter into UnixUsb, and use it inside LimeSDR_mini
    board->SetSerialNumber(handle.serial);
    return board;
}

SDRDevice* DeviceFactoryFTDI::make(const DeviceHandle& handle)
{
    const auto splitPos = handle.addr.find(':');

    uint16_t vid = 0;
    uint16_t pid = 0;

    if (splitPos != std::string::npos)
    {
        vid = std::stoi(handle.addr.substr(0, splitPos), nullptr, 16);
        pid = std::stoi(handle.addr.substr(splitPos + 1), nullptr, 16);
    }

    if (ids.find({ vid, pid }) == ids.end())
        return nullptr;

    // read info what kind of board it is
    LMS64CProtocol::FirmwareInfo fw{};
    {
        auto usbComms = std::make_shared<FT601>();
        if (!usbComms->Connect(vid, pid, handle.serial.c_str()))
            return nullptr;
        auto usbPipe = std::make_shared<USB_CSR_Pipe_Mini>(*usbComms);
        LMS64CProtocol::GetFirmwareInfo(*usbPipe.get(), fw);
    }

    switch (fw.deviceId)
    {
    case LMS_DEV_LIMENET_MICRO:
        return make_LimeNET_Micro(handle, vid, pid);
    case LMS_DEV_LIMESDRMINI:
    case LMS_DEV_LIMESDRMINI_V2:
    default:
        return make_LimeSDR_Mini(handle, vid, pid);
    }
    return nullptr;
}
