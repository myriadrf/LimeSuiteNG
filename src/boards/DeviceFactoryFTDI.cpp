#include "DeviceFactoryFTDI.h"
#include "LimeSDR_Mini.h"
#include "DeviceExceptions.h"
#include "limesuiteng/DeviceRegistry.h"
#include "limesuiteng/DeviceHandle.h"
#include "protocols/LMS64CProtocol.h"
#include "limesuiteng/Logger.h"
#include "USB_CSR_Pipe_Mini.h"
#include "LMS64C_LMS7002M_Over_USB.h"
#include "LMS64C_FPGA_Over_USB.h"
#include "CommonFunctions.h"
#include "FT601/FT601.h"

#ifndef __unix__
    #include "windows.h"
    #include "FTD3XXLibrary/FTD3XX.h"
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
#include <string_view>

using namespace lime;
using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

void __loadFTDI(void) // TODO: fixme replace with LoadLibrary/dlopen
{
    static DeviceFactoryFTDI FTDISupport; // Self register on initialization
}

// Device identifier vendor ID and product ID pairs.
static const std::set<VidPid> ids{ { 1027, 24607 } };

DeviceFactoryFTDI::DeviceFactoryFTDI()
    : USBEntry("FTDI"s, ids)
{
}

#ifndef __unix__
std::vector<DeviceHandle> DeviceFactoryFTDI::enumerate(const DeviceHandle& hint)
{
    std::vector<DeviceHandle> handles;

    if (!hint.media.empty() && hint.media.find("USB"sv) == std::string::npos)
    {
        return handles;
    }

    FT_STATUS ftStatus = FT_OK;
    static DWORD numDevs = 0;

    ftStatus = FT_CreateDeviceInfoList(&numDevs);

    if (!FT_FAILED(ftStatus) && numDevs > 0)
    {
        DWORD Flags = 0;
        char SerialNumber[16] = { 0 };
        char Description[32] = { 0 };
        for (DWORD i = 0; i < numDevs; i++)
        {
            DWORD deviceId = 0;
            ftStatus = FT_GetDeviceInfoDetail(i, &Flags, nullptr, &deviceId, nullptr, SerialNumber, Description, nullptr);
            if (!FT_FAILED(ftStatus))
            {
                WORD vendorId = (deviceId >> 16);
                WORD productId = static_cast<WORD>(deviceId);
                DeviceHandle handle;
                handle.media = Flags & FT_FLAGS_SUPERSPEED ? "USB 3"s : Flags & FT_FLAGS_HISPEED ? "USB 2"s : "USB"s;
                handle.name = Description;
                handle.serial = SerialNumber;
                handle.addr = intToHex(vendorId) + ':' + intToHex(productId);
                // Add handle conditionally, filter by serial number
                if (hint.serial.empty() || handle.serial.find(hint.serial) != std::string::npos)
                    handles.push_back(handle);
            }
        }
    }
    return handles;
}
#endif

SDRDevice* DeviceFactoryFTDI::make_LimeSDR_Mini(const DeviceHandle& handle, const uint16_t& vid, const uint16_t& pid)
{
    auto usbComms = std::make_shared<FT601>(
#ifdef __unix__
        ctx
#endif
    );
    if (!usbComms->Connect(vid, pid, handle.serial))
    {
        const std::string reason = "Unable to connect to device using handle ("s + handle.Serialize() + ")"s;
        throw std::runtime_error(reason);
    }

    auto usbPipe = std::make_shared<USB_CSR_Pipe_Mini>(*usbComms);

    // protocol layer
    auto route_lms7002m = std::make_shared<LMS64C_LMS7002M_Over_USB>(usbPipe);
    auto route_fpga = std::make_shared<LMS64C_FPGA_Over_USB>(usbPipe);

    return new LimeSDR_Mini(route_lms7002m, route_fpga, usbComms, usbPipe);
}

SDRDevice* DeviceFactoryFTDI::make(const DeviceHandle& handle)
{
    const auto splitPos = handle.addr.find(":");

    uint16_t vid = 0;
    uint16_t pid = 0;

    if (splitPos != std::string::npos)
    {
        vid = std::stoi(handle.addr.substr(0, splitPos), nullptr, 16);
        pid = std::stoi(handle.addr.substr(splitPos + 1), nullptr, 16);
    }

    if (ids.find({ vid, pid }) != ids.end())
        return make_LimeSDR_Mini(handle, vid, pid);

    lime::ReportError(OpStatus::InvalidValue, "Unrecognized device ID (%s)", handle.addr.c_str());
    return nullptr;
}
