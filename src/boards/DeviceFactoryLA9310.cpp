#include "DeviceFactoryLA9310.h"

#include <fcntl.h>

#include "limesuiteng/DeviceHandle.h"
#include "CommonFunctions.h"
#include "limesuiteng/Logger.h"
#include "comms/PCIe/LimePCIe.h"
#include "comms/PCIe/LA9310_PCIe.h"
#include "protocols/LMSBoards.h"
#include "protocols/LMS64C/SPI.h"
#include "comms/PCIe/PCIE_CSR_Pipe.h"

#include "boards/LimeSDR_Micro/LimeSDR_Micro.h"

#include <algorithm>

using namespace lime;
using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

static std::vector<std::string> GetLA9310DeviceList()
{
    std::vector<std::string> devices;
    FILE* lsPipe;
    lsPipe = popen("ls -1 -- /sys/class/la9310_limesdr 2> /dev/null", "r");
    char tempBuffer[512];
    while (fscanf(lsPipe, "%s", tempBuffer) == 1)
    {
        // Kernel code fakes directories by replacing '/' char with '!'
        // open() can't open that
        // Replace '!' with '/' so we could open device
        std::string parsedDevicePath{ tempBuffer };
        for (auto& c : parsedDevicePath)
        {
            if (c == '!')
                c = '/';
        }
        devices.push_back(parsedDevicePath);
    }
    pclose(lsPipe);
    return devices;
}

void __loadDeviceFactoryLA9310(void) //TODO fixme replace with LoadLibrary/dlopen
{
    static DeviceFactoryLA9310 limeLA9310Support; // self register on initialization
}

DeviceFactoryLA9310::DeviceFactoryLA9310()
    : DeviceRegistryEntry("LimeLA9310"s)
{
}

std::vector<DeviceHandle> DeviceFactoryLA9310::enumerate(const DeviceHandle& hint)
{
    std::vector<DeviceHandle> handles;
    DeviceHandle handle;
    handle.media = "PCIe"s;

    if (!hint.media.empty() && hint.media != handle.media)
        return handles;

    // generate handles by probing devices
    std::vector<std::string> nodes = GetLA9310DeviceList();
    for (const std::string& nodeName : nodes)
    {
        // look for _control devices only, skip _trx*
        std::size_t nameEnd = nodeName.find("/control"sv);
        if (nameEnd == std::string::npos)
            continue;

        handle.addr = "/dev/"s + nodeName.substr(0, nameEnd); // removed _control postfix

        std::shared_ptr<LA9310_PCIe> pcidev = std::make_shared<LA9310_PCIe>();
        if (pcidev->Open(handle.addr + "/control0", O_RDWR) != OpStatus::Success)
            continue;

        // // use GET_INFO command to recognize the device
        // auto controlPipe = std::make_shared<PCIE_CSR_Pipe>(pcidev);
        // LMS64CProtocol::FirmwareInfo fw{};
        // int subDeviceIndex = 0;
        // LMS64CProtocol::GetFirmwareInfo(*controlPipe, fw, subDeviceIndex);

        handle.name = "LimeSDR-Micro"; // GetDeviceName(static_cast<eLMS_DEV>(fw.deviceId));
        handle.serial = "0"; //intToHex(fw.boardSerialNumber);

        // Add handle conditionally, filter by serial number
        if (handle.IsEqualIgnoringEmpty(hint))
            handles.push_back(handle);
    }
    return handles;
}

SDRDevice* DeviceFactoryLA9310::make(const DeviceHandle& handle)
{
    // Data transmission layer
    std::shared_ptr<LA9310_PCIe> controlPort = std::make_shared<LA9310_PCIe>();
    std::vector<std::shared_ptr<LA9310_PCIe>> streamPorts;

    std::string controlFile(handle.addr + "/control0");
    OpStatus connectionStatus = controlPort->Open(controlFile, O_RDWR);
    if (connectionStatus != OpStatus::Success)
    {
        lime::ReportError(connectionStatus, "Unable to connect to device using handle (%s)", handle.Serialize().c_str());
        return nullptr;
    }

    std::vector<std::string> streamEndpoints = LimePCIe::GetEndpointsWithPattern(handle.addr, "trx*"s);
    std::sort(
        streamEndpoints.begin(), streamEndpoints.end()); // TODO: Fix potential sorting problem if there would be trx1 and trx11
    for (const std::string& endpointPath : streamEndpoints)
    {
        streamPorts.push_back(std::make_shared<LA9310_PCIe>());
        streamPorts.back()->SetPathName(endpointPath);
    }

    auto controlPipe = std::make_shared<PCIE_CSR_Pipe>(controlPort);

    auto route_lms7002m =
        std::make_shared<LMS64C_SPI>(controlPipe, LMS64CProtocol::Command::LMS7002_WR, LMS64CProtocol::Command::LMS7002_RD, 0, 0);

    // auto route_i2c = std::make_shared<LMS64C_SPI>(
    //         controlPipe, LMS64CProtocol::Command::LMS7002_WR, LMS64CProtocol::Command::LMS7002_RD, 0, 0);

    return new LimeSDR_Micro(
        route_lms7002m, streamPorts.empty() ? nullptr : streamPorts.front(), controlPipe, controlPort, 30.72e6);
}
