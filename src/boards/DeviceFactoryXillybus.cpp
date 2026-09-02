#include "DeviceFactoryXillybus.h"

#include <fcntl.h>

#include "limesuiteng/DeviceHandle.h"
#include "CommonFunctions.h"
#include "limesuiteng/Logger.h"
#include "comms/PCIe/xillybus/Xillybus.h"
#include "protocols/LMSBoards.h"
#include "protocols/LMS64C/SPI.h"

#include "boards/LimeSDR_PCIE/LimeSDR_PCIE.h"

#include <algorithm>

using namespace lime;
using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

#ifndef __unix__
    #define DEVPATH_PREFIX "\\\\.\\" // Windows prefix
#else
    #define DEVPATH_PREFIX "/dev/"
#endif

static const std::vector<std::string> device_endpoint_names = {
    "xillybus_read_8", "xillybus_write_8", "xillybus_read_32", "xillybus_write_32"
};

static std::vector<std::string> GetXillyBusEndPointList()
{
    std::vector<std::string> devices;
    FILE* lsPipe;
    lsPipe = popen("ls -1 -- /sys/class/xillybus 2> /dev/null", "r");
    if (lsPipe == nullptr)
        return devices;
    char tempBuffer[512];
    while (fscanf(lsPipe, "%511s", tempBuffer) == 1)
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

void __loadDeviceFactoryXillybus(void)
{
    static DeviceFactoryXillybus limeXillybusSupport; // self register on initialization
}

DeviceFactoryXillybus::DeviceFactoryXillybus()
    : DeviceRegistryEntry("Xillybus"s)
{
}

std::vector<DeviceHandle> DeviceFactoryXillybus::enumerate(const DeviceHandle& hint)
{
    std::vector<DeviceHandle> handles;
    DeviceHandle handle;
    handle.media = "PCIe"s;

    if (!hint.media.empty() && hint.media != handle.media)
        return handles;

    // generate handles by probing devices
    std::vector<std::string> nodes = GetXillyBusEndPointList();

    bool hasCtrlRead = find(nodes.begin(), nodes.end(), device_endpoint_names[0]) != nodes.end();
    bool hasCtrlWrite = find(nodes.begin(), nodes.end(), device_endpoint_names[1]) != nodes.end();
    bool hasStreamRead = find(nodes.begin(), nodes.end(), device_endpoint_names[2]) != nodes.end();
    bool hasStreamWrite = find(nodes.begin(), nodes.end(), device_endpoint_names[3]) != nodes.end();

    if (!hasCtrlRead || !hasCtrlWrite || !hasStreamRead || !hasStreamWrite)
        return handles;

    handle.addr = "/dev/xillybus"s;

    std::shared_ptr<Xillybus> pcidev = std::make_shared<Xillybus>();
    if (pcidev->Open(DEVPATH_PREFIX + device_endpoint_names[1], DEVPATH_PREFIX + device_endpoint_names[0], true) !=
        OpStatus::Success)
        return handles;

    // use GET_INFO command to recognize the device
    LMS64CProtocol::FirmwareInfo fw{};
    int subDeviceIndex = 0;
    LMS64CProtocol::GetFirmwareInfo(*pcidev, fw, subDeviceIndex);

    handle.name = GetDeviceName(static_cast<eLMS_DEV>(fw.deviceId));
    handle.serial = intToHex(fw.boardSerialNumber);

    // Add handle conditionally, filter by serial number
    if (handle.IsEqualIgnoringEmpty(hint))
        handles.push_back(handle);

    return handles;
}

SDRDevice* DeviceFactoryXillybus::make(const DeviceHandle& handle)
{
    std::vector<std::shared_ptr<Xillybus>> streamPorts;

    // Data transmission layer
    std::shared_ptr<Xillybus> controlPort = std::make_shared<Xillybus>();
    OpStatus connectionStatus =
        controlPort->Open(DEVPATH_PREFIX + device_endpoint_names[1], DEVPATH_PREFIX + device_endpoint_names[0], true);
    if (connectionStatus != OpStatus::Success)
    {
        lime::ReportError(connectionStatus, "Unable to connect to device using handle (%s)", handle.Serialize().c_str());
        return nullptr;
    }

    std::shared_ptr<Xillybus> streamPort = std::make_shared<Xillybus>();
    connectionStatus =
        streamPort->Open(DEVPATH_PREFIX + device_endpoint_names[3], DEVPATH_PREFIX + device_endpoint_names[2], false);
    if (connectionStatus != OpStatus::Success)
    {
        lime::ReportError(connectionStatus, "Unable to connect to device using handle (%s)", handle.Serialize().c_str());
        return nullptr;
    }

    streamPorts.push_back(streamPort);

    LMS64CProtocol::FirmwareInfo fw{};
    LMS64CProtocol::GetFirmwareInfo(*controlPort, fw, 0);

    switch (fw.deviceId)
    {
    case LMS_DEV_LIMESDR_PCIE: {
        auto route_lms7002m = std::make_shared<LMS64C_SPI>(
            controlPort, LMS64CProtocol::Command::LMS7002_WR, LMS64CProtocol::Command::LMS7002_RD, 0, 0);
        auto route_fpga =
            std::make_shared<LMS64C_SPI>(controlPort, LMS64CProtocol::Command::BRDSPI_WR, LMS64CProtocol::Command::BRDSPI_RD, 0, 0);
        return new LimeSDR_PCIE(route_lms7002m, route_fpga, streamPorts.empty() ? nullptr : streamPorts.front(), controlPort);
    }
    default:
        lime::ReportError(OpStatus::InvalidValue, "Unrecognized device ID (%i)", fw.deviceId);
        return nullptr;
    }
}
