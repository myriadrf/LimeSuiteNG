#include "DeviceFactoryPCIe_shiva.h"

#include <fcntl.h>

#include "limesuiteng/DeviceHandle.h"
#include "CommonFunctions.h"
#include "limesuiteng/Logger.h"
#include "comms/shiva/shiva.h"
#include "comms/PCIe/LimePCIe.h"
#include "comms/PCIe/PCIE_CSR_Pipe.h"
#include "protocols/LMSBoards.h"
#include "protocols/LMS64CProtocol.h"
#include "protocols/LMS64C/SPI.h"

#include "boards/LimeSDR_Micro/LimeSDR_Micro.h"

#include <algorithm>

using namespace lime;
using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

void __loadDeviceFactoryPCIe_shiva(void)
{
    static DeviceFactoryPCIe_shiva limeShivaSupport; // self register on initialization
}

DeviceFactoryPCIe_shiva::DeviceFactoryPCIe_shiva()
    : DeviceRegistryEntry("la9310shiva"s)
{
}

std::vector<DeviceHandle> DeviceFactoryPCIe_shiva::enumerate(const DeviceHandle& hint)
{
    std::vector<DeviceHandle> handles;
    DeviceHandle handle;
    handle.media = "la9310shiva"s;

    if (!hint.media.empty() && hint.media != handle.media)
        return handles;

    // generate handles by probing devices
    std::vector<std::string> nodes = ShivaPCIE::GetPCIeDeviceList();
    for (const std::string& nodeName : nodes)
    {
        handle.addr = nodeName;

        std::shared_ptr<ShivaPCIE> pcidev = std::make_shared<ShivaPCIE>();
        if (pcidev->Open(handle.addr, O_RDWR) != OpStatus::Success)
            continue;

        // use GET_INFO command to recognize the device
        auto controlPipe = std::make_shared<PCIE_CSR_Pipe>(pcidev);
        LMS64CProtocol::FirmwareInfo fw{};
        int subDeviceIndex = 0;
        LMS64CProtocol::GetFirmwareInfo(*controlPipe, fw, subDeviceIndex);

        handle.name = GetDeviceName(static_cast<eLMS_DEV>(fw.deviceId));
        handle.serial = intToHex(fw.boardSerialNumber);

        // Add handle conditionally, filter by serial number
        if (handle.IsEqualIgnoringEmpty(hint))
            handles.push_back(handle);
    }
    return handles;
}

SDRDevice* DeviceFactoryPCIe_shiva::make(const DeviceHandle& handle)
{
    // Data transmission layer
    std::shared_ptr<ShivaPCIE> controlPort = std::make_shared<ShivaPCIE>();

    std::string controlFile(handle.addr);
    OpStatus connectionStatus = controlPort->Open(controlFile, O_RDWR);
    if (connectionStatus != OpStatus::Success)
    {
        lime::ReportError(connectionStatus, "Unable to connect to device using handle (%s)", handle.Serialize().c_str());
        return nullptr;
    }

    LMS64CProtocol::FirmwareInfo fw{};
    int subDeviceIndex = 0;
    auto controlPipe = std::make_shared<PCIE_CSR_Pipe>(controlPort);
    LMS64CProtocol::GetFirmwareInfo(*controlPipe, fw, subDeviceIndex);

    // auto route_I2C = std::make_shared<I2C_Over_Shiva>(controlPort);

    switch (fw.deviceId)
    {
    // case LMS_DEV_LIMESDR_MICRO: {
    //     auto route_lms7002m = std::make_shared<LMS64C_SPI>(
    //         controlPipe, LMS64CProtocol::Command::LMS7002_WR, LMS64CProtocol::Command::LMS7002_RD, 0, 0);
    //     return new LimeSDR_Micro(route_lms7002m, nullptr, controlPipe, controlPort, 30.72e6);
    // }
    default:
        lime::ReportError(OpStatus::InvalidValue, "Unrecognized device ID (%i)", fw.deviceId);
        return nullptr;
    }
}
