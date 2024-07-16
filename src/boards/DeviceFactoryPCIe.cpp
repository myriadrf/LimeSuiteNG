#include "DeviceFactoryPCIe.h"

#include <fcntl.h>

#include "limesuiteng/DeviceHandle.h"
#include "CommonFunctions.h"
#include "limesuiteng/Logger.h"
#include "LitePCIe.h"
#include "LMSBoards.h"
#include "LMS64C_FPGA_Over_PCIe.h"
#include "LMS64C_LMS7002M_Over_PCIe.h"
#include "LMS64C_ADF_Over_PCIe_MMX8.h"
#include "LMS64C_FPGA_Over_PCIe_MMX8.h"
#include "LMS64C_LMS7002M_Over_PCIe_MMX8.h"

#include "boards/LimeSDR_XTRX/LimeSDR_XTRX.h"
#include "boards/LimeSDR_X3/LimeSDR_X3.h"
#include "boards/MMX8/MM_X8.h"
#include "boards/external/XSDR/XSDR.h"

#include <algorithm>

using namespace lime;

void __loadDeviceFactoryPCIe(void) //TODO fixme replace with LoadLibrary/dlopen
{
    static DeviceFactoryPCIe litePCIeSupport; // self register on initialization
}

DeviceFactoryPCIe::DeviceFactoryPCIe()
    : DeviceRegistryEntry("LitePCIe"s)
{
}

std::vector<DeviceHandle> DeviceFactoryPCIe::enumerate(const DeviceHandle& hint)
{
    std::vector<DeviceHandle> handles;
    DeviceHandle handle;
    handle.media = "PCIe"s;

    if (!hint.media.empty() && hint.media != handle.media)
        return handles;

    // generate handles by probing devices
    std::vector<std::string> nodes = LitePCIe::GetPCIeDeviceList();
    for (const std::string& nodeName : nodes)
    {
        // look for _control devices only, skip _trx*
        std::size_t nameEnd = nodeName.find("_control"sv);
        if (nameEnd == std::string::npos)
            continue;

        handle.name = nodeName.substr(0, nameEnd); // removed _control postfix
        handle.addr = "/dev/"s + nodeName;

        std::shared_ptr<LitePCIe> pcidev = std::make_shared<LitePCIe>();
        if (pcidev->Open(handle.addr, O_RDWR) != OpStatus::Success)
            continue;

        // use GET_INFO command to recognize the device
        auto controlPipe = std::make_shared<PCIE_CSR_Pipe>(pcidev);
        LMS64CProtocol::FirmwareInfo fw{};
        int subDeviceIndex = 0;
        LMS64CProtocol::GetFirmwareInfo(*controlPipe, fw, subDeviceIndex);

        handle.serial = intToHex(fw.boardSerialNumber);

        // Add handle conditionally, filter by serial number
        if (handle.IsEqualIgnoringEmpty(hint))
            handles.push_back(handle);
    }
    return handles;
}

SDRDevice* DeviceFactoryPCIe::make(const DeviceHandle& handle)
{
    // Data transmission layer
    std::shared_ptr<LitePCIe> controlPort = std::make_shared<LitePCIe>();
    std::vector<std::shared_ptr<LitePCIe>> streamPorts;

    std::string controlFile(handle.addr);
    OpStatus connectionStatus = controlPort->Open(controlFile, O_RDWR);
    if (connectionStatus != OpStatus::Success)
    {
        lime::ReportError(connectionStatus, "Unable to connect to device using handle (%s)", handle.Serialize().c_str());
        return nullptr;
    }

    std::vector<std::string> streamEndpoints = LitePCIe::GetDevicesWithPattern(handle.name + "_trx*"s);
    std::sort(streamEndpoints.begin(), streamEndpoints.end());
    for (const std::string& endpointPath : streamEndpoints)
    {
        streamPorts.push_back(std::make_shared<LitePCIe>());
        streamPorts.back()->SetPathName(endpointPath);
    }

    // protocol layer
    auto route_lms7002m = std::make_shared<LMS64C_LMS7002M_Over_PCIe>(controlPort);
    auto route_fpga = std::make_shared<LMS64C_FPGA_Over_PCIe>(controlPort);

    LMS64CProtocol::FirmwareInfo fw{};
    int subDeviceIndex = 0;
    auto controlPipe = std::make_shared<PCIE_CSR_Pipe>(controlPort);
    LMS64CProtocol::GetFirmwareInfo(*controlPipe, fw, subDeviceIndex);

    switch (fw.deviceId)
    {
    case LMS_DEV_LIMESDR_XTRX:
        return new LimeSDR_XTRX(route_lms7002m, route_fpga, streamPorts.front(), controlPipe);
    case LMS_DEV_LIMESDR_X3:
        return new LimeSDR_X3(route_lms7002m, route_fpga, std::move(streamPorts), controlPipe);
    case LMS_DEV_LIMESDR_MMX8: {
        auto adfComms = std::make_shared<LMS64C_ADF_Over_PCIe_MMX8>(controlPort, 0);
        std::vector<std::shared_ptr<IComms>> controls(8);
        std::vector<std::shared_ptr<IComms>> fpga(8);

        for (size_t i = 0; i < controls.size(); ++i)
        {
            controls[i] = std::make_shared<LMS64C_LMS7002M_Over_PCIe_MMX8>(controlPort, i + 1);
            fpga[i] = std::make_shared<LMS64C_FPGA_Over_PCIe_MMX8>(controlPort, i + 1);
        }
        fpga.push_back(std::make_shared<LMS64C_FPGA_Over_PCIe_MMX8>(controlPort, 0));

        return new LimeSDR_MMX8(controls, fpga, std::move(streamPorts), controlPipe, adfComms);
    }
    case LMS_DEV_EXTERNAL_XSDR:
        return new XSDR(route_lms7002m, route_fpga, streamPorts.front(), controlPipe);
    default:
        lime::ReportError(OpStatus::InvalidValue, "Unrecognized device ID (%i)", fw.deviceId);
        return nullptr;
    }
}
