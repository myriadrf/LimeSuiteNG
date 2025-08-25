#include "comms/shiva/shiva.h"

#include "limesuiteng/Logger.h"

#include <atomic>
#include <iostream>
#include <cerrno>
#include <cstring>
#include <thread>
#include "protocols/LMS64CProtocol.h"

#ifdef __unix__
    #include <unistd.h>
    #include <fcntl.h>
    #include <poll.h>
    #include <sys/mman.h>
    #include <sys/ioctl.h>
#endif

#include "la9310_modinfo.h"

using namespace std;
using namespace lime;
using namespace std::literals::string_literals;

std::vector<std::string> ShivaPCIE::GetPCIeDeviceList()
{
    std::vector<std::string> devices;
    devices = GetEndpointsWithPattern("/dev", "shiva0");
    return devices;
}

ShivaPCIE::ShivaPCIE()
    : LimePCIe()
{
}

ShivaPCIE::~ShivaPCIE()
{
    Close();
}

OpStatus ShivaPCIE::RunControlCommand(uint8_t* request, uint8_t* response, size_t length, int timeout_ms)
{
    uint8_t temp[64];
    volatile struct la9310_hif* hif;
    hif = static_cast<struct la9310_hif*>(hostInterfaceAddr);

    hif->sw_cmd_desc.cmd = 1;

    volatile uint32_t* arr32 = reinterpret_cast<uint32_t*>(request);
    const int words = length / sizeof(uint32_t);
    for (int i = 0; i < words; ++i)
    {
        hif->sw_cmd_desc.data[i] = arr32[i];
    }
    hif->sw_cmd_desc.status = LA9310_SW_CMD_STATUS_POSTED;
    do
    {
        // sleep is required otherwise received data is not always as expected
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } while (hif->sw_cmd_desc.status == LA9310_SW_CMD_STATUS_POSTED);

    volatile uint32_t* rarr32 = reinterpret_cast<uint32_t*>(temp);
    for (int i = 0; i < words; ++i)
    {
        rarr32[i] = hif->sw_cmd_desc.data[i];
    }
    memcpy(response, temp, length);

    if (hif->sw_cmd_desc.status == LA9310_SW_CMD_STATUS_DONE)
        return OpStatus::Success;
    else
        return OpStatus::Error;
    return OpStatus::NotImplemented;
}

OpStatus ShivaPCIE::RunControlCommand(uint8_t* data, size_t length, int timeout_ms)
{
    return RunControlCommand(data, data, length, timeout_ms);
}

OpStatus ShivaPCIE::Open(const std::filesystem::path& deviceFilename, uint32_t flags)
{
    modinfo_t info;

    mFilePath = deviceFilename;
    // use O_RDWR for now, because MMAP PROT_WRITE implies PROT_READ and will fail if file is opened write only
    flags &= ~O_WRONLY;
    flags |= O_RDWR;
    mFileDescriptor = open(mFilePath.c_str(), flags);
    if (mFileDescriptor < 0)
    {
        lime::error("ShivaPCIE: Failed to open (%s), errno(%i) %s", mFilePath.c_str(), errno, strerror(errno));
        // TODO: convert errno to OpStatus
        return OpStatus::FileNotFound;
    }

    int ret = ioctl(mFileDescriptor, IOCTL_LA93XX_MODINFO_GET, reinterpret_cast<modinfo_t*>(&info));
    if (ret < 0)
    {
        lime::error("IOCTL_LA93XX_IPC_GET_SYS_MAP failed.\n");
        close(mFileDescriptor);
        return OpStatus::Error;
    }

    memoryFileDescriptor = open("/dev/mem", O_RDWR);
    if (-1 == memoryFileDescriptor)
    {
        lime::error("/dev/mem open failed");
        return OpStatus::Error;
    }

    size_t hif_host_phys = info.hif.host_phy_addr;
    size_t hif_size = info.hif.size;
    hostInterfaceSize = hif_size;
    hostInterfaceAddr = mmap(NULL, hif_size, PROT_READ | PROT_WRITE, MAP_SHARED, memoryFileDescriptor, hif_host_phys);
    if (hostInterfaceAddr == MAP_FAILED)
    {
        lime::error("Mapping v_scratch_ddr_addr buffer failed\n");
        return OpStatus::Error;
    }

    volatile struct la9310_hif* hif;
    hif = static_cast<struct la9310_hif*>(hostInterfaceAddr);
    hif->host_ready |= LA9310_HIF_STATUS_IPC_APP_READY;

    return OpStatus::Success;
}

void ShivaPCIE::Close()
{
    if (mFileDescriptor >= 0)
        close(mFileDescriptor);
    mFileDescriptor = -1;

    if (hostInterfaceAddr)
        munmap(hostInterfaceAddr, hostInterfaceSize);

    if (memoryFileDescriptor >= 0)
        close(memoryFileDescriptor);
}

int ShivaPCIE::WriteControl(const uint8_t* buffer, const int length, int timeout_ms)
{
    return -1;
}

int ShivaPCIE::ReadControl(uint8_t* buffer, const int length, int timeout_ms)
{
    return -1;
}
