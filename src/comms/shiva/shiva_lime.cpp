#include "comms/shiva/shiva_lime.h"

#include "limesuiteng/Logger.h"

#include <errno.h>
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

ShivaPCIE_lime::ShivaPCIE_lime()
    : LimePCIe()
{
}

ShivaPCIE_lime::~ShivaPCIE_lime()
{
    Close();
}

OpStatus ShivaPCIE_lime::RunControlCommand(uint8_t* request, uint8_t* response, size_t length, int timeout_ms)
{
    uint8_t temp[64];
    volatile struct la9310_hif* hif = static_cast<struct la9310_hif*>(hostInterfaceAddr);

    hif->sw_cmd_desc.cmd = 1;

    volatile uint32_t* arr32 = reinterpret_cast<uint32_t*>(request);
    const int words = length / sizeof(uint32_t);
    for (int i = 0; i < words; ++i)
    {
        hif->sw_cmd_desc.data[i] = arr32[i];
    }
    hif->sw_cmd_desc.status = LA9310_SW_CMD_STATUS_POSTED;
    auto t1 = std::chrono::high_resolution_clock::now();
    do
    {
        auto t2 = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1) > std::chrono::milliseconds(1000))
        {
            lime::error("ShivaPCIE_lime: RunControlCommand timeout\n");
        }
        // sleep is required otherwise received data is not always as expected
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } while (hif->sw_cmd_desc.status == LA9310_SW_CMD_STATUS_POSTED);

    // auto t2 = std::chrono::high_resolution_clock::now();
    // int duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    // printf("cmd time: %ius\n", duration);

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

OpStatus ShivaPCIE_lime::RunControlCommand(uint8_t* data, size_t length, int timeout_ms)
{
    return RunControlCommand(data, data, length, timeout_ms);
}

OpStatus ShivaPCIE_lime::Open(const std::filesystem::path& deviceFilename, uint32_t flags)
{
    mFilePath = deviceFilename;
    // use O_RDWR for now, because MMAP PROT_WRITE implies PROT_READ and will fail if file is opened write only
    flags &= ~O_WRONLY;
    flags |= O_RDWR;
    mFileDescriptor = open(mFilePath.c_str(), flags);
    if (mFileDescriptor < 0)
    {
        lime::error("ShivaPCIE_lime: Failed to open (%s), errno(%i) %s", mFilePath.c_str(), errno, strerror(errno));
        // TODO: convert errno to OpStatus
        return OpStatus::FileNotFound;
    }

    int ret = ioctl(mFileDescriptor, LA9310_IOCTL_GET_MEMORY_LAYOUT, reinterpret_cast<LA9310_IOCTL_memory_layout*>(&memoryLayout));
    if (ret < 0)
    {
        lime::error("LA9310_IOCTL_GET_MEMORY_LAYOUT failed. errno: %i\n", errno);
        close(mFileDescriptor);
        return OpStatus::Error;
    }

    const int page_size = sysconf(_SC_PAGESIZE);
    for (int w = LA9310_WINDOW_BAR0; w < LA9310_WINDOW_COUNT; ++w)
    {
        if (w == LA9310_WINDOW_MSI)
            continue;
        mapped_ranges[w].size = memoryLayout.window_size[w];
        const size_t offset =
            w * page_size; // Ubuntu 24.04 does not continue into the drivers mmap() if the offset is not page multiple
        // printf("map %i sz: %i\n", w, mapped_ranges[w].size);
        mapped_ranges[w].vaddr = mmap(NULL, mapped_ranges[w].size, PROT_READ | PROT_WRITE, MAP_SHARED, mFileDescriptor, offset);
        if (mapped_ranges[w].vaddr == MAP_FAILED)
        {
            lime::error("Mapping window[%i] buffer failed errno: %i\n", w, errno);
            return OpStatus::Error;
        }
        // printf("Mapped: %llx, sz:%li\n", mapped_ranges[w].vaddr, mapped_ranges[w].size);
    }

    hostInterfaceAddr = mapped_ranges[memoryLayout.host_interface.window_id].vaddr + memoryLayout.host_interface.start_offset;
    volatile struct la9310_hif* hif = static_cast<struct la9310_hif*>(hostInterfaceAddr);
    hif->host_ready |= LA9310_HIF_STATUS_IPC_APP_READY;
    return OpStatus::Success;
}

void ShivaPCIE_lime::Close()
{
    for (const mmaped_region& region : mapped_ranges)
    {
        if (region.vaddr && region.size)
        {
            // printf("Unmap %llx sz: %llu\n", region.vaddr, region.size);
            munmap(region.vaddr, region.size);
        }
    }

    if (mFileDescriptor >= 0)
        close(mFileDescriptor);
    mFileDescriptor = -1;
}

int ShivaPCIE_lime::WriteControl(const uint8_t* buffer, const int length, int timeout_ms)
{
    return -1;
}

int ShivaPCIE_lime::ReadControl(uint8_t* buffer, const int length, int timeout_ms)
{
    return -1;
}

mmaped_region ShivaPCIE_lime::GetBar(uint8_t i)
{
    return mapped_ranges[i];
}