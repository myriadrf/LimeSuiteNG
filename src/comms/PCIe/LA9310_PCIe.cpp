#include "LA9310_PCIe.h"

#include "limesuiteng/Logger.h"

#include <assert.h>
#include <errno.h>
#include <atomic>
#include <iostream>
#include <cerrno>
#include <cstring>
#include <thread>
#include "protocols/LMS64CProtocol.h"
#include "boards/LimeSDR_Micro/m4_commands.h"

#include "drivers/linux/la9310_limesdr/common_headers/la9310_host_if.h"

#ifdef __unix__
    #include <unistd.h>
    #include <fcntl.h>
    #include <poll.h>
    #include <sys/mman.h>
    #include <sys/ioctl.h>
#endif

#if 0 // print debug messages
    #define printf_dbg_log(...) \
        do \
        { \
            printf(__VA_ARGS__); \
        } while (0)
#else
    #define printf_dbg_log(format, ...)
#endif

using namespace std;
using namespace lime;
using namespace std::literals::string_literals;

LA9310_PCIe::LA9310_PCIe()
    : LimePCIe()
    , hostInterface(nullptr)
{
}

LA9310_PCIe::~LA9310_PCIe()
{
    Close();
}

OpStatus LA9310_PCIe::RunControlCommand(uint8_t* request, uint8_t* response, size_t length, int timeout_ms)
{
    uint8_t temp[64];
    volatile struct la9310_hif* hif = hostInterface;
    assert(hif);

    hif->sw_cmd_desc.cmd = LIME_M4_LMS64C_PACKET;

    volatile uint32_t* arr32 = reinterpret_cast<uint32_t*>(request);
    const int words = length / sizeof(uint32_t);
    for (int i = 0; i < words; ++i)
    {
        hif->sw_cmd_desc.data[i] = arr32[i];
    }
    hif->sw_cmd_desc.status = LA9310_SW_CMD_STATUS_POSTED;
    auto t1 = std::chrono::high_resolution_clock::now();
    while (hif->sw_cmd_desc.status == LA9310_SW_CMD_STATUS_POSTED || hif->sw_cmd_desc.status == LA9310_SW_CMD_STATUS_IN_PROGRESS)
    {
        auto t2 = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1) > std::chrono::milliseconds(1000))
        {
            lime::error("LA9310_PCIe: RunControlCommand timeout\n");
            break;
        }
    }

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

OpStatus LA9310_PCIe::RunControlCommand(uint8_t* data, size_t length, int timeout_ms)
{
    return RunControlCommand(data, data, length, timeout_ms);
}

OpStatus LA9310_PCIe::Open(const std::filesystem::path& deviceFilename, uint32_t flags)
{
    mFilePath = deviceFilename;
    // use O_RDWR for now, because MMAP PROT_WRITE implies PROT_READ and will fail if file is opened write only
    flags &= ~O_WRONLY;
    flags |= O_RDWR;
    mFileDescriptor = open(mFilePath.c_str(), flags);
    if (mFileDescriptor < 0)
    {
        lime::error("LA9310_PCIe: Failed to open (%s), errno(%i) %s", mFilePath.c_str(), errno, strerror(errno));
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
        if (w == LA9310_WINDOW_MSI || w == LA9310_WINDOW_IPC)
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

    hostInterface = reinterpret_cast<volatile struct la9310_hif*>(
        size_t(mapped_ranges[memoryLayout.host_interface.window_id].vaddr) + memoryLayout.host_interface.start_offset);
    return OpStatus::Success;
}

void LA9310_PCIe::Close()
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

int LA9310_PCIe::WriteControl(const uint8_t* buffer, const int length, int timeout_ms)
{
    return -1;
}

int LA9310_PCIe::ReadControl(uint8_t* buffer, const int length, int timeout_ms)
{
    return -1;
}

mmaped_region LA9310_PCIe::GetBar(uint8_t i)
{
    return mapped_ranges[i];
}

OpStatus LA9310_PCIe::wait_for_new_data(int timeout_ms)
{
    while (1)
    {
        if (!ioctl(mFileDescriptor, LA9310_IOCTL_WAIT_FOR_DATA, &timeout_ms))
            return OpStatus::Success;

        // EINTR may be returned if a signal occured whilst in the ioctl
        if (errno == EINTR)
            continue;
        if (errno == ETIMEDOUT)
            return OpStatus::Timeout;

        return OpStatus::Error;
    }
}

void LA9310_PCIe::dmem_sync_to_cpu(volatile const void* addr, uint32_t data_size)
{
    struct LA9310_IOCTL_flush_cache cache_entry;
    auto vl_dmem_proxy_addr = mapped_ranges[LA9310_WINDOW_IQFLOOD].vaddr;

    // Fill struct with information
    cache_entry.sync_to_cpu = 1;
    cache_entry.size = data_size;
    cache_entry.offset = static_cast<uint32_t>(uint64_t(addr) - uint64_t(vl_dmem_proxy_addr));

    printf_dbg_log("sync_dmem_proxy_before_read called for offset: 0x%08x\n", cache_entry.offset);

    // Fire ioctl
    ioctl(mFileDescriptor, LA9310_IOCTL_FLUSH_CACHE_VSPA_DMEM, &cache_entry);
}

void LA9310_PCIe::dmem_sync_to_device(volatile const void* addr, uint32_t data_size)
{
    struct LA9310_IOCTL_flush_cache cache_entry;
    auto vl_dmem_proxy_addr = mapped_ranges[LA9310_WINDOW_IQFLOOD].vaddr;

    // Fill struct with information
    cache_entry.sync_to_cpu = 0;
    cache_entry.size = data_size;
    cache_entry.offset = static_cast<uint32_t>(uint64_t(addr) - uint64_t(vl_dmem_proxy_addr));

    printf_dbg_log("sync_dmem_proxy_after_write called for offset: 0x%08x\n", cache_entry.offset);

    // Fire ioctl
    ioctl(mFileDescriptor, LA9310_IOCTL_FLUSH_CACHE_VSPA_DMEM, &cache_entry);
}

OpStatus LA9310_PCIe::iowrite32(uint32_t window_id, uint32_t value, uint64_t address)
{
    // printf("iowr: %08X v: %08X\n", address, value);
    struct LA9310_IOCTL_CSR_op op;
    op.window_id = static_cast<la9310_window_t>(window_id);
    op.offset = address;
    op.value = value;
    op.write = 1;
    int ret = ioctl(mFileDescriptor, LA9310_IOCTL_CSR_OP, reinterpret_cast<LA9310_IOCTL_CSR_op*>(&op));
    if (ret < 0)
    {
        lime::error("LA9310_IOCTL_CSR_OP failed. errno: %i\n", errno);
        close(mFileDescriptor);
        return OpStatus::Error;
    }
    return OpStatus::Success;
}

uint32_t LA9310_PCIe::ioread32(uint32_t window_id, uint64_t address)
{
    struct LA9310_IOCTL_CSR_op op;
    op.window_id = static_cast<la9310_window_t>(window_id);
    op.offset = address;
    op.value = 0;
    op.write = 0;
    int ret = ioctl(mFileDescriptor, LA9310_IOCTL_CSR_OP, reinterpret_cast<LA9310_IOCTL_CSR_op*>(&op));
    if (ret < 0)
    {
        lime::error("LA9310_IOCTL_CSR_OP failed. errno: %i\n", errno);
        return 0xFFFFFFFF;
    }
    return op.value;
}

PCIe_CSR_Access::PCIe_CSR_Access(LA9310_PCIe* port, uint32_t window_id, size_t base_offset)
    : port(port)
    , window_id(window_id)
    , base_offset(base_offset)
{
}

void PCIe_CSR_Access::iowrite32(uint32_t value, size_t offset)
{
    port->iowrite32(window_id, value, base_offset + offset);
}

uint32_t PCIe_CSR_Access::ioread32(size_t offset)
{
    return port->ioread32(window_id, base_offset + offset);
}

OpStatus LA9310_PCIe::LoadArmM4Firmware(const char* data, size_t length)
{
    const struct LA9310_IOCTL_firmware fw = { data, length };
    return ioctl(mFileDescriptor, LA9310_IOCTL_LOAD_M4_FW, &fw) ? OpStatus::Error : OpStatus::Success;
}

OpStatus LA9310_PCIe::LoadVSPAFirmware(const char* data, size_t length)
{
    // Check RCW Completion bit, without it accessing VSPA registers will fail and reboot host
    constexpr uint32_t VSPA_CCSR_offset = 0x1000000;
    constexpr uint32_t vcpu_busy = (1 << 8);

    const auto VSPA_STATUS_reg_offset = VSPA_CCSR_offset + 0x10;
    auto csr = GetCSRAccess(LA9310_WINDOW_BAR0);
    bool busy = csr->ioread32(VSPA_STATUS_reg_offset) & vcpu_busy;
    if (busy)
        return OpStatus::Busy;

    const struct LA9310_IOCTL_firmware fw = { data, length };
    return ioctl(mFileDescriptor, LA9310_IOCTL_LOAD_VSPA_FW, &fw) ? OpStatus::Error : OpStatus::Success;
}
