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
#include "chips/LA9310/firmware/m4_commands.h"
#include "chips/LA9310/firmware/m4_events.h"

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

    OpStatus status;
    // status = ClearEvents((1 << M4_EVENT::HOST_COMMAND_DONE));
    // if (status != OpStatus::Success)
    // {
    //     lime::error("LA9310_PCIe: RunControlCommand failed clear IRQ\n");
    // }

    hif->sw_cmd_desc.cmd = LIME_M4_LMS64C_PACKET;

    volatile uint32_t* arr32 = reinterpret_cast<uint32_t*>(request);
    const int words = length / sizeof(uint32_t);
    for (int i = 0; i < words; ++i)
    {
        hif->sw_cmd_desc.data[i] = arr32[i];
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    hif->sw_cmd_desc.status = LA9310_SW_CMD_STATUS_POSTED;

    status = SendSignal(LA9310_SIGNALS::HOST_COMMAND_POSTED);
    if (status != OpStatus::Success)
    {
        lime::error("LA9310_PCIe: RunControlCommand failed to post\n");
        return status;
    }

    // status = WaitEvent(M4_EVENT::HOST_COMMAND_DONE, chrono::milliseconds(timeout_ms));
    // if (status != OpStatus::Success)
    // {
    //     lime::error("LA9310_PCIe: RunControlCommand IRQ timeout\n");
    //     // even if IRQ did not arrive, still check the command status
    //     // return status;
    // }
    // else
    // {
    //     status = ClearEvents((1 << M4_EVENT::HOST_COMMAND_DONE));
    //     if (status != OpStatus::Success)
    //     {
    //         lime::error("LA9310_PCIe: RunControlCommand failed clear IRQ\n");
    //         // return status;
    //     }
    // }

    while (hif->sw_cmd_desc.status == LA9310_SW_CMD_STATUS_POSTED || hif->sw_cmd_desc.status == LA9310_SW_CMD_STATUS_IN_PROGRESS)
    {
        auto t2 = chrono::high_resolution_clock::now();
        if (chrono::duration_cast<chrono::milliseconds>(t2 - t1) > chrono::milliseconds(timeout_ms))
        {
            lime::error("RunControlCommand timeout\n");
            return OpStatus::Timeout;
        }
    }

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

OpStatus LA9310_PCIe::AllocDMABuffer(size_t size, DMA_Buffer& buffer, LA9310_DMAPool pool)
{
    struct la9310_atu mem;
    memset(&mem, 0, sizeof(mem));
    mem.size = size;
    mem.flags = pool == LA9310_DMAPool::Uncached ? LA9310_DMA_FLAG_UNCACHED : 0;

    int ret = ioctl(mFileDescriptor, LA9310_IOCTL_DMA_ALLOC, &mem);
    if (ret < 0)
    {
        const int err = errno;
        lime::error("IOCTL DMA alloc of %zu bytes errno: %i (%s)", size, err, strerror(err));
        return err == ENOMEM ? OpStatus::NoMemory : OpStatus::Error;
    }

    void* vaddr = mmap(NULL, mem.size, PROT_READ | PROT_WRITE, MAP_SHARED, mFileDescriptor, mem.mmap_offset);
    if (vaddr == MAP_FAILED)
    {
        lime::error("DMA buffer mmap failed errno: %i (%s)", errno, strerror(errno));
        ioctl(mFileDescriptor, LA9310_IOCTL_DMA_FREE, &mem);
        return OpStatus::Error;
    }

    buffer = DMA_Buffer(vaddr, mem.host_bus, mem.ep_pa, mem.size, (mem.flags & LA9310_DMA_FLAG_COHERENT) != 0);
    return OpStatus::Success;
}

OpStatus LA9310_PCIe::FreeDMABuffer(const DMA_Buffer& buffer)
{
    if (buffer.size() == 0)
        return OpStatus::Success;

    munmap(buffer.va<void>(), buffer.size());

    struct la9310_atu mem;
    memset(&mem, 0, sizeof(mem));
    mem.host_bus = buffer.bus();

    int ret = ioctl(mFileDescriptor, LA9310_IOCTL_DMA_FREE, &mem);
    if (ret < 0)
    {
        lime::error("IOCTL DMA free errno: %i (%s)", errno, strerror(errno));
        return OpStatus::Error;
    }
    return OpStatus::Success;
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
        // The pools are accessed through the buffers from AllocDMABuffer()
        if (w == LA9310_WINDOW_MSI || w == LA9310_WINDOW_UNCACHED_POOL || w == LA9310_WINDOW_CACHED_POOL)
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

    // ClearEvents(0xFFFFFFFF);

    return OpStatus::Success;
}

void LA9310_PCIe::Close()
{
    for (mmaped_region& region : mapped_ranges)
    {
        if (region.vaddr && region.size)
        {
            // printf("Unmap %llx sz: %llu\n", region.vaddr, region.size);
            munmap(region.vaddr, region.size);
        }
        region = mmaped_region();
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

OpStatus LA9310_PCIe::iowrite32(uint32_t window_id, uint32_t value, uint64_t address)
{
    struct LA9310_IOCTL_CSR_op op;
    op.window_id = static_cast<la9310_window_t>(window_id);
    op.offset = address;
    op.value = value;
    op.write = 1;
    int ret = ioctl(mFileDescriptor, LA9310_IOCTL_CSR_OP, reinterpret_cast<LA9310_IOCTL_CSR_op*>(&op));
    if (ret < 0)
    {
        lime::error("LA9310_IOCTL_CSR_OP failed. errno: %i\n", errno);
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

OpStatus PCIe_CSR_Access::iowrite32(uint32_t value, size_t offset)
{
    return port->iowrite32(window_id, value, base_offset + offset);
}

uint32_t PCIe_CSR_Access::ioread32(size_t offset)
{
    return port->ioread32(window_id, base_offset + offset);
}

OpStatus LA9310_PCIe::SendSignal(uint32_t bitIndex)
{
    auto csr = GetCSRAccess(LA9310_WINDOW_BAR0);
    return csr->iowrite32(bitIndex, 0x1FC0000); // MSIIR
}

OpStatus LA9310_PCIe::EnableEvents(uint32_t enableBits, uint32_t mask)
{
    struct LA9310_IOCTL_EVENT_MASK event_enable;
    memset(&event_enable, 0, sizeof(event_enable));
    event_enable.enable_bits = enableBits;
    event_enable.enable_mask = mask;
    int ret = ioctl(mFileDescriptor, LA9310_IOCTL_EVENT_ENABLE, &event_enable);
    if (ret < 0)
    {
        // older drivers don't have the ioctl, they keep the events enabled
        if (errno == ENOTTY)
            return OpStatus::Success;
        lime::error("LA9310_PCIe EnableEvents(0x%04X, 0x%04X) errno: %i", enableBits, mask, errno);
        return OpStatus::Error;
    }
    return OpStatus::Success;
}

OpStatus LA9310_PCIe::ReinitEvents()
{
    int ret = ioctl(mFileDescriptor, LA9310_IOCTL_EVENT_REINIT);
    if (ret < 0)
    {
        // older drivers don't have the ioctl, their events don't depend on the firmware
        if (errno == ENOTTY)
            return OpStatus::Success;
        lime::error("LA9310_PCIe ReinitEvents errno: %i", errno);
        return OpStatus::Error;
    }
    return OpStatus::Success;
}

OpStatus LA9310_PCIe::WaitEvent(uint32_t event, std::chrono::milliseconds timeout)
{
    struct LA9310_IOCTL_EVENT event_wait;
    event_wait.event = event;
    event_wait.timeout_ms = timeout.count();
    int ret = ioctl(mFileDescriptor, LA9310_IOCTL_EVENT_WAIT, reinterpret_cast<LA9310_IOCTL_EVENT*>(&event_wait));
    if (ret < 0)
    {
        // lime::error("LA9310_PCIe WaitEvent errno: %i\n", errno);
        return OpStatus::Timeout;
    }
    return OpStatus::Success;
}

OpStatus LA9310_PCIe::ClearEvents(uint32_t bits)
{
    struct LA9310_IOCTL_EVENT_CTRL event_clear;
    memset(&event_clear, 0, sizeof(struct LA9310_IOCTL_EVENT_CTRL));
    event_clear.clear_bits = bits;
    event_clear.clear_mask = bits;
    event_clear.enable_mask = 0;
    int ret = ioctl(mFileDescriptor, LA9310_IOCTL_EVENT_CLEAR, reinterpret_cast<LA9310_IOCTL_EVENT_CTRL*>(&event_clear));
    if (ret < 0)
    {
        lime::error("LA9310_PCIe ClearEvents errno: %i\n", errno);
        return OpStatus::Error;
    }
    return OpStatus::Success;
}

OpStatus LA9310_PCIe::DMA_SyncForCPU(const DMA_Buffer& buffer)
{
    if (buffer.coherent())
        return OpStatus::Success;

    struct la9310_atu mem;
    mem.host_bus = buffer.bus();
    mem.size = buffer.size();

    int ret = ioctl(mFileDescriptor, LA9310_IOCTL_CACHE_SYNC_FOR_CPU, reinterpret_cast<la9310_atu*>(&mem));
    if (ret < 0)
    {
        printf("IOCTL SyncForCPU DMA errno: %i (%s)\n", errno, strerror(errno));
        return OpStatus::Error;
    }
    return OpStatus::Success;
}

OpStatus LA9310_PCIe::DMA_SyncForDevice(const DMA_Buffer& buffer)
{
    if (buffer.coherent())
        return OpStatus::Success;

    struct la9310_atu mem;
    mem.host_bus = buffer.bus();
    mem.size = buffer.size();

    int ret = ioctl(mFileDescriptor, LA9310_IOCTL_CACHE_SYNC_FOR_DEVICE, reinterpret_cast<la9310_atu*>(&mem));
    if (ret < 0)
    {
        printf("IOCTL SyncForDevice DMA errno: %i (%s)\n", errno, strerror(errno));
        return OpStatus::Error;
    }
    return OpStatus::Success;
}