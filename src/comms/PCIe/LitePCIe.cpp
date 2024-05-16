#include "LitePCIe.h"

#include <iostream>
#include <errno.h>
#include <string.h>
#include <thread>
#include <filesystem>
#include "limesuiteng/Logger.h"

#ifdef __unix__
    #include <unistd.h>
    #include <fcntl.h>
    #include <poll.h>
    #include <sys/mman.h>
    #include <sys/ioctl.h>
    #include "linux-kernel-module/litepcie.h"
#endif

using namespace std;
using namespace lime;
using namespace std::literals::string_literals;

#define EXTRA_CHECKS 1

std::vector<std::string> LitePCIe::GetDevicesWithPattern(const std::string& regex)
{
    std::vector<std::string> devices;
    FILE* lsPipe;
    std::string cmd = "find /dev -maxdepth 1 -readable -name "s + regex;
    lsPipe = popen(cmd.c_str(), "r");
    char tempBuffer[512];
    while (fscanf(lsPipe, "%s", tempBuffer) == 1)
        devices.push_back(tempBuffer);
    pclose(lsPipe);
    return devices;
}

std::vector<std::string> LitePCIe::GetPCIeDeviceList()
{
    std::vector<std::string> devices;
    FILE* lsPipe;
    lsPipe = popen("ls -1 -- /sys/class/litepcie", "r");
    char tempBuffer[512];
    while (fscanf(lsPipe, "%s", tempBuffer) == 1)
        devices.push_back(tempBuffer);
    pclose(lsPipe);
    return devices;
}

LitePCIe::LitePCIe()
    : mFilePath()
    , mFileDescriptor(-1)
{
}

LitePCIe::~LitePCIe()
{
    Close();
}

OpStatus LitePCIe::Open(const std::filesystem::path& deviceFilename, uint32_t flags)
{
    mFilePath = deviceFilename;
    // use O_RDWR for now, because MMAP PROT_WRITE imples PROT_READ and will fail if file is opened write only
    flags &= ~O_WRONLY;
    flags |= O_RDWR;
    mFileDescriptor = open(mFilePath.c_str(), flags);
    if (mFileDescriptor < 0)
    {
        lime::error("LitePCIe: Failed to open (%s), errno(%i) %s", mFilePath.c_str(), errno, strerror(errno));
        // TODO: convert errno to OpStatus
        return OpStatus::FileNotFound;
    }

    litepcie_ioctl_mmap_dma_info info;
    int ret = ioctl(mFileDescriptor, LITEPCIE_IOCTL_MMAP_DMA_INFO, &info);
    if (ret != 0)
    {
        return OpStatus::Success;
    }

    mDMA.bufferCount = info.dma_rx_buf_count;
    mDMA.bufferSize = info.dma_rx_buf_size;
    litepcie_ioctl_lock lockInfo;
    // O_RDONLY has value of 0, so cannot detect if file is being opened as read only when other flags are preset
    if ((flags & O_WRONLY) != O_WRONLY || (flags & O_RDWR) == O_RDWR)
    {
        memset(&lockInfo, 0, sizeof(lockInfo));
        lockInfo.dma_writer_request = 1;
        ret = ioctl(mFileDescriptor, LITEPCIE_IOCTL_LOCK, &lockInfo);
        if (ret != 0 || lockInfo.dma_writer_status == 0)
        {
            const std::string msg = mFilePath.string() + ": DMA writer request denied"s;
            throw std::runtime_error(msg);
        }
        auto buf = static_cast<uint8_t*>(mmap(
            NULL, info.dma_rx_buf_size * info.dma_rx_buf_count, PROT_READ, MAP_SHARED, mFileDescriptor, info.dma_rx_buf_offset));
        if (buf == MAP_FAILED || buf == nullptr)
        {
            const std::string msg = mFilePath.string() + ": failed to MMAP Rx DMA buffer"s;
            throw std::runtime_error(msg);
        }
        mDMA.rxMemory = buf;
    }

    if ((flags & O_WRONLY) == O_WRONLY || (flags & O_RDWR) == O_RDWR)
    {
        memset(&lockInfo, 0, sizeof(lockInfo));
        lockInfo.dma_reader_request = 1;
        ret = ioctl(mFileDescriptor, LITEPCIE_IOCTL_LOCK, &lockInfo);
        if (ret != 0 || lockInfo.dma_reader_status == 0)
        {
            const std::string msg = mFilePath.string() + ": DMA reader request denied"s;
            throw std::runtime_error(msg);
        }
        auto buf = static_cast<uint8_t*>(mmap(
            NULL, info.dma_tx_buf_size * info.dma_tx_buf_count, PROT_WRITE, MAP_SHARED, mFileDescriptor, info.dma_tx_buf_offset));
        if (buf == MAP_FAILED || buf == nullptr)
        {
            const std::string msg = mFilePath.string() + ": failed to MMAP Tx DMA buffer"s;
            throw std::runtime_error(msg);
        }
        mDMA.txMemory = buf;
    }

    return OpStatus::Success;
}

bool LitePCIe::IsOpen() const
{
    return mFileDescriptor >= 0;
}

void LitePCIe::Close()
{
    if (mFileDescriptor >= 0)
    {
        litepcie_ioctl_lock lockInfo{ 0, 0, 0, 0, 0, 0 };
        if (mDMA.rxMemory)
        {
            munmap(mDMA.rxMemory, mDMA.bufferSize * mDMA.bufferCount);
            lockInfo.dma_writer_release = 1;
        }
        if (mDMA.txMemory)
        {
            munmap(mDMA.txMemory, mDMA.bufferSize * mDMA.bufferCount);
            lockInfo.dma_reader_release = 1;
        }
        ioctl(mFileDescriptor, LITEPCIE_IOCTL_LOCK, &lockInfo);
        close(mFileDescriptor);
    }
    mFileDescriptor = -1;
}

int LitePCIe::WriteControl(const uint8_t* buffer, const int length, int timeout_ms)
{
    return write(mFileDescriptor, buffer, length);
}

int LitePCIe::ReadControl(uint8_t* buffer, const int length, int timeout_ms)
{
    memset(buffer, 0, length);
    uint32_t status = 0;
    auto t1 = chrono::high_resolution_clock::now();
    do
    { //wait for status byte to change
        int ret = read(mFileDescriptor, &status, sizeof(status));
        if (ret < 0)
        {
            if (errno != EAGAIN)
                break;
        }
        if ((status & 0xFF00) != 0)
            break;
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    } while (std::chrono::duration_cast<std::chrono::milliseconds>(chrono::high_resolution_clock::now() - t1).count() < timeout_ms);

    return read(mFileDescriptor, buffer, length);
}

void LitePCIe::RxDMAEnable(bool enabled, uint32_t bufferSize, uint8_t irqPeriod)
{
    if (!IsOpen())
        return;
    litepcie_ioctl_dma_writer writer;
    memset(&writer, 0, sizeof(litepcie_ioctl_dma_writer));
    writer.enable = enabled ? 1 : 0;
    writer.hw_count = 0;
    writer.sw_count = 0;
    if (enabled)
    {
        writer.write_size = bufferSize;
        writer.irqFreq = irqPeriod;
    }
    int ret = ioctl(mFileDescriptor, LITEPCIE_IOCTL_DMA_WRITER, &writer);
    if (ret < 0)
        lime::error("Failed DMA writer ioctl. errno(%i) %s", errno, strerror(errno));
}

void LitePCIe::TxDMAEnable(bool enabled)
{
    if (!IsOpen())
        return;
    litepcie_ioctl_dma_reader reader;
    memset(&reader, 0, sizeof(litepcie_ioctl_dma_reader));
    reader.enable = enabled ? 1 : 0;
    reader.hw_count = 0;
    reader.sw_count = 0;
    int ret = ioctl(mFileDescriptor, LITEPCIE_IOCTL_DMA_READER, &reader);
    if (ret < 0)
        lime::error("Failed DMA reader ioctl. err(%i) %s", errno, strerror(errno));
}

IDMA::DMAState LitePCIe::GetRxDMAState()
{
    litepcie_ioctl_dma_writer dma;
    memset(&dma, 0, sizeof(litepcie_ioctl_dma_writer));
    dma.enable = 1;
    int ret = ioctl(mFileDescriptor, LITEPCIE_IOCTL_DMA_WRITER, &dma);
    if (ret)
        throw std::runtime_error("TransmitLoop IOCTL failed to get DMA reader counters"s);
    DMAState state;
    state.isEnabled = dma.enable;
    state.hardwareIndex = dma.hw_count;
    state.softwareIndex = dma.sw_count;
    return state;
}

IDMA::DMAState LitePCIe::GetTxDMAState()
{
    litepcie_ioctl_dma_reader dma;
    memset(&dma, 0, sizeof(litepcie_ioctl_dma_reader));
    dma.enable = 1;
    int ret = ioctl(mFileDescriptor, LITEPCIE_IOCTL_DMA_READER, &dma);
    if (ret)
        throw std::runtime_error("TransmitLoop IOCTL failed to get DMA writer counters"s);
    DMAState state;
    state.isEnabled = dma.enable;
    state.hardwareIndex = dma.hw_count;
    state.softwareIndex = dma.sw_count;
    return state;
}

bool LitePCIe::WaitRx()
{
    pollfd desc;
    desc.fd = mFileDescriptor;
    desc.events = POLLIN;

    struct timespec timeout_ts;
    timeout_ts.tv_sec = 0;
    timeout_ts.tv_nsec = 10e6;
    int ret = ppoll(&desc, 1, &timeout_ts, NULL);
    if (ret < 0)
    {
        const std::string msg = "DMA writer poll errno("s + std::to_string(errno) + ") "s + strerror(errno);
        if (errno == EINTR)
            return false;
        throw std::runtime_error(msg);
    }

    if (ret != 0 && desc.revents & POLLIN)
    {
        return true;
    }

    if (ret == 0)
    {
        lime::error("PollRx timeout"s);
    }

    auto state = GetRxDMAState();
    return state.hardwareIndex - state.softwareIndex != 0;
}

bool LitePCIe::WaitTx()
{
    pollfd desc;
    desc.fd = mFileDescriptor;
    desc.events = POLLOUT;

    struct timespec timeout_ts;
    timeout_ts.tv_sec = 0;
    timeout_ts.tv_nsec = 1e8;
    int ret = ppoll(&desc, 1, &timeout_ts, NULL);
    if (ret < 0)
    {
        if (errno == EINTR)
            return false;

        const std::string msg = "DMA reader poll errno("s + std::to_string(errno) + ") "s + strerror(errno);
        throw std::runtime_error(msg);
    }

    if (ret == 0)
    {
        lime::error("PollTx timeout"s);
        return false;
    }

    return desc.revents & POLLOUT;
}

int LitePCIe::SetRxDMAState(DMAState s)
{
    litepcie_ioctl_mmap_dma_update sub;
    memset(&sub, 0, sizeof(litepcie_ioctl_mmap_dma_update));
    sub.sw_count = s.softwareIndex;
    sub.buffer_size = mDMA.bufferSize;
    int ret = ioctl(mFileDescriptor, LITEPCIE_IOCTL_MMAP_DMA_WRITER_UPDATE, &sub);
    if (ret < 0)
    {
        throw std::runtime_error("DMA writer failed update"s);
    }
    return ret;
}

int LitePCIe::SetTxDMAState(DMAState s)
{
    litepcie_ioctl_mmap_dma_update sub;
    memset(&sub, 0, sizeof(litepcie_ioctl_mmap_dma_update));
    sub.sw_count = s.softwareIndex;
    sub.buffer_size = s.bufferSize;
    sub.genIRQ = s.generateIRQ;
    int ret = ioctl(mFileDescriptor, LITEPCIE_IOCTL_MMAP_DMA_READER_UPDATE, &sub);
    // if (ret < 0)
    // {
    //     std::string msg = "DMA reader failed update"s;
    // }
    return ret;
}

void LitePCIe::CacheFlush(TRXDir samplesDirection, DataTransferDirection dataDirection, uint16_t index)
{
    litepcie_cache_flush sub;
    memset(&sub, 0, sizeof(litepcie_cache_flush));
    sub.isTx = samplesDirection == TRXDir::Tx;
    sub.toDevice = dataDirection == DataTransferDirection::HostToDevice;
    sub.bufferIndex = index;
    int ret = ioctl(mFileDescriptor, LITEPCIE_IOCTL_CACHE_FLUSH, &sub);
    if (ret < 0)
    {
        std::string msg = "DMA reader failed update"s;
    }
}

IDMA::DMAState LitePCIe::GetState(TRXDir direction)
{
    switch (direction)
    {
    case TRXDir::Rx:
        return GetRxDMAState();
    case TRXDir::Tx:
        return GetTxDMAState();
    }
}

bool LitePCIe::Wait(TRXDir direction)
{
    switch (direction)
    {
    case TRXDir::Rx:
        return WaitRx();
    case TRXDir::Tx:
        return WaitTx();
    }
}

void LitePCIe::Disable(TRXDir direction)
{
    switch (direction)
    {
    case TRXDir::Rx:
        return RxDMAEnable(false, 0, 0);
    case TRXDir::Tx:
        return TxDMAEnable(false);
    }
}

void LitePCIe::RxEnable(uint32_t bufferSize, uint8_t irqPeriod)
{
    return RxDMAEnable(true, bufferSize, irqPeriod);
}

void LitePCIe::TxEnable()
{
    return TxDMAEnable(true);
}

int LitePCIe::GetBufferSize() const
{
    return mDMA.bufferSize;
}

int LitePCIe::GetBufferCount() const
{
    return mDMA.bufferCount;
}

uint8_t* const LitePCIe::GetMemoryAddress(TRXDir direction) const
{
    switch (direction)
    {
    case TRXDir::Rx:
        return mDMA.rxMemory;
    case TRXDir::Tx:
        return mDMA.txMemory;
    }
}

int LitePCIe::SetState(TRXDir direction, DMAState state)
{
    switch (direction)
    {
    case TRXDir::Rx:
        return SetRxDMAState(state);
    case TRXDir::Tx:
        return SetTxDMAState(state);
    }
}
