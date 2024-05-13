#include "LimeLitePCIe.h"

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
    #include "linux-kernel-module/limelitepcie.h"
#endif

using namespace std;
using namespace lime;
using namespace std::literals::string_literals;

#define EXTRA_CHECKS 1

std::vector<std::string> LimeLitePCIe::GetEndpointsWithPattern(const std::string& deviceAddr, const std::string& regex)
{
    std::vector<std::string> devices;
    FILE* lsPipe;

    std::string cmd = "find "s + deviceAddr + " -readable -name "s + regex;
    lsPipe = popen(cmd.c_str(), "r");
    char tempBuffer[512];
    while (fscanf(lsPipe, "%s", tempBuffer) == 1)
        devices.push_back(tempBuffer);
    pclose(lsPipe);
    return devices;
}

std::vector<std::string> LimeLitePCIe::GetPCIeDeviceList()
{
    std::vector<std::string> devices;
    FILE* lsPipe;
    lsPipe = popen("ls -1 -- /sys/class/limelitepcie", "r");
    char tempBuffer[512];
    while (fscanf(lsPipe, "%s", tempBuffer) == 1)
    {
        // Kernel code fakes directories by replacing '/' char with '!'
        // open() can't open that
        // Replace '!' with '/' so we could open device
        std::string parsedDevicePath{tempBuffer};
        for (auto &c : parsedDevicePath)
        {
            if (c == '!')
                c = '/';
        }
        devices.push_back(parsedDevicePath);
    }
    pclose(lsPipe);
    return devices;
}

LimeLitePCIe::LimeLitePCIe()
    : mFilePath()
    , mFileDescriptor(-1)
    , isConnected(false)
{
}

LimeLitePCIe::~LimeLitePCIe()
{
    Close();
}

OpStatus LimeLitePCIe::Open(const std::filesystem::path& deviceFilename, uint32_t flags)
{
    mFilePath = deviceFilename;

    // use O_RDWR for now, because MMAP PROT_WRITE imples PROT_READ and will fail if file is opened write only
    flags &= ~O_WRONLY;
    flags |= O_RDWR;
    mFileDescriptor = open(mFilePath.c_str(), flags);
    if (mFileDescriptor < 0)
    {
        isConnected = false;
        lime::error("LimeLitePCIe: Failed to open (%s), errno(%i) %s", mFilePath.c_str(), errno, strerror(errno));
        // TODO: convert errno to OpStatus
        return OpStatus::FileNotFound;
    }

    limelitepcie_version version;
    int ret = ioctl(mFileDescriptor, LIMELITEPCIE_IOCTL_VERSION, &version);
    if (ret == 0)
        lime::debug("PCIe Driver Version: %i.%i.%i", version.major, version.minor, version.patch);
    else
        lime::error("Unable to get PCIe Driver Version");

    limelitepcie_ioctl_mmap_dma_info info;
    ret = ioctl(mFileDescriptor, LIMELITEPCIE_IOCTL_MMAP_DMA_INFO, &info);
    if (ret != 0)
    {
        isConnected = true;
        return OpStatus::Success;
    }

    mDMA.bufferCount = info.dma_rx_buf_count;
    mDMA.bufferSize = info.dma_rx_buf_size;
    limelitepcie_ioctl_lock lockInfo;
    // O_RDONLY has value of 0, so cannot detect if file is being opened as read only when other flags are preset
    if ((flags & O_WRONLY) != O_WRONLY || (flags & O_RDWR) == O_RDWR)
    {
        memset(&lockInfo, 0, sizeof(lockInfo));
        lockInfo.dma_writer_request = 1;
        ret = ioctl(mFileDescriptor, LIMELITEPCIE_IOCTL_LOCK, &lockInfo);
        if (ret != 0 || lockInfo.dma_writer_status == 0)
        {
            const std::string msg = mFilePath.string() + ": DMA writer request denied"s;
            throw std::runtime_error(msg);
        }
        uint8_t* buf = static_cast<uint8_t*>(mmap(
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
        ret = ioctl(mFileDescriptor, LIMELITEPCIE_IOCTL_LOCK, &lockInfo);
        if (ret != 0 || lockInfo.dma_reader_status == 0)
        {
            const std::string msg = mFilePath.string() + ": DMA reader request denied"s;
            throw std::runtime_error(msg);
        }
        uint8_t* buf = static_cast<uint8_t*>(mmap(
            NULL, info.dma_tx_buf_size * info.dma_tx_buf_count, PROT_WRITE, MAP_SHARED, mFileDescriptor, info.dma_tx_buf_offset));
        if (buf == MAP_FAILED || buf == nullptr)
        {
            const std::string msg = mFilePath.string() + ": failed to MMAP Tx DMA buffer"s;
            throw std::runtime_error(msg);
        }
        mDMA.txMemory = buf;
    }

    isConnected = true;
    return OpStatus::Success;
}

bool LimeLitePCIe::IsOpen()
{
    return mFileDescriptor >= 0;
}

void LimeLitePCIe::Close()
{
    if (mFileDescriptor >= 0)
    {
        limelitepcie_ioctl_lock lockInfo{ 0, 0, 0, 0, 0, 0 };
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
        ioctl(mFileDescriptor, LIMELITEPCIE_IOCTL_LOCK, &lockInfo);
        close(mFileDescriptor);
    }
    mFileDescriptor = -1;
}

int LimeLitePCIe::WriteControl(const uint8_t* buffer, const int length, int timeout_ms)
{
    return write(mFileDescriptor, buffer, length);
}

int LimeLitePCIe::ReadControl(uint8_t* buffer, const int length, int timeout_ms)
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

    //if ((status & 0xFF00) == 0)
    //throw std::runtime_error("LimeLitePCIe read status timeout"s);
    return read(mFileDescriptor, buffer, length);
}

int LimeLitePCIe::WriteRaw(const uint8_t* buffer, const int length, int timeout_ms)
{
    if (mFileDescriptor < 0)
        throw std::runtime_error("LimeLitePCIe port not opened"s);
    auto t1 = chrono::high_resolution_clock::now();
    int bytesRemaining = length;
    while (bytesRemaining > 0 &&
           std::chrono::duration_cast<std::chrono::milliseconds>(chrono::high_resolution_clock::now() - t1).count() < timeout_ms)
    {
        int bytesOut = write(mFileDescriptor, buffer, bytesRemaining);

        if (bytesOut < 0)
        {
            switch (errno)
            {
            case EAGAIN:
                bytesOut = 0;
                break;
            case EINTR:
                lime::error("Write EINTR"s);
                continue;
            default:
                lime::error("Write default"s);
                return errno;
            }
        }
        if (bytesOut == 0)
        {
            pollfd desc;
            desc.fd = mFileDescriptor;
            desc.events = POLLOUT;

            const int pollTimeout =
                std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t1).count();
            if (pollTimeout <= 0)
                break;
            lime::debug("poll for %ims", pollTimeout);
            int ret = poll(&desc, 1, pollTimeout);
            if (ret < 0)
            {
                lime::error("Write poll errno(%i) %s", errno, strerror(errno));
                return -errno;
            }
            else if (ret == 0) // timeout
            {
                lime::error("Write poll timeout %i", pollTimeout);
                break;
            }
            continue;
        }
        bytesRemaining -= bytesOut;
        buffer += bytesOut;
    }

    return length - bytesRemaining;
}

int LimeLitePCIe::ReadRaw(uint8_t* buffer, const int length, int timeout_ms)
{
    if (mFileDescriptor < 0)
        throw std::runtime_error("LimeLitePCIe port not opened"s);

    int bytesRemaining = length;
    uint8_t* dest = buffer;
    auto t1 = chrono::high_resolution_clock::now();
    do
    {
        int bytesIn = read(mFileDescriptor, dest, bytesRemaining);

        if (bytesIn < 0)
        {
            switch (errno)
            {
            case EAGAIN:
                bytesIn = 0;
                return length - bytesRemaining;
            case EINTR:
                lime::error("Read EINTR"s);
                continue;
            default:
                lime::error("Read default"s);
                return -errno;
            }
        }
        if (bytesIn == 0)
        {
            break;
            pollfd desc;
            desc.fd = mFileDescriptor;
            desc.events = POLLIN;

            const int pollTimeout =
                timeout_ms -
                std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t1).count();
            if (pollTimeout <= 0)
            {
                lime::error("Read poll timeout of %i", pollTimeout);
                return length - bytesRemaining;
            }
            int ret = poll(&desc, 1, pollTimeout);
            if (ret < 0)
            {
                lime::error("Read poll errno(%i) %s", errno, strerror(errno));
                return -errno;
            }
            else if (ret == 0) // timeout
            {
                lime::error("Read poll timeout %i", timeout_ms);
                break;
            }
            continue;
        }
#ifdef EXTRA_CHECKS
        if (bytesIn > bytesRemaining)
        {
            lime::error("LimeLitePCIe::ReadRaw read expected(%i), returned(%i)", bytesRemaining, bytesIn);
            return -1;
        }
#endif
        bytesRemaining -= bytesIn;
        dest += bytesIn;
    } while (bytesRemaining > 0 &&
             std::chrono::duration_cast<std::chrono::milliseconds>(chrono::high_resolution_clock::now() - t1).count() < timeout_ms);
#ifdef EXTRA_CHECKS
    // if (bytesRemaining > 0)
    //     lime::error("LimeLitePCIe::ReadRaw %i bytes remaining after timeout", bytesRemaining);
    // auto rdTime = std::chrono::duration_cast<std::chrono::microseconds>(chrono::high_resolution_clock::now() - t1).count();
    // if(rdTime > 100)
    //     lime::error("ReadRaw too long %i", rdTime);
#endif
    return length - bytesRemaining;
}

void LimeLitePCIe::RxDMAEnable(bool enabled, uint32_t bufferSize, uint8_t irqPeriod)
{
    if (!IsOpen())
        return;
    limelitepcie_ioctl_dma_writer writer;
    memset(&writer, 0, sizeof(limelitepcie_ioctl_dma_writer));
    writer.enable = enabled ? 1 : 0;
    writer.hw_count = 0;
    writer.sw_count = 0;
    if (enabled)
    {
        writer.write_size = bufferSize;
        writer.irqFreq = irqPeriod;
    }
    int ret = ioctl(mFileDescriptor, LIMELITEPCIE_IOCTL_DMA_WRITER, &writer);
    if (ret < 0)
        lime::error("Failed DMA writer ioctl. errno(%i) %s", errno, strerror(errno));
}

void LimeLitePCIe::TxDMAEnable(bool enabled)
{
    if (!IsOpen())
        return;
    limelitepcie_ioctl_dma_reader reader;
    memset(&reader, 0, sizeof(limelitepcie_ioctl_dma_reader));
    reader.enable = enabled ? 1 : 0;
    reader.hw_count = 0;
    reader.sw_count = 0;
    int ret = ioctl(mFileDescriptor, LIMELITEPCIE_IOCTL_DMA_READER, &reader);
    if (ret < 0)
        lime::error("Failed DMA reader ioctl. err(%i) %s", errno, strerror(errno));
}

LimeLitePCIe::DMAState LimeLitePCIe::GetRxDMAState()
{
    limelitepcie_ioctl_dma_writer dma;
    memset(&dma, 0, sizeof(limelitepcie_ioctl_dma_writer));
    dma.enable = 1;
    int ret = ioctl(mFileDescriptor, LIMELITEPCIE_IOCTL_DMA_WRITER, &dma);
    if (ret)
        throw std::runtime_error("TransmitLoop IOCTL failed to get DMA reader counters"s);
    DMAState state;
    state.enabled = dma.enable;
    state.hwIndex = dma.hw_count;
    state.swIndex = dma.sw_count;
    return state;
}

LimeLitePCIe::DMAState LimeLitePCIe::GetTxDMAState()
{
    limelitepcie_ioctl_dma_reader dma;
    memset(&dma, 0, sizeof(limelitepcie_ioctl_dma_reader));
    dma.enable = 1;
    int ret = ioctl(mFileDescriptor, LIMELITEPCIE_IOCTL_DMA_READER, &dma);
    if (ret)
        throw std::runtime_error("TransmitLoop IOCTL failed to get DMA writer counters"s);
    DMAState state;
    state.enabled = dma.enable;
    state.hwIndex = dma.hw_count;
    state.swIndex = dma.sw_count;
    return state;
}

bool LimeLitePCIe::WaitRx()
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
        //throw std::runtime_error(msg);
    }
    else if (ret == 0)
    {
        //lime::error("PollRx timeout"s);
    }
    else
    {
        if (desc.revents & POLLIN)
            return true;
    }
    auto state = GetRxDMAState();
    if (state.hwIndex - state.swIndex != 0)
        return true;
    else
        return false;
}

bool LimeLitePCIe::WaitTx()
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
    else if (ret == 0)
    {
        //lime::error("PollTx timeout"s);
    }
    else
    {
        return desc.revents & POLLOUT;
    }
    return ret > 0;
}

int LimeLitePCIe::SetRxDMAState(DMAState s)
{
    limelitepcie_ioctl_mmap_dma_update sub;
    memset(&sub, 0, sizeof(limelitepcie_ioctl_mmap_dma_update));
    sub.sw_count = s.swIndex;
    sub.buffer_size = mDMA.bufferSize;
    int ret = ioctl(mFileDescriptor, LIMELITEPCIE_IOCTL_MMAP_DMA_WRITER_UPDATE, &sub);
    if (ret < 0)
    {
        throw std::runtime_error("DMA writer failed update"s);
    }
    return ret;
}

int LimeLitePCIe::SetTxDMAState(DMAState s)
{
    limelitepcie_ioctl_mmap_dma_update sub;
    memset(&sub, 0, sizeof(limelitepcie_ioctl_mmap_dma_update));
    sub.sw_count = s.swIndex;
    sub.buffer_size = s.bufferSize;
    sub.genIRQ = s.genIRQ;
    int ret = ioctl(mFileDescriptor, LIMELITEPCIE_IOCTL_MMAP_DMA_READER_UPDATE, &sub);
    // if (ret < 0)
    // {
    //     std::string msg = "DMA reader failed update"s;
    // }
    return ret;
}

void LimeLitePCIe::CacheFlush(bool isTx, bool toDevice, uint16_t index)
{
    limelitepcie_cache_flush sub;
    memset(&sub, 0, sizeof(limelitepcie_cache_flush));
    sub.isTx = isTx;
    sub.toDevice = toDevice;
    sub.bufferIndex = index;
    int ret = ioctl(mFileDescriptor, LIMELITEPCIE_IOCTL_CACHE_FLUSH, &sub);
    if (ret < 0)
    {
        std::string msg = "DMA reader failed update"s;
    }
}
