#include "LitePCIeDMA.h"

#include <cassert>
#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "LitePCIe.h"
#include "limesuiteng/Logger.h"
#ifdef __unix__
    #include <unistd.h>
    #include <fcntl.h>
    #include <poll.h>
    #include <sys/mman.h>
    #include <sys/ioctl.h>
    #include "linux-kernel-module/litepcie.h"
#endif

using namespace std::literals::string_literals;

namespace lime {

LitePCIeDMA::LitePCIeDMA(std::shared_ptr<LitePCIe> port, DataTransferDirection dir)
    : port(port)
    , dir(dir)
{
    litepcie_ioctl_mmap_dma_info info{};
    int ret = ioctl(port->mFileDescriptor, LITEPCIE_IOCTL_MMAP_DMA_INFO, &info);
    if (ret != 0)
        return;

    mappings.reserve(info.dma_rx_buf_count);
    litepcie_ioctl_lock lockInfo{};

    if (dir == DataTransferDirection::DeviceToHost)
    {
        memset(&lockInfo, 0, sizeof(lockInfo));
        lockInfo.dma_writer_request = 1;
        ret = ioctl(port->mFileDescriptor, LITEPCIE_IOCTL_LOCK, &lockInfo);
        if (ret != 0 || lockInfo.dma_writer_status == 0)
        {
            const std::string msg = ": DMA writer request denied"s;
            throw std::runtime_error(msg);
        }
        auto buf = static_cast<uint8_t*>(mmap(NULL,
            info.dma_rx_buf_size * info.dma_rx_buf_count,
            PROT_READ,
            MAP_SHARED,
            port->mFileDescriptor,
            info.dma_rx_buf_offset));
        if (buf == MAP_FAILED || buf == nullptr)
        {
            const std::string msg = ": failed to MMAP Rx DMA buffer"s;
            throw std::runtime_error(msg);
        }
        for (size_t i = 0; i < info.dma_rx_buf_count; ++i)
            mappings.push_back({ buf + info.dma_rx_buf_size * i, info.dma_rx_buf_size });
    }
    else
    {
        memset(&lockInfo, 0, sizeof(lockInfo));
        lockInfo.dma_reader_request = 1;
        ret = ioctl(port->mFileDescriptor, LITEPCIE_IOCTL_LOCK, &lockInfo);
        if (ret != 0 || lockInfo.dma_reader_status == 0)
        {
            const std::string msg = ": DMA reader request denied"s;
            throw std::runtime_error(msg);
        }
        auto buf = static_cast<uint8_t*>(mmap(NULL,
            info.dma_tx_buf_size * info.dma_tx_buf_count,
            PROT_WRITE,
            MAP_SHARED,
            port->mFileDescriptor,
            info.dma_tx_buf_offset));
        if (buf == MAP_FAILED || buf == nullptr)
        {
            const std::string msg = ": failed to MMAP Tx DMA buffer"s;
            throw std::runtime_error(msg);
        }
        for (size_t i = 0; i < info.dma_tx_buf_count; ++i)
            mappings.push_back({ buf + info.dma_tx_buf_size * i, info.dma_tx_buf_size });
    }
}

LitePCIeDMA::~LitePCIeDMA()
{
    if (port->mFileDescriptor < 0)
        return;

    Enable(false);

    if (mappings.empty())
        return;

    munmap(mappings.front().buffer, mappings.front().size * mappings.size());

    litepcie_ioctl_lock lockInfo{ 0, 0, 0, 0, 0, 0 };
    if (dir == DataTransferDirection::DeviceToHost)
        lockInfo.dma_writer_release = 1;
    else
        lockInfo.dma_reader_release = 1;
    ioctl(port->mFileDescriptor, LITEPCIE_IOCTL_LOCK, &lockInfo);
}

OpStatus LitePCIeDMA::Enable(bool enabled)
{
    if (dir == DataTransferDirection::DeviceToHost)
    {
        litepcie_ioctl_dma_writer writer{};
        memset(&writer, 0, sizeof(litepcie_ioctl_dma_writer));
        writer.enable = enabled ? 1 : 0;
        writer.hw_count = 0;
        writer.sw_count = 0;
        if (enabled)
        {
            writer.write_size = 4096;
            writer.irqFreq = 16;
        }
        int ret = ioctl(port->mFileDescriptor, LITEPCIE_IOCTL_DMA_WRITER, &writer);
        if (ret < 0)
            return ReportError(OpStatus::IOFailure, "Failed DMA writer ioctl. errno(%i) %s", errno, strerror(errno));
    }
    else
    {
        litepcie_ioctl_dma_reader reader{};
        memset(&reader, 0, sizeof(litepcie_ioctl_dma_reader));
        reader.enable = enabled ? 1 : 0;
        reader.hw_count = 0;
        reader.sw_count = 0;
        int ret = ioctl(port->mFileDescriptor, LITEPCIE_IOCTL_DMA_READER, &reader);
        if (ret < 0)
            return ReportError(OpStatus::IOFailure, "Failed DMA reader ioctl. err(%i) %s", errno, strerror(errno));
    }
    return OpStatus::Success;
}

OpStatus LitePCIeDMA::EnableContinous(bool enabled, uint32_t maxTransferSize, uint8_t irqPeriod)
{
    assert(port->IsOpen());
    if (dir == DataTransferDirection::DeviceToHost)
    {
        litepcie_ioctl_dma_writer writer{};
        memset(&writer, 0, sizeof(litepcie_ioctl_dma_writer));
        writer.enable = enabled ? 1 : 0;
        writer.hw_count = 0;
        writer.sw_count = 0;
        if (enabled)
        {
            writer.write_size = maxTransferSize;
            writer.irqFreq = irqPeriod;
        }
        int ret = ioctl(port->mFileDescriptor, LITEPCIE_IOCTL_DMA_WRITER, &writer);
        if (ret < 0)
            return ReportError(OpStatus::IOFailure, "Failed DMA writer ioctl. errno(%i) %s", errno, strerror(errno));
    }
    else
    {
        litepcie_ioctl_dma_reader reader{};
        memset(&reader, 0, sizeof(litepcie_ioctl_dma_reader));
        reader.enable = enabled ? 1 : 0;
        reader.hw_count = 0;
        reader.sw_count = 0;
        int ret = ioctl(port->mFileDescriptor, LITEPCIE_IOCTL_DMA_READER, &reader);
        if (ret < 0)
            return ReportError(OpStatus::IOFailure, "Failed DMA reader ioctl. err(%i) %s", errno, strerror(errno));
    }
    return OpStatus::Success;
}

IDMA::State LitePCIeDMA::GetCounters()
{
    IDMA::State dma{};

    if (dir == DataTransferDirection::DeviceToHost)
    {
        litepcie_ioctl_dma_writer dmaWriter{};
        memset(&dmaWriter, 0, sizeof(litepcie_ioctl_dma_writer));
        dmaWriter.enable = 1;
        int ret = ioctl(port->mFileDescriptor, LITEPCIE_IOCTL_DMA_WRITER, &dmaWriter);
        if (ret)
            throw std::runtime_error("TransmitLoop IOCTL failed to get DMA reader counters");
        dma.consumerIndex = dmaWriter.sw_count;
        dma.producerIndex = dmaWriter.hw_count;
    }
    else
    {
        litepcie_ioctl_dma_reader dmaReader{};
        memset(&dmaReader, 0, sizeof(litepcie_ioctl_dma_reader));
        dmaReader.enable = 1;
        int ret = ioctl(port->mFileDescriptor, LITEPCIE_IOCTL_DMA_READER, &dmaReader);
        if (ret)
            throw std::runtime_error("TransmitLoop IOCTL failed to get DMA writer counters");
        dma.consumerIndex = dmaReader.hw_count;
        dma.producerIndex = dmaReader.sw_count;
    }
    return dma;
}

OpStatus LitePCIeDMA::SubmitRequest(uint64_t index, uint32_t bytesCount, DataTransferDirection direction, bool generateIRQ)
{
    assert(port->IsOpen());
    if (direction == DataTransferDirection::DeviceToHost)
    {
        litepcie_ioctl_mmap_dma_update sub{};
        memset(&sub, 0, sizeof(litepcie_ioctl_mmap_dma_update));
        sub.sw_count = index;
        sub.buffer_size = bytesCount;
        int ret = ioctl(port->mFileDescriptor, LITEPCIE_IOCTL_MMAP_DMA_WRITER_UPDATE, &sub);
        if (ret != 0)
            return OpStatus::Error;
    }
    else
    {
        litepcie_ioctl_mmap_dma_update sub{};
        memset(&sub, 0, sizeof(litepcie_ioctl_mmap_dma_update));
        sub.sw_count = index;
        sub.buffer_size = bytesCount;
        sub.genIRQ = generateIRQ;
        int ret = ioctl(port->mFileDescriptor, LITEPCIE_IOCTL_MMAP_DMA_READER_UPDATE, &sub);
        if (ret != 0)
            return OpStatus::Error;
    }
    return OpStatus::Success;
}

OpStatus LitePCIeDMA::Wait()
{
    assert(port->IsOpen());
    int eventMask;
    if (dir == DataTransferDirection::DeviceToHost)
        eventMask = POLLIN;
    else
        eventMask = POLLOUT;

    pollfd desc{};
    desc.fd = port->mFileDescriptor;
    desc.events = eventMask;

    timespec timeout_ts{};
    timeout_ts.tv_sec = 0;
    timeout_ts.tv_nsec = 10e6;

    int ret = ppoll(&desc, 1, &timeout_ts, NULL);
    if (ret < 0)
    {
        if (errno == EINTR)
            return OpStatus::Timeout;
        return ReportError(OpStatus::Error, "DMA poll errno("s + std::to_string(errno) + ") "s + strerror(errno));
    }

    if (ret != 0 && desc.revents & eventMask)
        return OpStatus::Success;
    return OpStatus::Timeout;
}

void LitePCIeDMA::BufferOwnership(uint16_t index, DataTransferDirection bufferDirection)
{
    litepcie_cache_flush sub{};
    memset(&sub, 0, sizeof(litepcie_cache_flush));
    sub.isTx = dir == DataTransferDirection::HostToDevice;
    sub.toDevice = bufferDirection == DataTransferDirection::HostToDevice;
    sub.bufferIndex = index;
    ioctl(port->mFileDescriptor, LITEPCIE_IOCTL_CACHE_FLUSH, &sub);
    // if (ret < 0)
    //     return OpStatus::Error;
    // return OpStatus::Success;
}

std::vector<IDMA::Buffer> LitePCIeDMA::GetBuffers() const
{
    return mappings;
}

} // namespace lime
