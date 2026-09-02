#include "XillybusDMAEmulation.h"

#include "comms/PCIe/xillybus/XillybusDMAEmulation.h"

#include "CommonFunctions.h"
#include "limesuiteng/Logger.h"
#include "comms/PCIe/xillybus/Xillybus.h"

namespace lime {

class XillybusReadThread : public WorkerThread
{
  public:
    XillybusReadThread(XillybusDMAEmulation* dma, size_t xferSize)
        : dma(dma)
        , xferSize(xferSize)
    {
    }

    bool Work() override
    {
        constexpr auto timeout = std::chrono::milliseconds(250);

        size_t bytes_read =
            dma->port->Read(dma->mappings[dma->xfersDone.load() % dma->mappings.size()].buffer, xferSize, timeout.count());
        if (bytes_read != xferSize)
        {
            return false; // stop work
        }
        ++dma->xfersDone;
        xfer_cv.notify_all();
        return true;
    }

    XillybusDMAEmulation* dma;
    std::condition_variable xfer_cv;
    std::mutex xfer_mutex;
    size_t xferSize;

    OpStatus WaitForXfer()
    {
        std::unique_lock lk{ xfer_mutex };
        return xfer_cv.wait_for(lk, std::chrono::milliseconds(1000)) == std::cv_status::timeout ? OpStatus::Timeout
                                                                                                : OpStatus::Success;
    }
};

// Too many async requests adds overhead and makes transfers timing consistency worse
static constexpr int maxAsyncTransfers = 16;
static constexpr uint32_t mappingBufferSize = 65536;

XillybusDMAEmulation::XillybusDMAEmulation(std::shared_ptr<Xillybus> port, DataTransferDirection dir)
    : port(port)
    , dir(dir)
    , continuous(false)
    , isEnabled(false)
    , xfersDone(0)
{
    name = strFormat("Xillybus");
    mappings.resize(maxAsyncTransfers);
    for (auto& memoryBlock : mappings)
    {
        memoryBlock.size = mappingBufferSize;
        memoryBlock.buffer = new uint8_t[memoryBlock.size];
    }
}

OpStatus XillybusDMAEmulation::Initialize()
{
    return OpStatus::Success;
}

XillybusDMAEmulation::~XillybusDMAEmulation()
{
    AbortAllTransfers();
    for (auto& memoryBlock : mappings)
        delete[] memoryBlock.buffer;
}

void XillybusDMAEmulation::AbortAllTransfers()
{
    if (mXferThread)
    {
        mXferThread->Stop();
        mXferThread->Wait();
    }
    xfersDone.store(0);
}

std::vector<IDMA::Buffer> XillybusDMAEmulation::GetBuffers() const
{
    return mappings;
}

std::string XillybusDMAEmulation::GetName() const
{
    return name;
}

OpStatus XillybusDMAEmulation::Enable(bool enable)
{
    if (isEnabled && enable)
        return OpStatus::Busy;
    continuous = false;
    if (!enable)
    {
        AbortAllTransfers();
        isEnabled = false;
        port->Reset();
        return OpStatus::Success;
    }

    xfersDone.store(0);
    isEnabled = true;
    return OpStatus::Success;
}

OpStatus XillybusDMAEmulation::EnableContinuous(bool enable, uint32_t maxTransferSize, uint8_t irqPeriod)
{
    OpStatus status = Enable(enable);
    continuous = true;

    if (!enable)
        return status;

    if (maxTransferSize == 0 || maxTransferSize > mappingBufferSize)
        return OpStatus::InvalidValue;

    if (dir != DataTransferDirection::DeviceToHost)
        return OpStatus::Success;
    mXferThread = std::make_unique<XillybusReadThread>(this, maxTransferSize);
    mXferThread->Start();
    return status;
}

XillybusDMAEmulation::State XillybusDMAEmulation::GetCounters()
{
    State counters;
    counters.transfersCompleted = xfersDone.load();
    return counters;
}

OpStatus XillybusDMAEmulation::SubmitRequest(uint64_t index, uint32_t bytesCount, DataTransferDirection dir, bool irq)
{
    if (!isEnabled)
        return OpStatus::Error;

    if (continuous)
        return OpStatus::Success;

    assert(bytesCount > 0);
    assert(index < mappings.size());
    if (index >= mappings.size() || bytesCount == 0 || bytesCount > mappings[index].size)
        return OpStatus::InvalidValue;

    size_t bytes_xferred = 0;
    if (dir == DataTransferDirection::DeviceToHost)
        bytes_xferred = port->Read(mappings[index].buffer, bytesCount, 1000);
    else
        bytes_xferred = port->Write(mappings[index].buffer, bytesCount, 1000);

    ++xfersDone;

    if (bytes_xferred == bytesCount)
        return OpStatus::Success;
    else
        return OpStatus::Error;
}

OpStatus XillybusDMAEmulation::Wait()
{
    if (continuous && isEnabled)
        return mXferThread->WaitForXfer();
    return OpStatus::Success;
}

void XillybusDMAEmulation::BufferOwnership(uint16_t index, DataTransferDirection dir)
{
    // works with buffer copying, no need to transfer ownership/flush caches.
    // do nothing.
}

} // namespace lime
