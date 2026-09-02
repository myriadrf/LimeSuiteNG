#ifndef LIME_XillybusDMAEmulation_H
#define LIME_XillybusDMAEmulation_H

#include "comms/IDMA.h"

#include "utilities/WorkerThread.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>
#include <queue>

namespace lime {

class Xillybus;
class XillybusReadThread;

/// @brief Class for emulating a DMA interface via Xillybus driver.
class XillybusDMAEmulation : public IDMA
{
  public:
    XillybusDMAEmulation(std::shared_ptr<Xillybus> port, DataTransferDirection dir);
    virtual ~XillybusDMAEmulation();
    OpStatus Initialize() override;

    OpStatus Enable(bool enabled) override;
    OpStatus EnableContinuous(bool enabled, uint32_t maxTransferSize, uint8_t irqPeriod) override;

    State GetCounters() override;
    OpStatus SubmitRequest(uint64_t index, uint32_t bytesCount, DataTransferDirection dir, bool irq) override;

    OpStatus Wait() override;
    void BufferOwnership(uint16_t index, DataTransferDirection dir) override;

    std::vector<IDMA::Buffer> GetBuffers() const override;
    std::string GetName() const override;

  private:
    friend XillybusReadThread;

    std::vector<Buffer> mappings;
    std::mutex queuesMutex;

    void AbortAllTransfers();

    std::string name;
    std::shared_ptr<Xillybus> port;
    DataTransferDirection dir;
    bool continuous;
    bool isEnabled;
    std::unique_ptr<XillybusReadThread> mXferThread;
    std::atomic<uint32_t> xfersDone;
    std::condition_variable xfer_cv;
};

} // namespace lime

#endif // LIME_XillybusDMAEmulation_H
