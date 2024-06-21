#ifndef LIME_LITEPCIE_DMA_H
#define LIME_LITEPCIE_DMA_H

#include "comms/IDMA.h"
#include "limesuiteng/OpStatus.h"

#include <cstdint>
#include <memory>

namespace lime {

class LitePCIe;

/** @brief Class for communicating with a PCIe device. */
class LitePCIeDMA : public IDMA
{
  public:
    LitePCIeDMA(std::shared_ptr<LitePCIe> port, DataTransferDirection dir);
    virtual ~LitePCIeDMA();

    OpStatus Enable(bool enabled) override;
    OpStatus EnableContinuous(bool enabled, uint32_t maxTransferSize, uint8_t irqPeriod) override;

    IDMA::State GetCounters() override;
    OpStatus SubmitRequest(uint64_t index, uint32_t bytesCount, DataTransferDirection dir, bool irq) override;

    OpStatus Wait() override;
    void BufferOwnership(uint16_t index, DataTransferDirection dir) override;

    std::vector<IDMA::Buffer> GetBuffers() const override;

  private:
    std::vector<Buffer> mappings;
    std::shared_ptr<LitePCIe> port;
    DataTransferDirection dir;
};

} // namespace lime

#endif // LIME_LITEPCIE_DMA_H
