#ifndef LIME_LIMEPCIE_DMA_H
#define LIME_LIMEPCIE_DMA_H

#include "comms/IDMA.h"
#include "limesuiteng/OpStatus.h"

#include <cstdint>
#include <memory>

namespace lime {

class LimePCIe;

/** @brief Class for communicating with a PCIe device. */
class LimePCIeDMA : public IDMA
{
  public:
    /// @brief Constructs the DMA wrapper based on a PCIe interface.
    /// @param port The communications port to use.
    /// @param dir The direction of communications to build the class for.
    LimePCIeDMA(std::shared_ptr<LimePCIe> port, DataTransferDirection dir);
    virtual ~LimePCIeDMA();

    OpStatus Initialize() override;

    OpStatus Enable(bool enabled) override;
    OpStatus EnableContinuous(bool enabled, uint32_t maxTransferSize, uint8_t irqPeriod) override;

    IDMA::State GetCounters() override;
    OpStatus SubmitRequest(uint64_t index, uint32_t bytesCount, DataTransferDirection dir, bool irq) override;

    OpStatus Wait() override;
    void BufferOwnership(uint16_t index, DataTransferDirection dir) override;

    std::vector<IDMA::Buffer> GetBuffers() const override;
    std::string GetName() const override;

  private:
    std::vector<Buffer> mappings;
    std::shared_ptr<LimePCIe> port;
    DataTransferDirection dir;
    bool isInitialized;
};

} // namespace lime

#endif // LIME_LIMEPCIE_DMA_H
