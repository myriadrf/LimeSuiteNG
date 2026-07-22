#ifndef LIME_VSPA_DMA_H
#define LIME_VSPA_DMA_H

#include <memory>
#include <mutex>

#include "limesuiteng/OpStatus.h"
#include "limesuiteng/config.h"

#include "comms/IDMA.h"
#include "dma_table.h"

#include "MemoryWindow.h"

typedef struct Tx_DMA_hif {
    uint64_t timestamp;
    uint32_t la9310_mem_address;
    uint32_t size;
    uint32_t flags;
    uint32_t loop_counter;
} tx_dma_hif_t;

namespace lime {

class LA9310_PCIe;
class PCIe_CSR_Access;
class VSPA_mailbox;

class VSPA_DMA : public lime::IDMA
{
  public:
    struct MemoryHandle {
        void* host_va;
        uint32_t la9310_pa;
        uint32_t size;
    };

    VSPA_DMA(std::shared_ptr<VSPA_mailbox> mailbox, tx_dma_hif_t* hif, MemoryWindow dma_memory);
    ~VSPA_DMA();

    OpStatus Initialize() override;

    OpStatus Enable(bool enabled) override;
    OpStatus EnableContinuous(bool enabled, uint32_t maxTransferSize, uint8_t irqPeriod) override;
    State GetCounters() override;

    OpStatus SubmitRequest(uint64_t index, uint32_t bytesCount, DataTransferDirection dir, bool irq) override;

    OpStatus Wait() override;
    void BufferOwnership(uint16_t index, DataTransferDirection dir) override;

    std::vector<Buffer> GetBuffers() const override;

    std::string GetName() const override;

    OpStatus SubmitTransfer(uint32_t index, size_t size, uint32_t timestamp, uint32_t flags);

  private:
    std::shared_ptr<VSPA_mailbox> mailbox;
    std::vector<Buffer> mappings;

    MemoryWindow dma_window;

    tx_dma_hif_t* hif;
};

} // namespace lime

#endif