#ifndef LIME_IQStreamer_DMA_H
#define LIME_IQStreamer_DMA_H

#include <memory>
#include <mutex>

#include "limesuiteng/OpStatus.h"
#include "limesuiteng/config.h"

#include "comms/IDMA.h"

#include "chips/LA9310/MemoryWindow.h"
#include "host_dma_hif.h"

namespace lime {

class LA9310_PCIe;
class PCIe_CSR_Access;
class DMA_Buffer;

class IQStreamer_DMA
{
  public:
    enum DMA_Dir { DMA_FROM_DEVICE, DMA_TO_DEVICE };

    struct State {
        uint64_t transfersCompleted;
    };

    IQStreamer_DMA(DMA_Dir dir, volatile host_dma_hif_t* dma_hif, std::shared_ptr<LA9310_PCIe> pcie);
    ~IQStreamer_DMA();

    OpStatus Enable(bool enabled, bool loop_table = false);
    State GetCounters();

    OpStatus Wait();
    std::string GetName() const;

    OpStatus SubmitTransfer(DMA_Buffer buffer, size_t size, uint64_t timestamp, uint32_t flags);

  private:
    volatile host_dma_hif_t* dma_hif;
    std::shared_ptr<LA9310_PCIe> pcie;
    DMA_Dir dir;
};

} // namespace lime

#endif