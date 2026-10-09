#include "IQStreamer_DMA.h"

#include "comms/DMA_Buffer.h"
#include "comms/PCIe/LA9310_PCIe.h"
#include "chips/LA9310/virq.h"

#include <cassert>
#include <cstdint>
#include <chrono>
#include <vector>
#include <string>
#include <cstring>

#ifdef __unix__
    #include <unistd.h>
    #include <fcntl.h>
    #include <poll.h>
    #include <sys/mman.h>
    #include <sys/ioctl.h>
#endif

using namespace std::literals::string_literals;
using namespace std;

namespace lime {

IQStreamer_DMA::IQStreamer_DMA(
    DMA_Dir dir, volatile vspa_dma_hif_t* dma_hif, volatile vspa_regs* csr, std::shared_ptr<LA9310_PCIe> pcie)
    : dma_hif(dma_hif)
    , csr(csr)
    , pcie(pcie)
    , dir(dir)
{
    assert(csr);
    assert(dma_hif);
    htv_tcd_pending_flag_mask = dma_hif->htv_tcd_pending_flag_mask;
}

IQStreamer_DMA::~IQStreamer_DMA()
{
    // Enable(false, false);
}

OpStatus IQStreamer_DMA::Enable(bool enabled, bool loop_table)
{
    assert(dma_hif);
    // chrono::milliseconds timeout(1000);
    // auto t1 = std::chrono::high_resolution_clock::now();
    // auto t2 = t1;
    // while (dma_hif->pending && (t2 - t1) < timeout)
    // {
    //     t2 = std::chrono::high_resolution_clock::now();
    // }
    // if (t2 - t1 > timeout)
    // {
    //     printf("DMA enable timeout\n");
    //     return OpStatus::Timeout;
    // }

    // dma_hif->enable = enabled;
    // dma_hif->loop_mode = loop_table;
    // dma_hif->clear = !enabled;
    // dma_hif->pending = true;

    // // Wait for operation to complete
    // t1 = std::chrono::high_resolution_clock::now();
    // t2 = t1;
    // while (dma_hif->pending && (t2 - t1) < timeout)
    // {
    //     t2 = std::chrono::high_resolution_clock::now();
    // }
    // if (t2 - t1 > timeout)
    // {
    //     printf("DMA wait enable timeout\n");
    //     return OpStatus::Timeout;
    // }

    return OpStatus::Success;
}

IQStreamer_DMA::State IQStreamer_DMA::GetCounters()
{
    IQStreamer_DMA::State dma{};
    dma.transfersCompleted = dma_hif->tcd_fifo.done & 0xFFFFFFFF;
    return dma;
}

OpStatus IQStreamer_DMA::Wait()
{
    const uint32_t bit = dir == DMA_Dir::DMA_FROM_DEVICE ? LA9310_VIRQ::VSPA_DDR_WRITE_DONE : LA9310_VIRQ::VSPA_DDR_READ_DONE;
    OpStatus status = pcie->WaitSIRQ(bit, chrono::milliseconds(500));
    if (status != OpStatus::Success)
    {
        // printf("IQStreamDMA-Wait %s timeout f:%x? %i\n", (dir == DMA_Dir::DMA_FROM_DEVICE ? "Rx" : "Tx"), bit, (int)status);
        return status;
    }
    status = pcie->ClearSIRQ((1 << bit));
    if (status != OpStatus::Success)
    {
        // lime::error("LA9310_PCIe: RunControlCommand failed clear IRQ\n");
    }
    return status;
}

std::string IQStreamer_DMA::GetName() const
{
    return "IQStreamer_DMA";
}

OpStatus IQStreamer_DMA::SubmitTransfer(DMA_Buffer buffer, size_t size, uint64_t timestamp, uint32_t flags)
{
    if (size == 0)
        return OpStatus::InvalidValue;

    auto t1 = std::chrono::high_resolution_clock::now();
    auto t2 = t1;

    chrono::milliseconds timeout(1000);
    // printf("tcdsz:%i\n", tcd_fifo_size(&dma_hif->tcd_fifo));
    int fifo_size = tcd_fifo_size(&dma_hif->tcd_fifo);
    while (fifo_size == MFIFO_SIZE && (t2 - t1) < timeout)
    {
        fifo_size = tcd_fifo_size(&dma_hif->tcd_fifo);
        t2 = std::chrono::high_resolution_clock::now();
    }
    if (t2 - t1 > timeout)
    {
        printf("submit timeout\n");
        return OpStatus::Timeout;
    }

    volatile dma_tcd_t* tcd = tcd_fifo_back(&dma_hif->tcd_fifo);
    tcd->la9310_mem_address = buffer.endpoint_pa();
    assert(size <= buffer.size());
    tcd->timestamp_lsb = timestamp & 0xFFFFFFFF;
    tcd->timestamp_msb = (timestamp >> 32) & 0xFFFFFFFF;
    tcd->flags = flags;
    tcd->size = size;
    tcd_fifo_push(&dma_hif->tcd_fifo);
    // printf("fs: %i\n", fifo_size);
    if (fifo_size < 1)
    {
        // Only need to signal TCD insertion if the FIFO is empty, i.e for the initial start.
        // after that data transfers are self perpetuating as long as FIFO doesn't become empty.
        // Signaling each TCD insertion is fine, but that makes VSPA go each time, such superfuluos
        // waking up can affect data processing pacing.
        return htv_signal(htv_tcd_pending_flag_mask);
    }
    else
        return OpStatus::Success;
}

OpStatus IQStreamer_DMA::htv_signal(uint32_t mask)
{
    csr->host_vcpu_flags0 = mask; // bit writes to flags get OR'ed, can be cleared only by VCPU
    return OpStatus::Success;
}

} // namespace lime
