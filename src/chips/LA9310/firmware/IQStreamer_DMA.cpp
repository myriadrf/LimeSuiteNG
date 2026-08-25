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

using namespace std::literals::string_literals;
using namespace std;

namespace lime {

static constexpr uint32_t blockSize = 32 * 1024;

IQStreamer_DMA::IQStreamer_DMA(DMA_Dir dir, volatile host_dma_hif_t* dma_hif, std::shared_ptr<LA9310_PCIe> pcie)
    : dma_hif(dma_hif)
    , pcie(pcie)
    , dir(dir)
{
    assert(dma_hif);
    // for (uint32_t sz = 0; sz < dma_memory.size; sz += blockSize)
    //     mappings.push_back({ static_cast<uint8_t*>(dma_memory.host_va) + sz, blockSize });
}

IQStreamer_DMA::~IQStreamer_DMA()
{
    Enable(false, false);
}

OpStatus IQStreamer_DMA::Enable(bool enabled, bool loop_table)
{
    assert(dma_hif);
    chrono::milliseconds timeout(1000);
    auto t1 = std::chrono::high_resolution_clock::now();
    auto t2 = t1;
    while (dma_hif->pending && (t2 - t1) < timeout)
    {
        t2 = std::chrono::high_resolution_clock::now();
    }
    if (t2 - t1 > timeout)
    {
        printf("DMA enable timeout\n");
        return OpStatus::Timeout;
    }

    dma_hif->enable = enabled;
    dma_hif->loop_mode = loop_table;
    dma_hif->clear = !enabled;
    dma_hif->pending = true;
    return OpStatus::Success;
}

IQStreamer_DMA::State IQStreamer_DMA::GetCounters()
{
    IQStreamer_DMA::State dma{};
    dma.transfersCompleted = dma_hif->tcd_complete_counter & 0xFFFF;
    return dma;
}

OpStatus IQStreamer_DMA::Wait()
{
    const uint32_t bit = dir == DMA_Dir::DMA_FROM_DEVICE ? LA9310_VIRQ::VSPA_DDR_WRITE_DONE : LA9310_VIRQ::VSPA_DDR_READ_DONE;
    OpStatus status = pcie->WaitSIRQ(bit, chrono::milliseconds(500));
    if (status != OpStatus::Success)
    {
        printf("IQStreamDMA-Wait timeout f:%x? %i\n", bit, (int)status);
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
    // if dma_hif->size is not 0, M4 has not yet processed previous request
    // printf("tcd %x, %i\n", buffer.endpoint_pa(), size);
    chrono::milliseconds timeout(1000);
    while (dma_hif->pending && (t2 - t1) < timeout)
    {
        t2 = std::chrono::high_resolution_clock::now();
    }
    if (t2 - t1 > timeout)
    {
        printf("submit timeout\n");
        return OpStatus::Timeout;
    }

    dma_hif->input_tcd.la9310_mem_address = buffer.endpoint_pa();
    assert(size <= buffer.size());
    dma_hif->input_tcd.timestamp = timestamp;
    dma_hif->input_tcd.flags = flags;
    dma_hif->input_tcd.size = size;
    dma_hif->pending = true;
    return OpStatus::Success;
}

} // namespace lime
