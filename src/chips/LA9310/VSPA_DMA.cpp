#include "VSPA_DMA.h"

#include <cassert>
#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "VSPA_mailbox.h"
#include "VSPA_iqplayer.h"

using namespace std::literals::string_literals;

namespace lime {

static const uint32_t vspa_cpu_id = 0;
static const uint32_t vspa_mbox_id = 0;
static constexpr uint32_t blockSize = 32 * 1024;

VSPA_DMA::VSPA_DMA(std::shared_ptr<VSPA_mailbox> mailbox, tx_dma_hif_t* hif, MemoryWindow dma_memory)
    : mailbox(mailbox)
    , dma_window(dma_memory)
    , hif(hif)
{
    for (uint32_t sz = 0; sz < dma_memory.size; sz += blockSize)
        mappings.push_back({ static_cast<uint8_t*>(dma_memory.host_va) + sz, blockSize });
}

VSPA_DMA::~VSPA_DMA()
{
    Enable(false);
}

OpStatus VSPA_DMA::Initialize()
{
    return OpStatus::Success;
}

OpStatus VSPA_DMA::Enable(bool ddr_enable)
{
    const mbox_opc_e command = MBOX_OPC_TX_CONTROL;
    uint32_t loword = 0;
    uint32_t hiword = 0;
    hiword |= command << 24;
    hiword |= ddr_enable ? (1 << 0) : 0;
    uint64_t value = uint64_t(hiword) << 32 | loword;
    uint64_t response = 0;
    OpStatus status = mailbox->Message(vspa_cpu_id, vspa_mbox_id, value, &response);
    if (status != OpStatus::Success)
        return status;
    return (response & 0xFF) == 0 ? OpStatus::Success : OpStatus::Error;
}

OpStatus VSPA_DMA::EnableContinuous(bool enabled, uint32_t maxTransferSize, uint8_t irqPeriod)
{
    return OpStatus::NotImplemented;
}

IDMA::State VSPA_DMA::GetCounters()
{
    IDMA::State dma{};
    dma.transfersCompleted = hif->loop_counter;
    return dma;
}

OpStatus VSPA_DMA::SubmitRequest(uint64_t index, uint32_t bytesCount, DataTransferDirection direction, bool generateIRQ)
{
    return OpStatus::NotImplemented;
}

OpStatus VSPA_DMA::Wait()
{
    return OpStatus::NotImplemented;
}

void VSPA_DMA::BufferOwnership(uint16_t index, DataTransferDirection bufferDirection)
{
}

std::vector<IDMA::Buffer> VSPA_DMA::GetBuffers() const
{
    return mappings;
}

std::string VSPA_DMA::GetName() const
{
    return "VSPA_DMA";
}

OpStatus VSPA_DMA::SubmitTransfer(uint32_t index, size_t size, uint32_t timestamp, uint32_t flags)
{
    hif->la9310_mem_address = dma_window.ep_pa + index * blockSize;
    ;
    hif->size = size;
    hif->timestamp = timestamp;
    hif->flags = flags;

    const mbox_opc_e command = MBOX_OPC_TX_DMA_SUBMIT;
    uint32_t loword = 0;
    uint32_t hiword = 0;
    hiword |= command << 24;
    uint64_t value = uint64_t(hiword) << 32 | loword;

    uint64_t response = 0;
    OpStatus status = mailbox->Message(vspa_cpu_id, vspa_mbox_id, value, &response);
    if (status != OpStatus::Success)
        return status;
    return (response & 0xFF) == 0 ? OpStatus::Success : static_cast<lime::OpStatus>(response & 0xFF);
}

} // namespace lime
