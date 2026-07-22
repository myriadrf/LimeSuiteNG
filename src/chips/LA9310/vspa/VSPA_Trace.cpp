#include "VSPA_Trace.h"

#include "comms/PCIe/LA9310_PCIe.h"

#include "drivers/linux/la9310_limesdr/common_headers/la9310_host_if.h"

#include <stdio.h>
#include <string.h>

namespace lime {

VSPA_Trace::VSPA_Trace(std::shared_ptr<LA9310_PCIe> port, void* hif)
    : port(port)
    , hif(reinterpret_cast<l1_trace_hif_t*>(hif))
    , events_va(nullptr)
    , bytes_read(0)
    , last_drops(0)
{
    auto scratch_buf_window = port->GetBar(LA9310_WINDOW_SCRATCH);
    buffer_size = 1024 * 1024 * 2;
    this->hif->buffer_size = buffer_size;

    const uint32_t offset = 1024 * 1024 * 2;
    this->hif->la9310_mem_address = PCI_OUTBOUND_WINDOW_BASE_ADDR + offset;

    events_va = reinterpret_cast<l1_trace_data_t*>(size_t(scratch_buf_window.vaddr) + offset);
    // memset(events_va, 0, buffer_size);
}

void VSPA_Trace::Clear()
{
    bytes_read = hif->bytes_produced;
    last_drops = hif->event_drops;
}

std::vector<l1_trace_data_t> VSPA_Trace::ReadTrace()
{
    std::vector<l1_trace_data_t> events;
    if (!events_va)
        return events;

    uint32_t bytes_available = (hif->bytes_produced - bytes_read);
    if (bytes_available > buffer_size)
    {
        bytes_available = bytes_available % buffer_size;
    }

    if (hif->event_drops != last_drops)
        printf("drop %i/%i/%i, %i\n", hif->event_drops, hif->event_count, hif->bytes_produced, bytes_read);
    last_drops = hif->event_drops;

    int events_made = hif->event_count;
    // printf("evt cnt: %i, %08X, %i, %i, %i \n", events_made, hif->la9310_mem_address, hif->buffer_size, hif->event_drops, hif->bytes_produced);

    if (!bytes_available)
        return events;
    printf("bytes_available: %i\n", bytes_available);

    const uint32_t bytes_to_end = buffer_size - (bytes_read % buffer_size);
    uint32_t contiguous_range = bytes_to_end < bytes_available ? bytes_to_end : bytes_available;

    const uint32_t events_available = (bytes_available) / sizeof(l1_trace_data_t);
    // assert(events_available > 0);
    if (events_available == 0)
        return events;
    events.resize(events_available);

    l1_trace_data_t* src = events_va + (bytes_read % buffer_size) / sizeof(l1_trace_data_t);
    memcpy(events.data(), src, contiguous_range);
    bytes_available -= contiguous_range;
    bytes_read += contiguous_range;

    if (bytes_available)
    {
        memcpy(&events[contiguous_range / sizeof(l1_trace_data_t)], events_va, bytes_available);
        bytes_read += bytes_available;
    }

    return events;
}

} // namespace lime
