#pragma once

#include "l1-trace.h"
#include "limesuiteng/config.h"

#include <memory>
#include <vector>

namespace lime {

class LA9310_PCIe;

class LIME_API VSPA_Trace
{
  public:
    VSPA_Trace(std::shared_ptr<LA9310_PCIe> port, void* hif);

    void Clear();
    std::vector<l1_trace_data_t> ReadTrace();

  private:
    std::shared_ptr<LA9310_PCIe> port;
    l1_trace_hif_t* hif;
    l1_trace_data_t* events_va;
    uint32_t bytes_read;
    uint32_t buffer_size;
    uint32_t last_drops;
};

} // namespace lime