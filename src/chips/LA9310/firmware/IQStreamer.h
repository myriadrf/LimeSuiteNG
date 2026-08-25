#ifndef LIME_LA9310_IQPLAYER_H
#define LIME_LA9310_IQPLAYER_H

#include <memory>
#include <mutex>

#include "limesuiteng/OpStatus.h"
#include "limesuiteng/complex.h"
#include "limesuiteng/config.h"

#include "drivers/linux/la9310_limesdr/common_headers/la9310_host_if.h"

namespace lime {

class IDCCorrector;
class IQuadratureErrorCorrector;
class LA9310_PCIe;
class LA9310_FW_Impl;
class IQStreamer_DMA;

typedef enum {
    VSPA_RO0,
    VSPA_RO1,
    VSPA_RX0,
    VSPA_RX1,
} e_rx_channel;

class LIME_API LA9310_IQStreamer
{
  public:
    LA9310_IQStreamer(std::shared_ptr<LA9310_FW_Impl> fw);

    OpStatus StreamEnable(uint32_t rxmask, uint32_t txmask, bool enable);

    int GetDecimation(uint32_t channel) const;
    int GetInterpolation() const;

    std::shared_ptr<IDCCorrector> GetRxDCCorrector(uint32_t pipeline);
    std::shared_ptr<IDCCorrector> GetTxDCCorrector(uint32_t pipeline);
    std::shared_ptr<IQuadratureErrorCorrector> GetRxQEC(uint32_t pipeline);
    std::shared_ptr<IQuadratureErrorCorrector> GetTxQEC(uint32_t pipeline);

    OpStatus SetDecimation(uint32_t channel, uint32_t decimation);
    OpStatus SetInterpolation(uint32_t interpolation);

    void HostToVCPU_Flag(uint32_t mask);

    uint64_t GetHardwareTimestamp();

    std::shared_ptr<IQStreamer_DMA> rx_dma[4];
    std::shared_ptr<IQStreamer_DMA> tx_dma;
    std::shared_ptr<LA9310_FW_Impl> fw;

  private:
    volatile struct la9310_sw_cmd_desc* cmd_hif;
};

} // namespace lime

#endif // LIME_LA9310_IQPLAYER_H
