#ifndef LIME_LA9310_IQPLAYER_H
#define LIME_LA9310_IQPLAYER_H

#include <memory>
#include <mutex>
#include <vector>

#include "limesuiteng/OpStatus.h"
#include "limesuiteng/complex.h"
#include "limesuiteng/config.h"
#include "limesuiteng/types.h"

#include "drivers/linux/la9310_limesdr/common_headers/la9310_host_if.h"

namespace lime {

class IOversampler;
class IDCCorrector;
class IQuadratureErrorCorrector;
class IToneGenerator;
class LA9310_PCIe;
class LA9310_FW_Impl;
class IQStreamer_DMA;

typedef enum {
    VSPA_RO0,
    VSPA_RO1,
    VSPA_RX0,
    VSPA_RX1,
} e_rx_channel;

typedef struct TxTDD_Config {
    int16_t dac_allowed;
    int16_t pa_on : 8;
    int16_t pa_off : 8;
    int16_t rf_sw_on : 8;
    int16_t rf_sw_off : 8;
    uint16_t rf_sw_on_comparator_out : 2;
    uint16_t rf_sw_off_comparator_out : 2;
    uint16_t pa_on_comparator_out : 2;
    uint16_t pa_off_comparator_out : 2;
} tx_tdd_config_t;

class LIME_API LA9310_IQStreamer
{
  public:
    LA9310_IQStreamer(std::shared_ptr<LA9310_FW_Impl> fw);

    OpStatus PipelineEnable(uint32_t rxmask, uint32_t txmask, bool enable);

    OpStatus SetPipelineChannel(TRXDir dir, uint32_t pipe, uint32_t channel);

    int GetDecimation(uint32_t channel) const;
    int GetInterpolation() const;

    std::shared_ptr<IOversampler> GetOversampler(TRXDir dir, uint32_t channel);
    std::shared_ptr<IDCCorrector> GetRxDCCorrector(uint32_t pipeline);
    std::shared_ptr<IDCCorrector> GetTxDCCorrector(uint32_t pipeline);
    std::shared_ptr<IQuadratureErrorCorrector> GetRxQEC(uint32_t pipeline);
    std::shared_ptr<IQuadratureErrorCorrector> GetTxQEC(uint32_t pipeline);
    std::shared_ptr<IToneGenerator> GetTxToneGenerator(uint32_t pipeline);

    OpStatus SetDecimation(uint32_t channel, uint32_t decimation);
    OpStatus SetInterpolation(uint32_t interpolation);

    const complex32f_t* CalcFFT(uint32_t channel);
    std::vector<uint32_t> CaptureADC(uint32_t channel);

    void HostToVCPU_Flag(uint32_t mask);

    uint64_t GetHardwareTimestamp();

    std::shared_ptr<IQStreamer_DMA> rx_dma[4];
    std::shared_ptr<IQStreamer_DMA> tx_dma;
    std::shared_ptr<LA9310_FW_Impl> fw;

    volatile tx_tdd_config_t* tdd_control;

  private:
    volatile struct la9310_sw_cmd_desc* m4_cmd_hif;
};

} // namespace lime

#endif // LIME_LA9310_IQPLAYER_H
