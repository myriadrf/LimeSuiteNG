#ifndef LIME_VSPA_IQPLAYER_H
#define LIME_VSPA_IQPLAYER_H

#include <memory>
#include <mutex>

#include "limesuiteng/OpStatus.h"
#include "limesuiteng/complex.h"
#include "limesuiteng/config.h"

#include "VSPA_mailbox.h"
#include "vspa_state.h"
#include "VSPA_DMA.h"

namespace lime {

class IDCCorrector;
class IQuadratureErrorCorrector;
class LA9310_PCIe;
class VSPA_Trace;

typedef enum {
    MBOX_OPC_EMPTY_0, // 0x0
    MBOX_OPC_SINGLE_TONE_TX, // 0x1
    MBOX_OPC_SINGLE_TONE_RX, // 0x2
    MBOX_OPC_DCOC, // 0x3
    MBOX_OPC_BW_CAL, // 0x4
    MBOX_OPC_IQ_MOD_TX, // 0x5
    MBOX_OPC_IQ_MOD_RX, // 0x6
    MBOX_OPC_MSI, // 0x7
    MBOX_OPC_IQ_CORR, // 0x8
    MBOX_OPC_EMPTY_1, // 0x9
    MBOX_OPC_EMPTY_2, // 0xA
    MBOX_OPC_TX_DCO_CORR, // 0xB
    MBOX_OPC_OVERLAY_BASE, // 0xC
    MBOX_OPC_RX_CHAN_SELECT, // 0xD
    MBOX_OPC_RX_DCO_CORR, // 0xE
    MBOX_OPC_GET_STATS_COUNT, // 0xF
    MBOX_OPC_DONE_SWRESET, // 0x10
    MBOX_OPC_PROXY_OFFSET, // 0x11

    MBOX_OPC_TX_AXIQ, // 0x12
    MBOX_OPC_TX_PTR_RST, // 0x13

    MBOX_OPC_TX_HOST_FIFO_CONFIG,
    MBOX_OPC_TX_CONFIGURE,
    MBOX_OPC_TX_CONTROL,
    MBOX_OPC_TX_BURST_LENGTH,

    MBOX_OPC_RX_HOST_FIFO_CONFIG,
    MBOX_OPC_RX_CONFIGURE,
    MBOX_OPC_RX_CONTROL,
    MBOX_OPC_RX_BURST_LENGTH,
    MBOX_OPC_RX_PREPARE,

    MBOX_OPC_GET_FEATURES_MAP,
    MBOX_OPC_TX_DMA_SUBMIT,
    MBOX_OPC_RX_DMA_SUBMIT,
} mbox_opc_e;

struct VSPA_FIFO_State {
    uint64_t bytes_consumed{ 0 }; // software counter, Bytes copied from modem rx Fifo
    uint64_t bytes_produced{ 0 }; // software counter, Bytes received in modem rx Fifo
    uint32_t last_consumed{ 0 }; // firmware counter, overflowing
    uint32_t last_produced{ 0 }; // firmware counter, overflowing
    uint32_t fifo_start_addr{ 0 }; // fifo start address offset in iqflood region
    uint32_t fifo_size{ 0 };
    uint32_t fifo_offset{ 0 };
};

typedef enum {
    VSPA_RO0,
    VSPA_RO1,
    VSPA_RX0,
    VSPA_RX1,
} e_rx_channel;

class LIME_API VSPA_iqplayer
{
  public:
    static e_rx_channel api_channel_remap(uint32_t index);
    VSPA_iqplayer(std::shared_ptr<LA9310_PCIe> port);
    OpStatus Initialize();

    OpStatus EnableRxChannels(uint32_t channel_mask);
    /// @brief Resets the VCPU core if it is running
    /// @return 0 on success, error code else
    OpStatus ResetVCPU();
    bool IsFirmwareLoaded() const;

    OpStatus GenerateTxTone(bool enabled, int fftBin = 0);

    OpStatus SetupResources(uint32_t rxMask, uint32_t txCount);

    int32_t Receive(uint32_t channel, uint32_t* destination, uint32_t read_size, uint64_t* timestamp);

    std::shared_ptr<IDCCorrector> GetRxDCCorrector();
    std::shared_ptr<IDCCorrector> GetTxDCCorrector();
    std::shared_ptr<IQuadratureErrorCorrector> GetRxQEC();
    std::shared_ptr<IQuadratureErrorCorrector> GetTxQEC();
    OpStatus SetDCOffset(complex16_t offset);

    OpStatus RxEnable(uint8_t channel, bool enable, bool reset_pipeline = true);
    OpStatus TxEnable(bool enable, bool flow_control_disable = false);
    OpStatus SetupRx(uint32_t chan, uint32_t fifo_start_offset, uint32_t fifo_size);

    int GetDecimation(uint32_t channel) const;

    OpStatus SetDecimation(uint32_t channel, uint32_t decimation);
    OpStatus SetInterpolation(uint32_t interpolation);
    int GetInterpolation() const;

    OpStatus PrepareRx();

    VSPA_DMA* GetTxDMA() { return tx_dma.get(); };

    std::shared_ptr<VSPA_Trace> tracer;
    uint32_t GetRxOverruns(uint32_t ch);
    uint32_t GetTxUnderruns();

    bool IsClockEnabled() const;
    bool IsVCPUBusy() const;
    void HostToVCPU_Flag(uint32_t mask);

  private:
    void DisableGOtriggers();
    void ScanFeatures();
    std::shared_ptr<LA9310_PCIe> port;
    std::shared_ptr<VSPA_mailbox> mailbox;
    std::shared_ptr<VSPA_DMA> tx_dma;

    VSPA_FIFO_State mRx[4];

    uint8_t* vl_iqflood_ddr_addr;
    size_t iqflood_size;
    volatile const vspa_state_t* vspa_dmem_proxy_ro = NULL;
    volatile vspa_state_t* vspa_dmem_proxy_wo = NULL;

    uint32_t rx_fifo_start_offset_in_iqflood;
    uint8_t rx_dma_channel_count;

    std::mutex mx;
};

} // namespace lime

#endif // LIME_VSPA_IQPLAYER_H
