#ifndef LIME_VSPA_IQPLAYER_H
#define LIME_VSPA_IQPLAYER_H

#include <memory>
#include <mutex>

#include "limesuiteng/OpStatus.h"
#include "limesuiteng/complex.h"
#include "limesuiteng/config.h"

#include "vspa_dmem_proxy.h"
#include "VSPA_mailbox.h"

namespace lime {

class IDCCorrector;
class IQuadratureErrorCorrector;
class LA9310_PCIe;

struct VSPA_FIFO_State {
    uint64_t bytes_consumed{ 0 }; // software counter, Bytes copied from modem rx Fifo
    uint64_t bytes_produced{ 0 }; // software counter, Bytes received in modem rx Fifo
    uint32_t last_consumed{ 0 }; // firmware counter, overflowing
    uint32_t last_produced{ 0 }; // firmware counter, overflowing
    uint32_t fifo_start_addr{ 0 }; // fifo start address offset in iqflood region
    uint32_t fifo_size{ 0 };
    uint32_t fifo_offset{ 0 };
};

class LIME_API VSPA_iqplayer
{
  public:
    VSPA_iqplayer(std::shared_ptr<LA9310_PCIe> port);
    OpStatus Initialize();

    OpStatus SelectRxChannel(uint32_t rx_channel_index);
    OpStatus StartRx();
    OpStatus StartTx();
    OpStatus StopRx();
    OpStatus StopTx();
    /// @brief Resets the VCPU core if it is running
    /// @return 0 on success, error code else
    OpStatus ResetVCPU();
    bool IsFirmwareLoaded() const;

    OpStatus StartTxTone(bool enabled, int fftBin);
    OpStatus GenerateTxTone(bool enabled, int fftBin = 0);

    OpStatus Setup(uint32_t rxCount, uint32_t txCount, double expectedTxDataRate);

    int32_t Receive(uint32_t channel, uint32_t* destination, uint32_t read_size, uint64_t* timestamp);
    int32_t Transmit(const void* src, uint32_t write_size, uint64_t timestamp);

    OpStatus ClearStats();

    std::shared_ptr<IDCCorrector> GetRxDCCorrector();
    std::shared_ptr<IDCCorrector> GetTxDCCorrector();
    std::shared_ptr<IQuadratureErrorCorrector> GetRxQEC();
    std::shared_ptr<IQuadratureErrorCorrector> GetTxQEC();
    OpStatus SetDCOffset(complex16_t offset);

    t_stats GetStats();

    OpStatus StartRx(uint8_t channel, uint32_t fifo_size);
    OpStatus StartTx(uint32_t fifo_size, bool flow_control);
    OpStatus SetupRx(uint32_t chan, uint32_t fifo_start_offset, uint32_t fifo_size, double expectedDataRate = 0);
    OpStatus SetupTx(uint32_t fifo_start_offset, uint32_t fifo_size, double expectedTxDataRate = 0);

    int GetDecimation() const;

  private:
    std::shared_ptr<LA9310_PCIe> port;
    std::shared_ptr<VSPA_mailbox> mailbox;

    VSPA_FIFO_State mRx[4];
    VSPA_FIFO_State mTx;

    uint8_t* vl_iqflood_ddr_addr;
    size_t iqflood_size;
    volatile t_vspa_dmem_proxy* v_vspa_dmem_proxy_ro = NULL;
    volatile t_tx_ch_host_proxy* tx_vspa_proxy_ro = NULL;
    volatile t_rx_ch_host_proxy* rx_vspa_proxy_ro = NULL;

    volatile t_tx_ch_host_proxy* tx_vspa_proxy_wo = NULL;
    volatile t_tx_ch_host_proxy* rx_vspa_proxy_wo = NULL;
    volatile t_stats* app_stats = nullptr;

    uint32_t rx_fifo_start_offset_in_iqflood;
    uint8_t tx_dma_channel_count;
    uint8_t rx_dma_channel_count;

    std::mutex mx;
};

} // namespace lime

#endif // LIME_VSPA_IQPLAYER_H
