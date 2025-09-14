#ifndef LIME_VSPA_IQPLAYER_H
#define LIME_VSPA_IQPLAYER_H

#include <memory>

#include "limesuiteng/OpStatus.h"

#include "vspa_dmem_proxy.h"
#include "VSPA_mailbox.h"

namespace lime {

class LA9310_PCIe;

struct VSPA_FIFO_State {
    uint32_t fifo_start_addr{ 0 }; // fifo start address offset in iqflood region
    uint32_t fifo_size{ 0 };
    uint32_t fifo_offset{ 0 };
    uint32_t bytes_consumed{ 0 }; // Bytes copied from modem rx Fifo
    uint32_t bytes_produced{ 0 }; // Bytes received in modem rx Fifo
};

class VSPA_iqplayer
{
  public:
    VSPA_iqplayer(std::shared_ptr<LA9310_PCIe> port);

    OpStatus SelectRxChannel(uint32_t rx_channel_index);
    OpStatus StartRx();
    OpStatus StopRx();
    OpStatus StopTx();

    OpStatus Setup(uint32_t rxCount, uint32_t txCount);

    int32_t Receive(uint32_t channel, uint32_t* destination, uint32_t read_size);
    int32_t Transmit(const uint32_t* src, uint32_t write_size);

  private:
    OpStatus StartRx(uint32_t fifo_size, uint32_t fifo_base_la9310_phys_addr);
    OpStatus SetupRx(uint32_t chan, uint32_t fifo_start_offset, uint32_t fifo_size);
    OpStatus SetupTx(uint32_t fifo_start_offset, uint32_t fifo_size);
    std::shared_ptr<LA9310_PCIe> port;
    VSPA_mailbox mailbox;

    VSPA_FIFO_State mRx[4];
    VSPA_FIFO_State mTx;

    uint8_t* vl_iqflood_ddr_addr;
    size_t iqflood_size;
    const volatile t_vspa_dmem_proxy* v_vspa_dmem_proxy_ro = NULL;
    const volatile t_tx_ch_host_proxy* tx_vspa_proxy_ro = NULL;
    const volatile t_rx_ch_host_proxy* rx_vspa_proxy_ro = NULL;

    volatile t_tx_ch_host_proxy* tx_vspa_proxy_wo = NULL;

    uint32_t rx_fifo_start_offset_in_iqflood;
};

} // namespace lime

#endif // LIME_VSPA_IQPLAYER_H