#ifndef LIME_VSPA_IQPLAYER_H
#define LIME_VSPA_IQPLAYER_H

#include <memory>

#include "limesuiteng/OpStatus.h"

#include "VSPA_mailbox.h"

namespace lime {

class LA9310_PCIe;

class VSPA_iqplayer
{
  public:
    VSPA_iqplayer(std::shared_ptr<LA9310_PCIe> port);

    OpStatus SelectRxChannel(uint32_t rx_channel_index);
    OpStatus StartRx(uint32_t fifo_size, uint32_t fifo_base_la9310_phys_addr);
    OpStatus StopRx();
    OpStatus StopTx();

  private:
    std::shared_ptr<LA9310_PCIe> port;
    VSPA_mailbox mailbox;
};

} // namespace lime

#endif // LIME_VSPA_IQPLAYER_H