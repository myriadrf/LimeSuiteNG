#ifndef LIME_LA9310_H
#define LIME_LA9310_H

#include <string>
#include <memory>
#include <span>

#include "PHYTimer.h"

#include "chips/EEPROM/EEPROM_I2C.h"
#include "comms/DMA_Buffer.h"

struct la9310_hif;

namespace lime {

class LA9310_PCIe;
class LA9310_I2C;
class VSPA_mailbox;

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

/// LA9310 hardware components representation
class LIME_API LA9310
{
  private:
    std::shared_ptr<LA9310_PCIe> pcie;
    std::shared_ptr<LA9310_I2C> i2c;

  public:
    LA9310(std::shared_ptr<LA9310_PCIe> port);
    ~LA9310();

    void GetADCDACRates(uint8_t* adc_rate_mask, uint8_t* dac_rate_mask);
    bool IsM4CoreProgrammed();

    OpStatus LoadVSPAFirmware(std::span<const char> firmware);
    bool IsVSPAFirmwareLoaded() const;
    std::vector<DMA_Buffer> GetUserSpaceDMABuffers();

    PHYTimer phytimer;
    EEPROM_I2C eeprom;

    std::shared_ptr<VSPA_mailbox> mailbox;

    OpStatus ResetVCPU();

  private:
    bool IsClockEnabled() const;
    volatile struct la9310_hif* hif; // host interface
};

} // namespace lime

#endif // LIME_LA9310_H
