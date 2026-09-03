#include "IQStreamer.h"

#include "interface/IOversampler.h"
#include "interface/IDCCorrector.h"
#include "interface/IQuadratureErrorCorrector.h"
#include "chips/LA9310/vspa/VSPA_mailbox.h"
#include "chips/LA9310/LA9310.h"
#include "chips/LA9310/firmware/LA9310_FW_Impl.h"
#include "chips/LA9310/firmware/IQStreamer_DMA.h"

#include "drivers/linux/la9310_limesdr/common_headers/la9310_host_if.h"

#include "m4_commands.h"

#include <chrono>
#include <string.h>

using namespace std;

static const uint32_t vspa_cpu_id = 0;
static const uint32_t vspa_mbox_id = 0;

enum {
    MBOX_EMPTY = 0, // 0x0
    MBOX_IQ_CORR_FTAP0, // 0x1
    MBOX_IQ_CORR_FTAP1, // 0x2
    MBOX_IQ_CORR_FTAP2, // 0x3
    MBOX_IQ_CORR_FTAP3, // 0x4
    MBOX_IQ_CORR_FTAP4, // 0x5
    MBOX_IQ_CORR_FTAP5, // 0x6
    MBOX_IQ_CORR_FTAP6, // 0x7
    MBOX_IQ_CORR_FTAP7, // 0x8
    MBOX_IQ_CORR_FTAP8, // 0x9
    MBOX_IQ_CORR_FTAP9, // 0xA
    MBOX_IQ_CORR_FTAP10, // 0xB
    MBOX_IQ_CORR_FTAP11, // 0xC
    MBOX_IQ_CORR_FTAP12, // 0xD
    MBOX_IQ_CORR_DC_I, // 0xE
    MBOX_IQ_CORR_DC_Q, // 0xF
    MBOX_IQ_CORR_FDELAY, // 0x10
    MBOX_IQ_CORR_MAX, // 0x11
};

namespace lime {

static void memcpy_w32(volatile void* dest, volatile const void* src, size_t bytes)
{
    // check alignment
    assert((size_t(dest) & 0x3) == 0);
    assert((size_t(src) & 0x3) == 0);
    volatile uint32_t* dest32 = reinterpret_cast<volatile uint32_t*>(dest);
    volatile const uint32_t* src32 = reinterpret_cast<volatile const uint32_t*>(src);
    size_t wordsToCopy = bytes / sizeof(uint32_t) + (bytes % sizeof(uint32_t) > 0 ? 1 : 0);
    for (size_t w = 0; w < wordsToCopy; ++w)
    {
        // must copy word by word. memcpy could attempt to access more than 32bit at a time triggering BUS error.
        dest32[w] = src32[w];
    };
}

template<class PayloadT, class ResponseT>
OpStatus CallCommand(volatile la9310_sw_cmd_desc* cmd_hif,
    M4_Command cmd,
    const PayloadT* payload,
    ResponseT* response,
    chrono::milliseconds timeout = chrono::milliseconds(1000))
{
    cmd_hif->cmd = cmd;
    memcpy_w32(cmd_hif->data, payload, sizeof(PayloadT));

    auto t1 = chrono::high_resolution_clock::now();
    cmd_hif->status = LA9310_SW_CMD_STATUS_POSTED;
    while (cmd_hif->status == LA9310_SW_CMD_STATUS_POSTED || cmd_hif->status == LA9310_SW_CMD_STATUS_IN_PROGRESS)
    {
        auto t2 = chrono::high_resolution_clock::now();
        if (chrono::duration_cast<chrono::milliseconds>(t2 - t1) > timeout)
        {
            printf("M4: CallCommand timeout\n");
            break;
        }
    }

    if (cmd_hif->status == LA9310_SW_CMD_STATUS_DONE)
    {
        memcpy_w32(response, cmd_hif->data, sizeof(ResponseT));
        return OpStatus::Success;
    }

    return OpStatus::Error;
}

LA9310_IQStreamer::LA9310_IQStreamer(std::shared_ptr<LA9310_FW_Impl> fw)
    : fw(fw)
    , cmd_hif(nullptr)
{
    if (!fw->IsVSPAFirmwareLoaded())
        return;

    cmd_hif = reinterpret_cast<volatile la9310_sw_cmd_desc*>(fw->GetHIF(M4_MMAP_COMMAND_HIF));

    for (int i = 0; i < 1; ++i)
    {
        volatile host_dma_hif_t* ptr = reinterpret_cast<volatile host_dma_hif_t*>(fw->GetHIF(M4_MMAP_IQPLAYER_RXPIPE0));
        if (ptr)
            rx_dma[i] = std::make_shared<IQStreamer_DMA>(IQStreamer_DMA::DMA_FROM_DEVICE, ptr, fw->pcie);
    }
    for (int i = 0; i < 1; ++i)
    {
        volatile host_dma_hif_t* ptr = reinterpret_cast<volatile host_dma_hif_t*>(fw->GetHIF(M4_MMAP_IQPLAYER_TXPIPE0));
        if (ptr)
            tx_dma = std::make_shared<IQStreamer_DMA>(IQStreamer_DMA::DMA_TO_DEVICE, ptr, fw->pcie);
    }
}

OpStatus LA9310_IQStreamer::PipelineEnable(uint32_t rxmask, uint32_t txmask, bool enable)
{
    if (!cmd_hif)
        return OpStatus::NotImplemented;

    iqstream_control_payload payload;
    payload.enable = enable ? 1 : 0;
    payload.rxmask = rxmask;
    payload.txmask = txmask;

    simple_response_payload response;

    OpStatus status = CallCommand(cmd_hif, LIME_M4_IQSTREAM_CTRL, &payload, &response);
    if (status != OpStatus::Success)
        return status;

    return response.status == 0 ? OpStatus::Success : OpStatus::Error;
}

OpStatus LA9310_IQStreamer::SetPipelineChannel(lime::TRXDir dir, uint32_t pipe, uint32_t channel)
{
    return OpStatus::NotImplemented;
}

int LA9310_IQStreamer::GetDecimation(uint32_t channel) const
{
    return 1;
}

int LA9310_IQStreamer::GetInterpolation() const
{
    return 1;
}

uint64_t LA9310_IQStreamer::GetHardwareTimestamp()
{
    return 0;
}

std::shared_ptr<IOversampler> LA9310_IQStreamer::GetOversampler(TRXDir dir, uint32_t channel)
{
    return nullptr;
}

class VSPA_DC_Offset : public IDCCorrector
{
  public:
    VSPA_DC_Offset(std::shared_ptr<VSPA_mailbox> mailbox, mbox_opc_e command)
        : mailbox(mailbox)
        , command(command)
    {
    }

    OpStatus Enabled(bool enable) { return enable ? OpStatus::Success : OpStatus::NotSupported; }

    OpStatus SetDCOffset(complex16_t offset) override
    {
        OpStatus status = SetDCI(offset.real());
        if (status != OpStatus::Success)
            return status;
        return SetDCQ(offset.imag());
    }

    OpStatus SetDCI(int16_t offset) override
    {
        uint32_t iq_channel_id = 0;
        bool iq_tx_rx = command == MBOX_OPC_TX_DCO_CORR;
        bool iq_rst = false;

        uint32_t hiword = MBOX_OPC_IQ_CORR << 24;
        hiword |= (iq_channel_id & 0x3) << 16;
        hiword |= iq_tx_rx << 21;
        hiword |= iq_rst << 20;

        uint32_t loword = 0;
        float fval = float(offset / 32768.0);
        memcpy(&loword, &fval, sizeof(uint32_t));
        uint64_t value = (uint64_t(hiword | (MBOX_IQ_CORR_DC_I & 0xFFFF)) << 32) | loword;
        return mailbox->Message(vspa_cpu_id, vspa_mbox_id, value);
    }
    OpStatus SetDCQ(int16_t offset) override
    {
        uint32_t iq_channel_id = 0;
        bool iq_tx_rx = command == MBOX_OPC_TX_DCO_CORR;
        bool iq_rst = false;

        uint32_t hiword = MBOX_OPC_IQ_CORR << 24;
        hiword |= (iq_channel_id & 0x3) << 16;
        hiword |= iq_tx_rx << 21;
        hiword |= iq_rst << 20;

        uint32_t loword = 0;
        float fval = float(offset / 32768.0);
        memcpy(&loword, &fval, sizeof(uint32_t));
        uint64_t value = (uint64_t(hiword | (MBOX_IQ_CORR_DC_Q & 0xFFFF)) << 32) | loword;
        return mailbox->Message(vspa_cpu_id, vspa_mbox_id, value);
    }

    complex16_t GetDCOffset() override { return complex16_t(0, 0); }

    lime::Range<float> GetRange() override { return lime::Range<float>(-16384, 16383, 1.0); }

    IDCCorrector::Type GetType() const { return IDCCorrector::Type::Digital; }

  private:
    std::shared_ptr<VSPA_mailbox> mailbox;
    const mbox_opc_e command;
};

std::shared_ptr<IDCCorrector> LA9310_IQStreamer::GetRxDCCorrector(uint32_t pipeline)
{
    return std::make_shared<VSPA_DC_Offset>(fw->mailbox, MBOX_OPC_RX_DCO_CORR);
}

std::shared_ptr<IDCCorrector> LA9310_IQStreamer::GetTxDCCorrector(uint32_t pipeline)
{
    return std::make_shared<VSPA_DC_Offset>(fw->mailbox, MBOX_OPC_TX_DCO_CORR);
}

class VSPA_QEC : public IQuadratureErrorCorrector
{
  public:
    VSPA_QEC(std::shared_ptr<VSPA_mailbox> mailbox, lime::TRXDir dir)
        : mailbox(mailbox)
        , dir(dir)
    {
    }

    virtual ~VSPA_QEC() {}
    OpStatus SetImbalance(float iq_gain_imb, float phase_imb_deg) override
    {
        float f1, f2, f4;
        if (dir == TRXDir::Rx)
        {
            float gamma = pow(10.0, (iq_gain_imb / 20.0));
            float theta_z = phase_imb_deg * (M_PI / 180.0);
            f1 = 1.0 / gamma;
            f2 = std::tan(theta_z) / gamma;
            f4 = 1.0 / std::cos(theta_z);
        }
        else
        {
            float alpha = pow(10.0, (iq_gain_imb / 20.0));
            float phi_z = phase_imb_deg * (M_PI / 180.0);

            f1 = 1.0 / std::cos(phi_z);
            f2 = -std::tan(phi_z) / alpha;
            f4 = 1.0 / alpha;
        }

        const uint32_t iq_channel_id = 0;
        const bool iq_tx_rx = dir == TRXDir::Tx;
        const bool iq_rst = false;

        uint32_t hiword = MBOX_OPC_IQ_CORR << 24;
        hiword |= (iq_channel_id & 0x3) << 16;
        hiword |= iq_tx_rx << 21;
        hiword |= iq_rst << 20;

        uint32_t loword = 0;
        uint64_t value = 0;

        memcpy(&loword, &f2, sizeof(uint32_t));
        value = (uint64_t(hiword | (MBOX_IQ_CORR_FTAP1 & 0xFFFF)) << 32) | loword;
        OpStatus status = mailbox->Message(vspa_cpu_id, vspa_mbox_id, value);
        if (status != OpStatus::Success)
            return status;

        memcpy(&loword, &f1, sizeof(uint32_t));
        value = (uint64_t(hiword | (MBOX_IQ_CORR_FTAP2 & 0xFFFF)) << 32) | loword;
        mailbox->Message(vspa_cpu_id, vspa_mbox_id, value);
        if (status != OpStatus::Success)
            return status;

        memcpy(&loword, &f4, sizeof(uint32_t));
        value = (uint64_t(hiword | (MBOX_IQ_CORR_FTAP3 & 0xFFFF)) << 32) | loword;
        return mailbox->Message(vspa_cpu_id, vspa_mbox_id, value);
    }

    lime::Range<float> GetGainRange() override { return lime::Range(-3.0f, 3.0f, 1.0f / 512); }

    lime::Range<float> GetPhaseRange() override { return lime::Range(-22.5f, 22.5f, 45.0f / 1024); }

    OpStatus SetPhaseCorrection(float phase_imb_deg) override { return OpStatus::NotImplemented; }
    OpStatus SetGainCorrection(float phase_imb_deg) override { return OpStatus::NotImplemented; }

  private:
    std::shared_ptr<VSPA_mailbox> mailbox;
    const lime::TRXDir dir;
};

std::shared_ptr<IQuadratureErrorCorrector> LA9310_IQStreamer::GetRxQEC(uint32_t pipeline)
{
    return std::make_shared<VSPA_QEC>(fw->mailbox, TRXDir::Rx);
}

std::shared_ptr<IQuadratureErrorCorrector> LA9310_IQStreamer::GetTxQEC(uint32_t pipeline)
{
    return std::make_shared<VSPA_QEC>(fw->mailbox, TRXDir::Tx);
}

} // namespace lime
