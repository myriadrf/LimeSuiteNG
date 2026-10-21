#include "IQStreamer.h"

#include "interface/IOversampler.h"
#include "interface/IDCCorrector.h"
#include "interface/IQuadratureErrorCorrector.h"
#include "interface/IToneGenerator.h"
#include "interface/IRFTiming.h"
#include "chips/LA9310/vspa/VSPA_mailbox.h"
#include "chips/LA9310/LA9310.h"
#include "chips/LA9310/firmware/LA9310_FW_Impl.h"
#include "chips/LA9310/firmware/IQStreamer_DMA.h"
#include "chips/LA9310/firmware/vspa_memorymap.h"
#include "chips/LA9310/firmware/iqplayer_commands.h"
#include "comms/PCIe/LA9310_PCIe.h"

#include "drivers/linux/la9310_limesdr/common_headers/la9310_host_if.h"

#include "m4_commands.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <iterator>
#include <string_view>
#include <string.h>

#ifndef M_PI // Windows
    #define M_PI 3.1415926535897932384626433832
#endif

using namespace std;

static const uint32_t vspa_cpu_id = 0;
static const uint32_t vspa_mbox_id = 0;

#define VSPA_CCSR 0x1000000

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
    // assert((size_t(dest) & 0x3) == 0);
    // assert((size_t(src) & 0x3) == 0);
    volatile uint32_t* dest32 = reinterpret_cast<volatile uint32_t*>(dest);
    volatile const uint32_t* src32 = reinterpret_cast<volatile const uint32_t*>(src);
    size_t wordsToCopy = bytes / sizeof(uint32_t);
    for (size_t w = 0; w < wordsToCopy; ++w)
    {
        // must copy word by word. memcpy could attempt to access more than 32bit at a time triggering BUS error.
        *dest32 = *src32;
        ++dest32;
        ++src32;
    };
    uint32_t bytes_tail = bytes % sizeof(uint32_t);
    for (uint32_t i = 0; i < bytes_tail; ++i)
        reinterpret_cast<volatile uint8_t*>(dest32)[i] = reinterpret_cast<const volatile uint8_t*>(src32)[i];
}

template<class PayloadT, class ResponseT>
OpStatus CallCommand(volatile la9310_sw_cmd_desc* m4_cmd_hif,
    M4_Command cmd,
    const PayloadT* payload,
    ResponseT* response,
    chrono::milliseconds timeout = chrono::milliseconds(1000))
{
    m4_cmd_hif->cmd = cmd;
    memcpy_w32(m4_cmd_hif->data, payload, sizeof(PayloadT));

    auto t1 = chrono::high_resolution_clock::now();
    m4_cmd_hif->status = LA9310_SW_CMD_STATUS_POSTED;
    while (m4_cmd_hif->status == LA9310_SW_CMD_STATUS_POSTED || m4_cmd_hif->status == LA9310_SW_CMD_STATUS_IN_PROGRESS)
    {
        auto t2 = chrono::high_resolution_clock::now();
        if (chrono::duration_cast<chrono::milliseconds>(t2 - t1) > timeout)
        {
            printf("M4: CallCommand timeout\n");
            break;
        }
    }

    if (m4_cmd_hif->status == LA9310_SW_CMD_STATUS_DONE)
    {
        memcpy_w32(response, m4_cmd_hif->data, sizeof(ResponseT));
        return OpStatus::Success;
    }

    return OpStatus::Error;
}

LA9310_IQStreamer::LA9310_IQStreamer(std::shared_ptr<LA9310_FW_Impl> fw)
    : fw(fw)
    , tdd_control(nullptr)
    , m4_cmd_hif(nullptr)
{
    if (!fw->IsVSPAFirmwareLoaded())
        return;

    m4_cmd_hif = reinterpret_cast<volatile la9310_sw_cmd_desc*>(fw->GetHIF(M4_MMAP_COMMAND_HIF));

    volatile vspa_regs* vspa_ccsr =
        reinterpret_cast<volatile vspa_regs*>(uint64_t(fw->pcie->GetBar(LA9310_WINDOW_BAR0).vaddr) + VSPA_CCSR);
    for (int i = 0; i < 4; ++i)
    {
        volatile vspa_dma_hif_t* ptr = reinterpret_cast<volatile vspa_dma_hif_t*>(fw->GetVSPAHIF(VSPA_MMAP_RXDMA_LANE0 + i));
        // optional, older firmware does not provide output blocks
        volatile dma_tcd_fifo_t* output_blocks =
            reinterpret_cast<volatile dma_tcd_fifo_t*>(fw->GetVSPAHIF(VSPA_MMAP_RXDDR_OUTPUT_LANE0 + i));
        if (ptr)
            rx_dma[i] = std::make_shared<IQStreamer_DMA>(IQStreamer_DMA::DMA_FROM_DEVICE, ptr, vspa_ccsr, fw->pcie, output_blocks);
    }
    for (int i = 0; i < 1; ++i)
    {
        volatile vspa_dma_hif_t* ptr = reinterpret_cast<volatile vspa_dma_hif_t*>(fw->GetVSPAHIF(VSPA_MMAP_TXDMA_LANE0));
        if (ptr)
            tx_dma = std::make_shared<IQStreamer_DMA>(IQStreamer_DMA::DMA_TO_DEVICE, ptr, vspa_ccsr, fw->pcie);
    }
    tdd_control = reinterpret_cast<volatile tx_tdd_config_t*>(fw->GetVSPAHIF(VSPA_MMAP_TX_TDD_CONFIG));
}

OpStatus LA9310_IQStreamer::PipelineEnable(uint32_t rxmask, uint32_t txmask, bool enable)
{
    if (!m4_cmd_hif)
        return OpStatus::NotImplemented;

    iqstream_control_payload payload;
    payload.enable = enable ? 1 : 0;
    payload.rxmask = rxmask;
    payload.txmask = txmask;

    simple_response_payload response;

    OpStatus status = CallCommand(m4_cmd_hif, LIME_M4_IQSTREAM_CTRL, &payload, &response);
    if (status != OpStatus::Success)
        return status;

    return response.status == 0 ? OpStatus::Success : OpStatus::Error;
}

OpStatus LA9310_IQStreamer::SetPipelineChannel(lime::TRXDir dir, uint32_t pipe, uint32_t channel)
{
    // uint32_t hiword = MBOX_OPC_RX_CHAN_SELECT << 24;
    // uint32_t loword = pipe & 0xFF;
    // loword |= (channel & 0xFF) << 8;

    // uint64_t value = (uint64_t(hiword) << 32) | loword;
    // return fw->mailbox->Message(vspa_cpu_id, vspa_mbox_id, value);

    if (!m4_cmd_hif)
        return OpStatus::NotImplemented;

    iqstream_channel_select payload;
    payload.lane = pipe;
    payload.channel = channel;

    simple_response_payload response;

    OpStatus status = CallCommand(m4_cmd_hif, LIME_M4_RX_CHANNEL_SELECT, &payload, &response);
    if (status != OpStatus::Success)
        return status;

    return response.status == 0 ? OpStatus::Success : OpStatus::Error;
}

OpStatus LA9310_IQStreamer::SetPipelineSampleFormat(lime::TRXDir dir, uint32_t pipe, DataFormat format)
{
    iq_sample_format_e vspa_format;
    switch (format)
    {
    case DataFormat::I16:
        vspa_format = IQ_SAMPLE_FORMAT_I16;
        break;
    case DataFormat::I12:
        vspa_format = IQ_SAMPLE_FORMAT_I12;
        break;
    default:
        return OpStatus::NotSupported;
    }

    uint32_t hiword = MBOX_OPC_SET_SAMPLE_FORMAT << 24;
    uint32_t loword = pipe & 0xFF;
    loword |= (vspa_format & 0xFF) << 8;
    if (dir == TRXDir::Tx)
        loword |= 1 << 16;

    uint64_t value = (uint64_t(hiword) << 32) | loword;
    uint64_t response = 0;
    OpStatus status = fw->mailbox->Message(vspa_cpu_id, vspa_mbox_id, value, &response);
    if (status != OpStatus::Success)
        return status;

    return response == 0 ? OpStatus::Success : OpStatus::Error;
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

class LA9310_Oversampler : public IOversampler
{
  public:
    LA9310_Oversampler(volatile struct la9310_sw_cmd_desc* m4_cmd_hif, TRXDir dir, uint8_t channel)
        : m4_cmd_hif(m4_cmd_hif)
        , dir(dir)
        , channel(channel)
    {
    }

    virtual ~LA9310_Oversampler(){};
    OpStatus SetOversample(uint32_t oversample_pow2) override
    {
        if (!m4_cmd_hif)
            return OpStatus::NotImplemented;

        iqstream_channel_config payload;
        payload.channel = channel;
        payload.oversample_pow2 = oversample_pow2;

        simple_response_payload response;

        M4_Command cmd = dir == TRXDir::Rx ? LIME_M4_RX_SET_OVERSAMPLE : LIME_M4_TX_SET_OVERSAMPLE;
        OpStatus status = CallCommand(m4_cmd_hif, cmd, &payload, &response);
        if (status != OpStatus::Success)
            return status;

        return response.status == 0 ? OpStatus::Success : OpStatus::Error;
    }
    uint32_t GetOversample() override
    {
        if (!m4_cmd_hif)
            return 0;

        iqstream_channel_config payload;
        payload.channel = channel;
        payload.oversample_pow2 = 0;

        uint32_t oversample_pow2 = 0;

        M4_Command cmd = dir == TRXDir::Rx ? LIME_M4_RX_GET_OVERSAMPLE : LIME_M4_TX_GET_OVERSAMPLE;
        OpStatus status = CallCommand(m4_cmd_hif, cmd, &payload, &oversample_pow2);
        // older firmware does not support readback, assume no oversampling
        if (status != OpStatus::Success)
            return 0;

        return oversample_pow2;
    }

  private:
    volatile struct la9310_sw_cmd_desc* m4_cmd_hif;
    TRXDir dir;
    uint8_t channel;
};

std::shared_ptr<IOversampler> LA9310_IQStreamer::GetOversampler(TRXDir dir, uint32_t channel)
{
    return std::make_shared<LA9310_Oversampler>(m4_cmd_hif, dir, channel);
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
        status = mailbox->Message(vspa_cpu_id, vspa_mbox_id, value);
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

class VSPA_Tone : public IToneGenerator
{
  private:
    typedef struct ToneState {
        float amplitude;
        uint32_t phase;
        uint32_t freq_bin;
    } tone_state_t;

  public:
    VSPA_Tone(std::shared_ptr<VSPA_mailbox> mailbox, lime::TRXDir dir, volatile void* hif)
        : mailbox(mailbox)
        , dir(dir)
    {
        state = reinterpret_cast<volatile tone_state_t*>(hif);
    }

    virtual ~VSPA_Tone() {}

    virtual OpStatus Enabled(bool enabled)
    {
        uint64_t value = 0;

        uint32_t hiword = MBOX_OPC_SINGLE_TONE_TX << 24;
        uint32_t loword = 0;
        loword |= enabled ? (1 << 16) : 0;

        value = (uint64_t(hiword) << 32) | loword;
        OpStatus status = mailbox->Message(vspa_cpu_id, vspa_mbox_id, value);
        return status;
    }

    virtual OpStatus SetParameters(double amplitude, uint32_t fftbin)
    {
        if (!state)
            return OpStatus::NotSupported;

        state->amplitude = amplitude;
        state->freq_bin = fftbin;

        return OpStatus::Success;
    }

  private:
    std::shared_ptr<VSPA_mailbox> mailbox;
    volatile tone_state_t* state;
    const lime::TRXDir dir;
};

std::shared_ptr<IToneGenerator> LA9310_IQStreamer::GetTxToneGenerator(uint32_t pipeline)
{
    volatile void* tone_hif = fw->GetVSPAHIF(VSPA_MMAP_TX_TONE_LANE0);
    if (!tone_hif)
        return nullptr;

    return std::make_shared<VSPA_Tone>(fw->mailbox, TRXDir::Tx, tone_hif);
}

// Tx burst timing of the VSPA PHY timer comparators. Offsets are in PHY timer ticks relative to the burst start/end.
// The VSPA reads the values each time it schedules a burst edge, so changes apply from the next burst on.
class VSPA_TxRFTiming : public IRFTiming
{
  public:
    VSPA_TxRFTiming(LA9310_IQStreamer* iqstreamer)
        : iqstreamer(iqstreamer)
    {
    }

    TRXDir GetDirection() const override { return TRXDir::Tx; }

    std::vector<ControlDescriptor> GetControls() const override
    {
        const std::vector<std::string> paTriggers(std::begin(paTriggerNames), std::end(paTriggerNames));
        const std::vector<std::string> rfSwitchTriggers(std::begin(rfSwitchTriggerNames), std::end(rfSwitchTriggerNames));
        return {
            { "dac"s, "DAC DMA allowed (CH5_TX_ALLOWED)"s, lime::Range<int32_t>(INT16_MIN, INT16_MAX), true, false, {} },
            { "pa"s, "PA enable (RFCTL_5)"s, lime::Range<int32_t>(INT8_MIN, INT8_MAX), false, true, paTriggers },
            { "rf_switch"s,
                "Tx band RF switch (RFCTL_0)"s,
                lime::Range<int32_t>(INT8_MIN, INT8_MAX),
                false,
                true,
                rfSwitchTriggers },
        };
    }

    OpStatus GetControl(const std::string& name, Control& control) override
    {
        volatile tx_tdd_config_t* tdd = iqstreamer->tdd_control;
        if (!tdd)
            return OpStatus::NotSupported;

        if (name == "dac"s)
        {
            control.start = { tdd->dac_allowed, "high"s };
            control.end = { tdd->dac_allowed, "low"s };
        }
        else if (name == "pa"s)
        {
            control.start = { tdd->pa_on, ToTrigger(paTriggerNames, tdd->pa_on_comparator_out) };
            control.end = { tdd->pa_off, ToTrigger(paTriggerNames, tdd->pa_off_comparator_out) };
        }
        else if (name == "rf_switch"s)
        {
            control.start = { tdd->rf_sw_on, ToTrigger(rfSwitchTriggerNames, tdd->rf_sw_on_comparator_out) };
            control.end = { tdd->rf_sw_off, ToTrigger(rfSwitchTriggerNames, tdd->rf_sw_off_comparator_out) };
        }
        else
            return OpStatus::InvalidValue;
        return OpStatus::Success;
    }

    OpStatus SetControl(const std::string& name, const Control& control) override
    {
        volatile tx_tdd_config_t* tdd = iqstreamer->tdd_control;
        if (!tdd)
            return OpStatus::NotSupported;

        if (name == "dac"s)
        {
            if (control.start.offset != control.end.offset || !InRange(control.start.offset, INT16_MIN, INT16_MAX))
                return OpStatus::OutOfRange;
            tdd->dac_allowed = control.start.offset;
            return OpStatus::Success;
        }

        if (name != "pa"s && name != "rf_switch"s)
            return OpStatus::InvalidValue;
        if (!InRange(control.start.offset, INT8_MIN, INT8_MAX) || !InRange(control.end.offset, INT8_MIN, INT8_MAX))
            return OpStatus::OutOfRange;
        uint16_t startOut, endOut;
        const TriggerNames& names = name == "pa"s ? paTriggerNames : rfSwitchTriggerNames;
        if (!ParseTrigger(names, control.start.trigger, startOut) || !ParseTrigger(names, control.end.trigger, endOut))
            return OpStatus::InvalidValue;

        if (name == "pa"s)
        {
            tdd->pa_on = control.start.offset;
            tdd->pa_off = control.end.offset;
            tdd->pa_on_comparator_out = startOut;
            tdd->pa_off_comparator_out = endOut;
        }
        else
        {
            tdd->rf_sw_on = control.start.offset;
            tdd->rf_sw_off = control.end.offset;
            tdd->rf_sw_on_comparator_out = startOut;
            tdd->rf_sw_off_comparator_out = endOut;
        }
        return OpStatus::Success;
    }

  private:
    static bool InRange(int32_t value, int32_t min, int32_t max) { return value >= min && value <= max; }

    // Indexed by VSPA ePhyTimerComparatorTrigger: 0 = no change, 1 = output 0, 2 = output 1, 3 = toggle
    using TriggerNames = const char* const[4];
    static constexpr TriggerNames paTriggerNames = { "nochange", "low", "high", "toggle" };
    // RFCTL_0 drives the Tx band switch: output 1 selects Band1, output 0 selects Band2
    static constexpr TriggerNames rfSwitchTriggerNames = { "nochange", "band2", "band1", "toggle" };

    static bool ParseTrigger(const TriggerNames& triggerNames, const std::string& text, uint16_t& comparatorOut)
    {
        for (uint16_t i = 0; i < std::size(triggerNames); ++i)
        {
            const std::string_view name(triggerNames[i]);
            const auto equalNoCase = [](unsigned char a, unsigned char b) { return std::tolower(a) == std::tolower(b); };
            if (std::equal(text.begin(), text.end(), name.begin(), name.end(), equalNoCase))
            {
                comparatorOut = i;
                return true;
            }
        }
        return false;
    }

    static std::string ToTrigger(const TriggerNames& triggerNames, uint16_t comparatorOut)
    {
        return comparatorOut < std::size(triggerNames) ? triggerNames[comparatorOut] : triggerNames[0];
    }

    LA9310_IQStreamer* iqstreamer;
};

std::shared_ptr<IRFTiming> LA9310_IQStreamer::GetTxRFTiming()
{
    if (!txRFTiming)
        txRFTiming = std::make_shared<VSPA_TxRFTiming>(this);
    return txRFTiming;
}

const complex32f_t* LA9310_IQStreamer::CalcFFT(uint32_t channel)
{
    uint64_t value = 0;
    uint32_t hiword = MBOX_OPC_RX_FFT << 24;
    uint32_t loword = 2; // channel

    value = (uint64_t(hiword) << 32) | loword;
    uint64_t response = 0;
    OpStatus status = fw->mailbox->Message(vspa_cpu_id, vspa_mbox_id, value, &response);
    if (status != OpStatus::Success)
        return nullptr;

    const uint32_t data_addr = (response >> 32);
    uint8_t* vspa_base_addr = reinterpret_cast<uint8_t*>(fw->pcie->GetBar(LA9310_WINDOW_BAR2).vaddr) + 0x400000;
    return reinterpret_cast<complex32f_t*>(vspa_base_addr + data_addr);
}

std::vector<uint32_t> LA9310_IQStreamer::CaptureADC(uint32_t channel)
{
    uint64_t value = 0;
    uint32_t hiword = MBOX_OPC_ADC_CAPTURE << 24;
    uint32_t loword = 2; // channel

    std::vector<uint32_t> chf_samples(512); // complex half fixed precision
    memset(chf_samples.data(), 0, chf_samples.size() * sizeof(chf_samples[0]));

    value = (uint64_t(hiword) << 32) | loword;
    uint64_t response = 0;
    OpStatus status = fw->mailbox->Message(vspa_cpu_id, vspa_mbox_id, value, &response);
    if (status != OpStatus::Success)
        return chf_samples;

    const uint32_t data_addr = (response >> 32);
    uint32_t* vspa_base_addr =
        reinterpret_cast<uint32_t*>(size_t(fw->pcie->GetBar(LA9310_WINDOW_BAR2).vaddr) + 0x400000 + data_addr);
    for (uint32_t i = 0; i < chf_samples.size(); ++i)
        chf_samples[i] = vspa_base_addr[i];
    return chf_samples;
}

OpStatus LA9310_IQStreamer::SetProxyDataBuffer(DMA_Buffer buffer)
{
    uint64_t value = 0;
    uint32_t hiword = MBOX_OPC_PROXY_OFFSET << 24;
    value = (uint64_t(hiword) << 32) | buffer.endpoint_pa();
    uint64_t response = 0;
    OpStatus status = fw->mailbox->Message(vspa_cpu_id, vspa_mbox_id, value, &response);
    if (status != OpStatus::Success)
        return status;

    return response == 0 ? OpStatus::Success : OpStatus::Error;
}

} // namespace lime
