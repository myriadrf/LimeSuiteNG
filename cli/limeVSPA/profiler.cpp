#include "chips/LA9310/vspa/l1-trace.h"

#include <sstream>

extern std::string GetMsgName(uint32_t msg);

static const double clockRate = 30.72e6 * 4;
static const double tickDuration = 1 / clockRate;

enum class ePhase {
    Instant,
    Counter,
    Begin,
    End,
    Complete,
};

struct Event {
    std::string name;
    std::string category;
    ePhase phase;
    uint64_t timestamp;
    uint32_t pid;
    uint32_t tid;
    uint32_t id;
    uint32_t value;
};

static std::string ToString(ePhase phase)
{
    switch (phase)
    {
    case ePhase::Instant:
        return "I";
    case ePhase::Counter:
        return "C";
    case ePhase::Begin:
        return "B";
    case ePhase::End:
        return "E";
    case ePhase::Complete:
        return "X";
    default:
        return "";
    }
}

static std::string ToString(Event& evt)
{
    std::stringstream ss;
    ss << "{ " << "\"cat\": \"" << evt.category << "\""
       << ",\"ts\":"
       // << uint64_t(double(evt.timestamp * tickDuration) * 1e6)
       << uint64_t(evt.timestamp) << ",\"pid\":" << evt.pid << ",\"tid\":" << evt.tid << ",\"ph\":" << "\"" << ToString(evt.phase)
       << "\"" << ",\"name\": \"" << evt.name << "\"";
    if (evt.id)
        ss << ",\"id\":" << evt.id;
    if (evt.phase == ePhase::Counter)
        ss << ", \"args\": {\"counter\": " << evt.value << "}";
    else if (evt.phase == ePhase::Complete)
        ss << ", \"dur\": " << evt.value;
    ss << "}";
    return ss.str();
}

static std::string NameWithTag(const std::string& name, uint32_t tag)
{
    char ctemp[64];
    sprintf(ctemp, "%s:%X", name.c_str(), tag);
    return ctemp;
}

static std::string OpName(uint32_t op)
{
    switch (op)
    {
    case T_XFER_BUFFER:
        return "DMA";
    case T_QEC_TX_BUFFER:
        return "TX_QEC";
    case T_QEC_RX_BUFFER:
        return "RX_QEC";
    case T_DEC_BUFFER:
        return "DEC";
    case T_INT_BUFFER:
        return "INT";
    case T_UNDERRUN:
        return "UDR";
    case T_OVERRUN:
        return "OVR";
    case T_UNEXPECTED:
        return "EXCEPTION";
    case T_NO_MEMORY:
        return "NO_MEMORY";
    case T_AXIQ_ENQ:
        return "AXIQ_ENQ";
    case T_DDR_ENQ:
        return "DDR_ENQ";
    case T_GO:
        return "GO";
    case T_ADC_ENQ:
        return "ADC_ENQ";
    case T_DAC:
        return "DAC";
    case T_DDR_RD:
        return "DDR_RD";
    case T_DDR_WR:
        return "DDR_WR";
    case T_HOST_PRODUCE:
        return "HOST_PRODUCE";
    case T_DMA_NOT_AVAILABLE:
        return "NO_DMA";
    case T_AXIQ_COMPLETE:
        return "AXIQ_COMPLETE";
    case T_DDR_COMPLETE:
        return "DDR_COMPLETE";
    case T_AXIQ_TX_ENABLE:
        return "AXIQ_TX_EN";
    case T_AXIQ_RX0_ENABLE:
        return "AXIQ_RX0_EN";
    case T_AXIQ_RX1_ENABLE:
        return "AXIQ_RX1_EN";
    case T_AXIQ_RO0_ENABLE:
        return "AXIQ_RO0_EN";
    case T_AXIQ_RO1_ENABLE:
        return "AXIQ_RO0_EN";
    case T_BUFFER_FILL:
        return "BUFFER_FILL";
    case T_INTER_CACHE_FILL:
        return "INT_CACHE";
    case T_DEC_CACHE_FILL:
        return "DEC_CACHE";
    case T_PHYTIMER:
        return "TIMER";
    case T_ERROR:
        return "ERROR";
    case T_TIME_NOW:
        return "PHYTIME_NOW";
    case T_MBOX:
        return "MBOX";
    case T_ADC_COMPLETE:
        return "ADC_COMPLETE";
    case T_DDR_WR_COMPLETE:
        return "DDR_WR_COMPLETE";
    default: {
        char ctemp[32];
        sprintf(ctemp, "%X", op);
        return ctemp;
    }
    }
}

enum {
    CNT_TX_UDR,
    CNT_TX_OVR,
    CNT_RX0_UDR,
    CNT_RX1_UDR,
    CNT_RX2_UDR,
    CNT_RX3_UDR,
    CNT_RX0_OVR,
    CNT_RX1_OVR,
    CNT_RX2_OVR,
    CNT_RX3_OVR,
    CNT_DECIM,
    CNT_INTERP,
    CNT_PHYTIME,
    CNT_TX_AXIQ_EN,
    CNT_TX_DMA_ALLOW,
    CNT_DDR_RD_ENQ,
    CNT_DDR_RD_READY,
    CNT_ADC_ENQ,
    CNT_DAC_ENQ,
    CNT_DAC_COMPLETION_TIME,
    CNT_DDR_RD_COMPLETION_TIME,
    CNT_DDR_WR_ENQ,
    CNT_ADC_READY,
    CNT_DAC_READY,
};

static std::string CounterNames(uint32_t id)
{
    switch (id)
    {
    case CNT_TX_UDR:
        return "TX_UDR";
    case CNT_TX_OVR:
        return "TX_OVR";
    case CNT_RX0_UDR:
        return "RX0_UDR";
    case CNT_RX1_UDR:
        return "RX1_UDR";
    case CNT_RX2_UDR:
        return "RX2_UDR";
    case CNT_RX3_UDR:
        return "RX3_UDR";
    case CNT_RX0_OVR:
        return "RX0_OVR";
    case CNT_RX1_OVR:
        return "RX1_OVR";
    case CNT_RX2_OVR:
        return "RX2_OVR";
    case CNT_RX3_OVR:
        return "RX3_OVR";
    case CNT_DECIM:
        return "Decim_done";
    case CNT_INTERP:
        return "Interp done";
    case CNT_PHYTIME:
        return "PHYTIME";
    case CNT_TX_AXIQ_EN:
        return "TX_AXIQ_EN";
    case CNT_TX_DMA_ALLOW:
        return "TX_DMA_ALLOW";
    case CNT_DDR_RD_ENQ:
        return "DDR_RD_ENQ";
    case CNT_DDR_RD_READY:
        return "DDR_RD_READY";
    case CNT_ADC_ENQ:
        return "ADC_ENQ";
    case CNT_DAC_ENQ:
        return "DAC_ENQ";
    case CNT_DAC_COMPLETION_TIME:
        return "DAC_COMPLETION_TIME";
    case CNT_DDR_RD_COMPLETION_TIME:
        return "DDR_RD_COMPLETION_TIME";
    case CNT_DDR_WR_ENQ:
        return "DDR_WR_ENQ";
    case CNT_ADC_READY:
        return "ADC_READY";
    case CNT_DAC_READY:
        return "DAC_READY";
    default: {
        char ctemp[32];
        sprintf(ctemp, "%d", id);
        return ctemp;
    }
    }
}

Event Convert(const l1_trace_data_t& data)
{
    Event evt;
    evt.pid = (data.msg >> 28) & 0xf;
    evt.phase = static_cast<ePhase>((data.msg >> 25) & 0x7);
    evt.tid = (data.msg >> 20) & 0xf;
    evt.id = data.param;
    evt.timestamp = data.cnt;
    if (evt.phase == ePhase::Counter)
    {
        evt.name = CounterNames(data.msg & 0xFFFFF);
        evt.id = 0;
        evt.value = data.param;
    }
    else if (evt.phase == ePhase::Complete)
    {
        evt.name = OpName(data.msg & 0xFFFFF);
        evt.id = 0;
        evt.value = data.param;
    }
    else
        evt.name = NameWithTag(OpName(data.msg & 0xFFFFF), data.param);
    if (evt.pid == 2)
    {
        evt.pid = evt.tid + 100;
        evt.tid = data.param;
        evt.name = OpName(data.msg & 0xFFFFF);
    }
    // evt.name = OpName(data.msg & 0x1FFFFF);
    evt.category = "";
    return evt;
}

void ToTraceFile(std::ofstream& ofs, const std::vector<l1_trace_data_t> events)
{
    for (const auto& e : events)
    {
        if (e.msg == 0)
            break;

        Event evt = Convert(e);
        // if (!evt.pid || evt.name.empty())
        //     continue;

        // if (!baseTime)
        //     baseTime = data[i].cnt;

        // evt.timestamp -= baseTime;
        ofs << ToString(evt) << '\n'; //std::endl;
        // std::cout << ToString(evt) << std::endl;
    }
}