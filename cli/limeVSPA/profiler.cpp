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
    ss << "{ "
       << "\"ts\":"
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
    case T_BUSY:
        return "BUSY";
    case T_GO:
        return "GO";
    case T_XFER_BUFFER:
        return "DMA";
        ;
    case T_EXTERNAL_GO:
        return "EXT_GO";
    case T_TRACE_PUSH:
        return "TRACE";
    case T_DDR_WR_ENQ:
        return "DDR_WR_ENQ";
    case T_DDR_RD_ENQ:
        return "DDR_RD_ENQ";
    case T_DDR_WR_COMPLETE:
        return "DDR_WR_C";
    case T_DDR_RD_COMPLETE:
        return "DDR_RD_C";
    case T_ADC_COMPLETE:
        return "ADC";
    case T_DAC_COMPLETE:
        return "DAC";
    case T_RX_WORK:
        return "RX_WORK";
    case T_TX_WORK:
        return "TX_WORK";
    case T_ADC_ENQ:
        return "T_ADC_ENQ";
    case T_DAC_ENQ:
        return "T_DAC_ENQ";
    case T_DAC_AXIQ_RST:
        return "T_DAC_AXIQ_RST";
    case T_ADC_AXIQ_RST:
        return "T_ADC_AXIQ_RST";

    default: {
        char ctemp[32];
        sprintf(ctemp, "%X", op);
        return ctemp;
    }
    }
}

enum {
    CNT_GO,
    CNT_HOST_UDR,
    CNT_TX_DFE_UDR,
    CNT_TX_AFE_UDR,
    CNT_TX_AFE_OVR,
    CNT_RX_DDR_ENQ,
    CNT_TX_DDR_ENQ,
    CNT_TX_TCD,
};

static std::string CounterNames(uint32_t id)
{
    switch (id)
    {

    case CNT_GO:
        return "GO";
    case CNT_HOST_UDR:
        return "CNT_HOST_UDR";
    case CNT_TX_DFE_UDR:
        return "CNT_TX_DFE_UDR";
    case CNT_TX_AFE_UDR:
        return "CNT_TX_AFE_UDR";
    case CNT_TX_AFE_OVR:
        return "CNT_TX_AFE_OVR";
    case CNT_RX_DDR_ENQ:
        return "CNT_RX_DDR_ENQ";
    case CNT_TX_DDR_ENQ:
        return "CNT_TX_DDR_ENQ";
    case CNT_TX_TCD:
        return "CNT_TX_TCD";
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
    size_t cnt = 0;
    for (const auto& e : events)
    {
        if (e.msg == 0)
        {
            printf("Bad msg %i\n", cnt);
            break;
        }

        Event evt = Convert(e);
        // if (!evt.pid || evt.name.empty())
        //     continue;

        // if (!baseTime)
        //     baseTime = data[i].cnt;

        // evt.timestamp -= baseTime;
        ofs << ToString(evt) << ",\n"; //std::endl;
        ++cnt;
        // std::cout << ToString(evt) << std::endl;
    }

    ofs.flush();
    printf("Events to file: %i\n", cnt);
}