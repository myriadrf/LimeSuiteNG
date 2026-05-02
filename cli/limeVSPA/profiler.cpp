#include "l1-trace.h"

#include <sstream>

extern std::string GetMsgName(uint32_t msg);

static const double clockRate = 100e6;
static const double tickDuration = 1 / clockRate;

enum class ePhase {
    Begin,
    End,
};

struct Event {
    std::string name;
    std::string category;
    ePhase phase;
    uint64_t timestamp;
    uint32_t pid;
    uint32_t tid;
};

std::string ToString(Event& evt)
{
    std::stringstream ss;
    ss << "{ " << "\"cat\": \"" << evt.category << "\"" << ",\"ts\":" << std::setprecision(9)
       << double(evt.timestamp * tickDuration) * 1e6 << ",\"pid\":" << evt.pid << ",\"tid\":" << evt.tid << ",\"ph\":" << "\""
       << (evt.phase == ePhase::Begin ? "B" : "E") << "\"" << ",\"name\": \"" << evt.name << "\"" << "}";
    return ss.str();
}

Event Convert(const l1_trace_data_t& data)
{
    Event evt;
    evt.pid = 1;
    switch (data.msg)
    {
    case L1_TRACE_MSG_DMA_AXIQ_RX_START:
        evt.pid = 2;
        evt.name = "AXIQ_RX";
        evt.category = "RX";
        evt.phase = ePhase::Begin;
        evt.timestamp = data.cnt;
        evt.tid = data.param;
        break;
    case L1_TRACE_MSG_DMA_AXIQ_RX_COMP:
        evt.pid = 2;
        evt.name = "AXIQ_RX";
        evt.category = "RX";
        evt.phase = ePhase::End;
        evt.timestamp = data.cnt;
        evt.tid = data.param;
        break;

    case L1_TRACE_MSG_DMA_DDR_WR_START:
        evt.pid = 2;
        evt.name = "DDR_WR";
        evt.category = "RX";
        evt.phase = ePhase::Begin;
        evt.timestamp = data.cnt;
        evt.tid = data.param;
        break;
    case L1_TRACE_MSG_DMA_DDR_WR_COMP:
        evt.pid = 2;
        evt.name = "DDR_WR";
        evt.category = "RX";
        evt.phase = ePhase::End;
        evt.timestamp = data.cnt;
        evt.tid = data.param;
        break;

    case L1_TRACE_L1APP_RX_QEC_START:
        evt.name = "QEC_RX";
        evt.category = "RX";
        evt.phase = ePhase::Begin;
        evt.timestamp = data.cnt;
        evt.tid = data.param;
        break;
    case L1_TRACE_L1APP_RX_QEC_COMP:
        evt.name = "QEC_RX";
        evt.category = "RX";
        evt.phase = ePhase::End;
        evt.timestamp = data.cnt;
        evt.tid = data.param;
        break;

    case L1_TRACE_MSG_DMA_DDR_RD_START:
        evt.pid = 2;
        evt.name = "DDR_RD";
        evt.category = "TX";
        evt.phase = ePhase::Begin;
        evt.timestamp = data.cnt;
        evt.tid = data.param;
        break;
    case L1_TRACE_MSG_DMA_DDR_RD_COMP:
        evt.pid = 2;
        evt.name = "DDR_RD";
        evt.category = "TX";
        evt.phase = ePhase::End;
        evt.timestamp = data.cnt;
        evt.tid = data.param;
        break;
    case L1_TRACE_L1APP_TX_INTERP_START:
        evt.pid = 2;
        evt.name = "TX_INTERP";
        evt.category = "TX";
        evt.phase = ePhase::Begin;
        evt.timestamp = data.cnt;
        evt.tid = data.param;
        break;
    case L1_TRACE_L1APP_TX_INTERP_COMP:
        evt.pid = 2;
        evt.name = "TX_INTERP";
        evt.category = "TX";
        evt.phase = ePhase::End;
        evt.timestamp = data.cnt;
        evt.tid = data.param;
        break;
    case L1_TRACE_L1APP_TX_QEC_START:
        evt.name = "QEC_TX";
        evt.category = "TX";
        evt.phase = ePhase::Begin;
        evt.timestamp = data.cnt;
        evt.tid = data.param;
        break;
    case L1_TRACE_L1APP_TX_QEC_COMP:
        evt.name = "QEC_TX";
        evt.category = "TX";
        evt.phase = ePhase::End;
        evt.timestamp = data.cnt;
        evt.tid = data.param;
        break;
    case L1_TRACE_MSG_DMA_AXIQ_TX_START:
        evt.pid = 2;
        evt.name = "AXIQ_TX";
        evt.category = "TX";
        evt.phase = ePhase::Begin;
        evt.timestamp = data.cnt;
        evt.tid = data.param;
        break;
    case L1_TRACE_MSG_DMA_AXIQ_TX_COMP:
        evt.pid = 2;
        evt.name = "AXIQ_TX";
        evt.category = "TX";
        evt.phase = ePhase::End;
        evt.timestamp = data.cnt;
        evt.tid = data.param;
        break;

    case L1_TRACE_MSG_DMA_AXIQ_RX_OVER:
    case L1_TRACE_MSG_DMA_AXIQ_RX_UNDER:
    case L1_TRACE_MSG_DMA_AXIQ_TX_OVER:
    case L1_TRACE_MSG_DMA_AXIQ_TX_UNDER:
        break;
    }
    return evt;
}

std::string GenerateTraceFile(const l1_trace_data_t* data, uint32_t length)
{
    std::stringstream ss;
    ss << "{\n"
       << "\"displayTimeUnit\":\"ns\"," << "\"traceEvents\": [\n";
    uint64_t baseTime = 0;
    for (uint32_t i = 0; i < length; ++i)
    {
        if (data[i].msg == 0)
            break;

        Event evt = Convert(data[i]);
        if (!evt.pid || evt.name.empty())
            continue;

        if (!baseTime)
            baseTime = data[i].cnt;

        evt.timestamp -= baseTime;
        ss << ToString(evt) << std::endl;
    }
    ss << "]\n}";
    return ss.str();
}