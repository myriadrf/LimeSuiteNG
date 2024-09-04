#include "CommonFunctions.h"
#include "lime/LimeSuite.h"
#include "limesuiteng/limesuiteng.hpp"
#include "limesuiteng/LMS7002M.h"
#include "chips/LMS7002M/LMS7002MCSR_Data.h"
#include "utilities/DeltaVariable.h"
#include "utilities/toString.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

using namespace lime;
using namespace lime::LMS7002MCSR_Data;
using namespace std;
using namespace std::literals::string_literals;

static inline uint64_t ClearBit(uint64_t value, size_t bitIndex)
{
    return value & (~(1 << bitIndex));
}

static inline uint64_t InsertBit(uint64_t value, size_t bitIndex)
{
    return value | (1 << bitIndex);
}

static inline bool IsSetBit(uint64_t value, size_t bitIndex)
{
    return value & (1 << bitIndex);
}

namespace {

struct StatsDeltas {
    lime::DeltaVariable<uint32_t> underrun;
    lime::DeltaVariable<uint32_t> overrun;
    lime::DeltaVariable<uint32_t> droppedPackets;

    StatsDeltas()
        : underrun(0)
        , overrun(0)
        , droppedPackets(0)
    {
    }
};

struct StreamStagingBuffers {
    std::vector<uint8_t> buffer[2];
    size_t bufferBytesFilled;
    uint64_t timestamp;

    lime::TRXDir direction;
    uint8_t maskDataPresentInBuffer;
    uint8_t maskChannelsActive;
    uint8_t maskChannelsSetup;

    StreamStagingBuffers()
        : timestamp(0)
        , maskDataPresentInBuffer(0)
        , maskChannelsActive(0)
        , maskChannelsSetup(0)
    {
        buffer[0].resize(sizeof(complex32f_t) * 1024 * 1024);
        buffer[1].resize(sizeof(complex32f_t) * 1024 * 1024);
        bufferBytesFilled = 0;
    }
};

struct LMS_APIDevice {
    lime::SDRDevice* device;
    lime::StreamConfig lastSavedStreamConfig;
    std::array<std::array<float_type, 2>, lime::SDRConfig::MAX_CHANNEL_COUNT> lastSavedLPFValue;
    StatsDeltas statsDeltas;

    uint8_t moduleIndex;

    std::array<StreamStagingBuffers, 2> streamBuffers;
    lms_dev_info_t* deviceInfo;

    lime::eGainTypes rxGain;
    lime::eGainTypes txGain;

    LMS_APIDevice() = delete;
    LMS_APIDevice(lime::SDRDevice* device)
        : device(device)
        , lastSavedStreamConfig()
        , lastSavedLPFValue()
        , statsDeltas()
        , moduleIndex(0)
        , deviceInfo(nullptr)
        , rxGain(lime::eGainTypes::UNKNOWN)
        , txGain(lime::eGainTypes::UNKNOWN)
    {
    }

    ~LMS_APIDevice()
    {
        lime::DeviceRegistry::freeDevice(device);

        if (deviceInfo != nullptr)
        {
            delete deviceInfo;
        }
    }

    const lime::RFSOCDescriptor& GetRFSOCDescriptor() const
    {
        assert(device);
        return device->GetDescriptor().rfSOC.at(moduleIndex);
    }
};

struct SubChannelHandle {
    static constexpr std::size_t MAX_ELEMENTS_IN_BUFFER = 4096;

    LMS_APIDevice* parent;
    StreamStagingBuffers* stagingArea;
    size_t channelIndex;

    SubChannelHandle() = delete;
    SubChannelHandle(LMS_APIDevice* parent, StreamStagingBuffers* staging, uint8_t index)
        : parent(parent)
        , stagingArea(staging)
        , channelIndex(index)
    {
    }
};

static inline int OpStatusToReturnCode(OpStatus value)
{
    return value == OpStatus::Success ? 0 : -1;
}

static inline LMS_APIDevice* CheckDevice(lms_device_t* device)
{
    if (device == nullptr)
    {
        lime::error("Device cannot be NULL."s);
        return nullptr;
    }

    return static_cast<LMS_APIDevice*>(device);
}

static inline LMS_APIDevice* CheckDevice(lms_device_t* device, unsigned chan)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr || apiDevice->device == nullptr)
    {
        return nullptr;
    }

    if (chan >= apiDevice->GetRFSOCDescriptor().channelCount)
    {
        lime::error("Invalid channel number."s);
        return nullptr;
    }

    return apiDevice;
}

static inline void CopyString(const std::string_view source, char* destination, std::size_t destinationLength)
{
    std::size_t charsToCopy = std::min(destinationLength - 1, source.size());
    std::strncpy(destination, source.data(), charsToCopy);
    destination[charsToCopy] = 0;
}

template<class T> static inline lms_range_t RangeToLMS_Range(const lime::Range<T>& range)
{
    return { range.min, range.max, range.step };
}

static LMS_LogHandler api_msg_handler;
static void APIMsgHandler(const lime::LogLevel level, const char* message)
{
    api_msg_handler(static_cast<int>(level), message);
}

static lms_prog_callback_t programmingCallback;
static bool ProgrammingCallback(std::size_t bsent, std::size_t btotal, const std::string& statusMessage)
{
    return programmingCallback(static_cast<int>(bsent), static_cast<int>(btotal), statusMessage.c_str());
}

} //unnamed namespace

API_EXPORT int CALL_CONV LMS_GetDeviceList(lms_info_str_t* dev_list)
{
    std::vector<lime::DeviceHandle> handles = lime::DeviceRegistry::enumerate();

    if (dev_list != nullptr)
    {
        for (std::size_t i = 0; i < handles.size(); ++i)
        {
            std::string str = handles[i].Serialize();
            CopyString(str, dev_list[i], sizeof(lms_info_str_t));
        }
    }

    return handles.size();
}

API_EXPORT int CALL_CONV LMS_Open(lms_device_t** device, const lms_info_str_t info, void* args)
{
    if (device == nullptr)
    {
        lime::error("Device pointer cannot be NULL."s);
        return -1;
    }

    std::vector<lime::DeviceHandle> handles = lime::DeviceRegistry::enumerate();
    for (auto& handle : handles)
    {
        if (info == nullptr || std::strcmp(handle.Serialize().c_str(), info) == 0)
        {
            auto dev = lime::DeviceRegistry::makeDevice(handle);

            if (dev == nullptr)
            {
                lime::error("Unable to open device."s);
                return -1;
            }

            auto apiDevice = new LMS_APIDevice{ dev };

            *device = apiDevice;
            return LMS_SUCCESS;
        }
    }

    lime::error("Specified device could not be found."s);
    return -1;
}

API_EXPORT int CALL_CONV LMS_Close(lms_device_t* device)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    delete apiDevice;
    return LMS_SUCCESS;
}

API_EXPORT int CALL_CONV LMS_Init(lms_device_t* device)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    try
    {
        OpStatus status = apiDevice->device->Init();
        return OpStatusToReturnCode(status);
    } catch (...)
    {
        return -1;
    }
}

API_EXPORT int CALL_CONV LMS_Reset(lms_device_t* device)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    try
    {
        apiDevice->device->Reset();
    } catch (...)
    {
        return -1;
    }

    return 0;
}

API_EXPORT int CALL_CONV LMS_EnableChannel(lms_device_t* device, bool dir_tx, size_t chan, bool enabled)
{
    LMS_APIDevice* apiDevice = CheckDevice(device, chan);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    lime::TRXDir direction = dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx;
    try
    {
        apiDevice->device->EnableChannel(apiDevice->moduleIndex, direction, chan, enabled);
    } catch (...)
    {
        lime::error("Device configuration failed."s);

        return -1;
    }

    return 0;
}

API_EXPORT int CALL_CONV LMS_SetSampleRate(lms_device_t* device, float_type rate, size_t oversample)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    try
    {
        apiDevice->device->SetSampleRate(apiDevice->moduleIndex, lime::TRXDir::Rx, 0, rate, oversample);
    } catch (...)
    {
        lime::error("Failed to set sampling rate."s);

        return -1;
    }

    return 0;
}

API_EXPORT int CALL_CONV LMS_SetSampleRateDir(lms_device_t* device, bool dir_tx, float_type rate, size_t oversample)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    lime::TRXDir direction = dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx;

    try
    {
        apiDevice->device->SetSampleRate(apiDevice->moduleIndex, direction, 0, rate, oversample);

    } catch (...)
    {
        lime::error("Failed to set %s sampling rate.", ToString(direction).c_str());

        return -1;
    }

    return 0;
}

API_EXPORT int CALL_CONV LMS_GetSampleRate(lms_device_t* device, bool dir_tx, size_t chan, float_type* host_Hz, float_type* rf_Hz)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    lime::TRXDir direction = dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx;
    uint32_t rf_rate;
    auto rate = apiDevice->device->GetSampleRate(apiDevice->moduleIndex, direction, 0, &rf_rate);

    if (host_Hz)
        *host_Hz = rate;
    if (rf_Hz)
        *rf_Hz = rf_rate;

    return 0;
}

API_EXPORT int CALL_CONV LMS_GetSampleRateRange(lms_device_t* device, bool dir_tx, lms_range_t* range)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    if (range)
        *range = RangeToLMS_Range(apiDevice->GetRFSOCDescriptor().samplingRateRange);

    return 0;
}

API_EXPORT int CALL_CONV LMS_GetNumChannels(lms_device_t* device, bool dir_tx)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    return apiDevice->GetRFSOCDescriptor().channelCount;
}

API_EXPORT int CALL_CONV LMS_SetLOFrequency(lms_device_t* device, bool dir_tx, size_t chan, float_type frequency)
{
    LMS_APIDevice* apiDevice = CheckDevice(device, chan);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    lime::TRXDir direction = dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx;

    try
    {
        apiDevice->device->SetFrequency(apiDevice->moduleIndex, direction, chan, frequency);
    } catch (...)
    {
        lime::error("Failed to set %s LO frequency.", ToString(direction).c_str());

        return -1;
    }

    return 0;
}

API_EXPORT int CALL_CONV LMS_GetLOFrequency(lms_device_t* device, bool dir_tx, size_t chan, float_type* frequency)
{
    LMS_APIDevice* apiDevice = CheckDevice(device, chan);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    if (frequency)
        *frequency = apiDevice->device->GetFrequency(apiDevice->moduleIndex, dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx, chan);

    return 0;
}

API_EXPORT int CALL_CONV LMS_GetLOFrequencyRange(lms_device_t* device, bool dir_tx, lms_range_t* range)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    if (range)
        *range = RangeToLMS_Range(apiDevice->GetRFSOCDescriptor().frequencyRange);

    return 0;
}

API_EXPORT int CALL_CONV LMS_GetAntennaList(lms_device_t* device, bool dir_tx, size_t chan, lms_name_t* list)
{
    LMS_APIDevice* apiDevice = CheckDevice(device, chan);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    const auto& rfSOC = apiDevice->GetRFSOCDescriptor();
    const auto& strings = rfSOC.pathNames.at(dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx);

    if (list != nullptr)
    {
        for (std::size_t i = 0; i < strings.size(); ++i)
        {
            CopyString(strings.at(i), list[i], sizeof(lms_name_t));
        }
    }

    return strings.size();
}

API_EXPORT int CALL_CONV LMS_SetAntenna(lms_device_t* device, bool dir_tx, size_t chan, size_t path)
{
    LMS_APIDevice* apiDevice = CheckDevice(device, chan);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    lime::TRXDir direction = dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx;

    try
    {
        apiDevice->device->SetAntenna(apiDevice->moduleIndex, direction, chan, path);
    } catch (...)
    {
        lime::error("Failed to set %s antenna.", ToString(direction).c_str());

        return -1;
    }

    return 0;
}

API_EXPORT int CALL_CONV LMS_GetAntenna(lms_device_t* device, bool dir_tx, size_t chan)
{
    LMS_APIDevice* apiDevice = CheckDevice(device, chan);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    lime::TRXDir direction = dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx;

    return apiDevice->device->GetAntenna(apiDevice->moduleIndex, direction, chan);
}

API_EXPORT int CALL_CONV LMS_GetAntennaBW(lms_device_t* device, bool dir_tx, size_t chan, size_t path, lms_range_t* range)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    lime::TRXDir direction = dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx;
    std::string pathName = apiDevice->GetRFSOCDescriptor().pathNames.at(direction).at(path);

    if (range)
    {
        *range = RangeToLMS_Range(apiDevice->GetRFSOCDescriptor().antennaRange.at(direction).at(pathName));
    }

    return 0;
}

API_EXPORT int CALL_CONV LMS_SetLPFBW(lms_device_t* device, bool dir_tx, size_t chan, float_type bandwidth)
{
    LMS_APIDevice* apiDevice = CheckDevice(device, chan);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    lime::TRXDir direction = dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx;

    try
    {
        apiDevice->device->SetLowPassFilter(apiDevice->moduleIndex, direction, chan, bandwidth);
        apiDevice->lastSavedLPFValue[chan][dir_tx] = bandwidth;
    } catch (...)
    {
        lime::error("Failed to set %s LPF bandwidth.", ToString(direction).c_str());

        return -1;
    }

    return 0;
}

API_EXPORT int CALL_CONV LMS_GetLPFBWRange(lms_device_t* device, bool dir_tx, lms_range_t* range)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    lime::TRXDir direction = dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx;

    if (range)
        *range = RangeToLMS_Range(apiDevice->GetRFSOCDescriptor().lowPassFilterRange.at(direction));

    return 0;
}

API_EXPORT int CALL_CONV LMS_SetNormalizedGain(lms_device_t* device, bool dir_tx, size_t chan, float_type gain)
{
    LMS_APIDevice* apiDevice = CheckDevice(device, chan);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    gain = std::clamp(gain, 0.0, 1.0);

    auto direction = dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx;
    auto gainToUse = dir_tx ? apiDevice->txGain : apiDevice->rxGain;

    auto range = apiDevice->GetRFSOCDescriptor().gainRange.at(direction).at(gainToUse);

    gain = range.min + gain * (range.max - range.min);

    try
    {
        apiDevice->device->SetGain(apiDevice->moduleIndex, direction, chan, gainToUse, gain);
    } catch (...)
    {
        lime::error("Failed to set %s normalized gain.", ToString(direction).c_str());

        return -1;
    }

    return 0;
}

API_EXPORT int CALL_CONV LMS_SetGaindB(lms_device_t* device, bool dir_tx, size_t chan, unsigned gain)
{
    LMS_APIDevice* apiDevice = CheckDevice(device, chan);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    auto direction = dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx;
    auto gainToUse = dir_tx ? apiDevice->txGain : apiDevice->rxGain;

    try
    {
        apiDevice->device->SetGain(apiDevice->moduleIndex, direction, chan, gainToUse, gain);
    } catch (...)
    {
        lime::error("Failed to set %s gain.", ToString(direction).c_str());

        return -1;
    }

    return 0;
}

API_EXPORT int CALL_CONV LMS_GetNormalizedGain(lms_device_t* device, bool dir_tx, size_t chan, float_type* gain)
{
    LMS_APIDevice* apiDevice = CheckDevice(device, chan);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    auto direction = dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx;
    auto gainToUse = dir_tx ? apiDevice->txGain : apiDevice->rxGain;

    auto range = apiDevice->GetRFSOCDescriptor().gainRange.at(direction).at(gainToUse);

    double deviceGain = 0.0;
    OpStatus returnValue = apiDevice->device->GetGain(apiDevice->moduleIndex, direction, chan, gainToUse, deviceGain);

    if (gain)
        *gain = (deviceGain - range.min) / (range.max - range.min);

    return OpStatusToReturnCode(returnValue);
}

API_EXPORT int CALL_CONV LMS_GetGaindB(lms_device_t* device, bool dir_tx, size_t chan, unsigned* gain)
{
    LMS_APIDevice* apiDevice = CheckDevice(device, chan);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    auto direction = dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx;
    auto gainToUse = dir_tx ? apiDevice->txGain : apiDevice->rxGain;
    auto deviceGain = 0.0;

    OpStatus returnValue = apiDevice->device->GetGain(apiDevice->moduleIndex, direction, chan, gainToUse, deviceGain);

    if (gain)
        *gain = std::lround(deviceGain) + 12;

    return OpStatusToReturnCode(returnValue);
}

API_EXPORT int CALL_CONV LMS_Calibrate(lms_device_t* device, bool dir_tx, size_t chan, double bw, unsigned flags)
{
    LMS_APIDevice* apiDevice = CheckDevice(device, chan);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    auto direction = dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx;

    try
    {
        apiDevice->device->Calibrate(apiDevice->moduleIndex, direction, chan, bw);
    } catch (...)
    {
        lime::error("Failed to calibrate %s channel %li.", ToString(direction).c_str(), chan);

        return -1;
    }

    return 0;
}

API_EXPORT int CALL_CONV LMS_SetTestSignal(
    lms_device_t* device, bool dir_tx, size_t chan, lms_testsig_t sig, int16_t dc_i, int16_t dc_q)
{
    LMS_APIDevice* apiDevice = CheckDevice(device, chan);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    if (sig > LMS_TESTSIG_DC)
    {
        lime::error("Invalid signal."s);
        return -1;
    }

    auto direction = dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx;

    auto enumToTestStruct = [](lms_testsig_t signal) -> lime::ChannelConfig::Direction::TestSignal {
        switch (signal)
        {
        case LMS_TESTSIG_NONE:
            return { false };
        case LMS_TESTSIG_DC:
            return { true, true };
        case LMS_TESTSIG_NCODIV8:
            return { true,
                false,
                lime::ChannelConfig::Direction::TestSignal::Divide::Div8,
                lime::ChannelConfig::Direction::TestSignal::Scale::Half };
        case LMS_TESTSIG_NCODIV4:
            return { true,
                false,
                lime::ChannelConfig::Direction::TestSignal::Divide::Div4,
                lime::ChannelConfig::Direction::TestSignal::Scale::Half };
        case LMS_TESTSIG_NCODIV8F:
            return { true,
                false,
                lime::ChannelConfig::Direction::TestSignal::Divide::Div8,
                lime::ChannelConfig::Direction::TestSignal::Scale::Full };
        case LMS_TESTSIG_NCODIV4F:
            return { true,
                false,
                lime::ChannelConfig::Direction::TestSignal::Divide::Div4,
                lime::ChannelConfig::Direction::TestSignal::Scale::Full };
        default:
            throw std::logic_error("Unexpected enumerator lms_testsig_t value"s);
        }
    };

    try
    {
        apiDevice->device->SetTestSignal(apiDevice->moduleIndex, direction, chan, enumToTestStruct(sig), dc_i, dc_q);
    } catch (...)
    {
        lime::error("Failed to set %s channel %li test signal.", ToString(direction).c_str(), chan);
    }

    return 0;
}

API_EXPORT int CALL_CONV LMS_SetupStream(lms_device_t* device, lms_stream_t* stream)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    if (stream == nullptr)
    {
        lime::error("Stream cannot be NULL."s);
        return -1;
    }

    lime::StreamConfig config = apiDevice->lastSavedStreamConfig;
    config.bufferSize = stream->fifoSize;

    auto channelIndex = stream->channel & ~LMS_ALIGN_CH_PHASE; // Clear the align phase bit

    config.channels.at(stream->isTx ? lime::TRXDir::Tx : lime::TRXDir::Rx).push_back(channelIndex);

    config.alignPhase = stream->channel & LMS_ALIGN_CH_PHASE;

    switch (stream->dataFmt)
    {
    case lms_stream_t::LMS_FMT_F32:
        config.format = lime::DataFormat::F32;
        break;
    case lms_stream_t::LMS_FMT_I16:
        config.format = lime::DataFormat::I16;
        break;
    case lms_stream_t::LMS_FMT_I12:
        config.format = lime::DataFormat::I12;
        break;
    default:
        return lime::error("Setup stream failed: invalid data format."s);
    }

    switch (stream->linkFmt)
    {
    case lms_stream_t::LMS_LINK_FMT_I16:
        config.linkFormat = lime::DataFormat::I16;
        break;
    case lms_stream_t::LMS_LINK_FMT_I12:
    case lms_stream_t::LMS_LINK_FMT_DEFAULT:
    default:
        config.linkFormat = lime::DataFormat::I12;
        break;
    }

    // stream->throughputVsLatency can be ignored, it's automatic now

    OpStatus returnValue = apiDevice->device->StreamSetup(config, apiDevice->moduleIndex);

    if (returnValue == OpStatus::Success)
    {
        apiDevice->lastSavedStreamConfig = config;
    }

    StreamStagingBuffers& stage = apiDevice->streamBuffers.at(stream->isTx ? 1 : 0);
    SubChannelHandle* handle = new SubChannelHandle(apiDevice, &stage, channelIndex);
    stage.maskChannelsSetup = InsertBit(stage.maskChannelsSetup, channelIndex);

    stream->handle = reinterpret_cast<size_t>(handle);
    return returnValue == OpStatus::Success ? 0 : -1;
}

API_EXPORT int CALL_CONV LMS_DestroyStream(lms_device_t* device, lms_stream_t* stream)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    if (stream == nullptr)
    {
        lime::error("Stream cannot be NULL."s);
        return -1;
    }

    apiDevice->device->StreamDestroy(apiDevice->moduleIndex);
    SubChannelHandle* handle = reinterpret_cast<SubChannelHandle*>(stream->handle);
    handle->stagingArea->maskChannelsSetup = ClearBit(handle->stagingArea->maskChannelsSetup, handle->channelIndex);
    if (handle != nullptr)
        delete handle;

    auto& channels = apiDevice->lastSavedStreamConfig.channels.at(stream->isTx ? lime::TRXDir::Tx : lime::TRXDir::Rx);
    auto iter = std::find(channels.begin(), channels.end(), stream->channel);
    if (iter != std::end(channels))
        channels.erase(iter);

    return 0;
}

API_EXPORT int CALL_CONV LMS_StartStream(lms_stream_t* stream)
{
    if (stream == nullptr || stream->handle < 0)
    {
        return -1;
    }

    SubChannelHandle* handle = reinterpret_cast<SubChannelHandle*>(stream->handle);
    if (handle == nullptr || handle->parent == nullptr)
    {
        return -1;
    }

    StreamStagingBuffers* stage = handle->stagingArea;
    stage->maskChannelsActive = InsertBit(stage->maskChannelsActive, handle->channelIndex);

    if (stage->maskChannelsActive == stage->maskChannelsSetup)
        handle->parent->device->StreamStart(handle->parent->moduleIndex);

    return 0;
}

API_EXPORT int CALL_CONV LMS_StopStream(lms_stream_t* stream)
{
    if (stream == nullptr || stream->handle < 0)
    {
        return -1;
    }

    SubChannelHandle* handle = reinterpret_cast<SubChannelHandle*>(stream->handle);
    if (handle == nullptr || handle->parent == nullptr)
    {
        return -1;
    }

    StreamStagingBuffers* stage = handle->stagingArea;
    stage->maskChannelsActive = ClearBit(stage->maskChannelsActive, handle->channelIndex);

    if (stage->maskChannelsActive == 0)
        handle->parent->device->StreamStop(handle->parent->moduleIndex);

    return 0;
}

namespace {

template<class T>
int ReceiveStream(lms_stream_t* stream, void* samples, size_t sample_count, lms_stream_meta_t* meta, chrono::microseconds timeout)
{
    SubChannelHandle* handle = reinterpret_cast<SubChannelHandle*>(stream->handle);
    if (handle == nullptr || handle->parent == nullptr)
    {
        return -1;
    }

    const auto direction = stream->isTx ? lime::TRXDir::Tx : lime::TRXDir::Rx;
    if (direction == lime::TRXDir::Tx)
    {
        lime::error("Invalid direction."s);
        return -1;
    }

    const std::size_t sampleSize = sizeof(T);
    int samplesToReturn = 0;

    StreamStagingBuffers* stage = handle->stagingArea;
    if (stage->maskDataPresentInBuffer == 0)
    {
        // if staging buffers are depleted request new batch
        lime::StreamMeta metadata{ 0, false, false };
        T* dest[2] = { reinterpret_cast<T*>(stage->buffer[0].data()), reinterpret_cast<T*>(stage->buffer[1].data()) };
        size_t samplesProduced = handle->parent->device->StreamRx(
            handle->parent->moduleIndex, reinterpret_cast<T**>(dest), sample_count, &metadata, timeout);

        samplesToReturn = samplesProduced;
        stage->maskDataPresentInBuffer = stage->maskChannelsActive;
        stage->bufferBytesFilled = samplesProduced * sampleSize;
    }

    // take samples from staging buffer
    if (IsSetBit(stage->maskDataPresentInBuffer, handle->channelIndex))
    {
        samplesToReturn = std::min(sample_count, stage->bufferBytesFilled / sampleSize);
        std::memcpy(samples, stage->buffer[handle->channelIndex].data(), samplesToReturn * sampleSize);
        stage->maskDataPresentInBuffer = ClearBit(stage->maskDataPresentInBuffer, handle->channelIndex);
        if (stage->maskDataPresentInBuffer == 0)
            stage->bufferBytesFilled = 0;
        if (meta)
            meta->timestamp = stage->timestamp;
    }

    return samplesToReturn;
}

} // namespace

API_EXPORT int CALL_CONV LMS_RecvStream(
    lms_stream_t* stream, void* samples, size_t sample_count, lms_stream_meta_t* meta, unsigned timeout_ms)
{
    if (stream == nullptr || stream->handle < 0)
    {
        return -1;
    }

    std::size_t samplesProduced = 0;
    chrono::microseconds timeout{ timeout_ms * 1000 };
    switch (stream->dataFmt)
    {
    case lms_stream_t::LMS_FMT_F32:
        samplesProduced = ReceiveStream<lime::complex32f_t>(stream, samples, sample_count, meta, timeout);
        break;
    case lms_stream_t::LMS_FMT_I12:
        samplesProduced = ReceiveStream<lime::complex12_t>(stream, samples, sample_count, meta, timeout);
        break;
    case lms_stream_t::LMS_FMT_I16:
        samplesProduced = ReceiveStream<lime::complex16_t>(stream, samples, sample_count, meta, timeout);
        break;
    default:
        break;
    }

    return samplesProduced;
}

namespace {

template<class T>
int SendStream(
    lms_stream_t* stream, const void* samples, size_t sample_count, const lms_stream_meta_t* meta, chrono::microseconds timeout)
{
    SubChannelHandle* handle = reinterpret_cast<SubChannelHandle*>(stream->handle);
    if (handle == nullptr || handle->parent == nullptr)
    {
        return -1;
    }

    const auto direction = stream->isTx ? lime::TRXDir::Tx : lime::TRXDir::Rx;
    if (direction == lime::TRXDir::Rx)
    {
        lime::error("Invalid direction."s);
        return -1;
    }

    // const uint32_t streamChannel = stream->channel & ~LMS_ALIGN_CH_PHASE;
    const std::size_t sampleSize = sizeof(T);

    int samplesToReturn = 0;
    // fill the staging buffer
    StreamStagingBuffers* stage = handle->stagingArea;
    if (!IsSetBit(stage->maskDataPresentInBuffer, handle->channelIndex))
    {
        samplesToReturn = sample_count;
        std::memcpy(stage->buffer[handle->channelIndex].data(), samples, samplesToReturn * sampleSize);
        stage->maskDataPresentInBuffer = InsertBit(stage->maskDataPresentInBuffer, handle->channelIndex);
        stage->bufferBytesFilled = samplesToReturn * sampleSize;
        if (meta)
            stage->timestamp = meta->timestamp;
    }

    if (stage->maskDataPresentInBuffer == stage->maskChannelsActive)
    {
        // all staging buffers ready, submit the batch
        lime::StreamMeta metadata{ 0, false, false };
        if (meta != nullptr)
        {
            metadata.flushPartialPacket = meta->flushPartialPacket;
            metadata.waitForTimestamp = meta->waitForTimestamp;
            metadata.timestamp = meta->timestamp;
        }

        T* src[2] = { reinterpret_cast<T*>(stage->buffer[0].data()), reinterpret_cast<T*>(stage->buffer[1].data()) };
        size_t samplesSent = handle->parent->device->StreamTx(
            handle->parent->moduleIndex, reinterpret_cast<T**>(src), sample_count, &metadata, timeout);
        stage->maskDataPresentInBuffer = 0;
        stage->bufferBytesFilled = 0;

        if (samplesSent != sample_count)
            samplesToReturn = samplesSent;
    }
    return samplesToReturn;
}

} // namespace

API_EXPORT int CALL_CONV LMS_SendStream(
    lms_stream_t* stream, const void* samples, size_t sample_count, const lms_stream_meta_t* meta, unsigned timeout_ms)
{
    if (stream == nullptr || stream->handle < 0)
    {
        return -1;
    }

    int samplesSent = 0;
    chrono::microseconds timeout{ timeout_ms * 1000 };
    switch (stream->dataFmt)
    {
    case lms_stream_t::LMS_FMT_F32:
        samplesSent = SendStream<lime::complex32f_t>(stream, samples, sample_count, meta, timeout);
        break;
    case lms_stream_t::LMS_FMT_I12:
        samplesSent = SendStream<lime::complex12_t>(stream, samples, sample_count, meta, timeout);
        break;
    case lms_stream_t::LMS_FMT_I16:
        samplesSent = SendStream<lime::complex16_t>(stream, samples, sample_count, meta, timeout);
        break;
    default:
        break;
    }

    return samplesSent;
}

API_EXPORT int CALL_CONV LMS_GetStreamStatus(lms_stream_t* stream, lms_stream_status_t* status)
{
    if (stream == nullptr || stream->handle < 0)
    {
        return -1;
    }

    SubChannelHandle* handle = reinterpret_cast<SubChannelHandle*>(stream->handle);
    if (handle == nullptr || handle->parent == nullptr)
    {
        return -1;
    }

    if (status == nullptr)
    {
        return -1;
    }

    lime::StreamStats stats;
    lime::TRXDir direction = stream->isTx ? lime::TRXDir::Tx : lime::TRXDir::Rx;

    switch (direction)
    {
    case lime::TRXDir::Rx:
        handle->parent->device->StreamStatus(handle->parent->moduleIndex, &stats, nullptr);
        break;
    case lime::TRXDir::Tx:
        handle->parent->device->StreamStatus(handle->parent->moduleIndex, nullptr, &stats);
        break;
    default:
        break;
    }

    status->fifoFilledCount = stats.FIFO.usedCount;
    status->fifoSize = stats.FIFO.totalCount;

    handle->parent->statsDeltas.underrun.set(stats.underrun);
    status->underrun = handle->parent->statsDeltas.underrun.delta();
    handle->parent->statsDeltas.underrun.checkpoint();

    handle->parent->statsDeltas.overrun.set(stats.overrun);
    status->overrun = handle->parent->statsDeltas.overrun.delta();
    handle->parent->statsDeltas.overrun.checkpoint();

    handle->parent->statsDeltas.droppedPackets.set(stats.loss);
    status->droppedPackets = handle->parent->statsDeltas.underrun.delta();
    handle->parent->statsDeltas.droppedPackets.checkpoint();

    // status->sampleRate; // Is noted as unused
    status->linkRate = stats.dataRate_Bps;
    status->timestamp = stats.timestamp;

    return 0;
}

API_EXPORT int CALL_CONV LMS_GPIORead(lms_device_t* dev, uint8_t* buffer, size_t len)
{
    LMS_APIDevice* apiDevice = CheckDevice(dev);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    OpStatus status = apiDevice->device->GPIORead(buffer, len);
    return OpStatusToReturnCode(status);
}

API_EXPORT int CALL_CONV LMS_GPIOWrite(lms_device_t* dev, const uint8_t* buffer, size_t len)
{
    LMS_APIDevice* apiDevice = CheckDevice(dev);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    OpStatus status = apiDevice->device->GPIOWrite(buffer, len);
    return OpStatusToReturnCode(status);
}

API_EXPORT int CALL_CONV LMS_GPIODirRead(lms_device_t* dev, uint8_t* buffer, size_t len)
{
    LMS_APIDevice* apiDevice = CheckDevice(dev);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    OpStatus status = apiDevice->device->GPIODirRead(buffer, len);
    return OpStatusToReturnCode(status);
}

API_EXPORT int CALL_CONV LMS_GPIODirWrite(lms_device_t* dev, const uint8_t* buffer, size_t len)
{
    LMS_APIDevice* apiDevice = CheckDevice(dev);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    OpStatus status = apiDevice->device->GPIODirWrite(buffer, len);
    return OpStatusToReturnCode(status);
}

API_EXPORT int CALL_CONV LMS_ReadCustomBoardParam(lms_device_t* device, uint8_t param_id, float_type* val, lms_name_t units)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    std::vector<lime::CustomParameterIO> parameter{ { param_id, *val, units } };
    OpStatus returnValue = apiDevice->device->CustomParameterRead(parameter);

    if (returnValue != OpStatus::Success)
    {
        return -1;
    }

    if (val)
        *val = parameter[0].value;
    if (units != nullptr)
    {
        CopyString(parameter[0].units, units, sizeof(lms_name_t));
    }

    return OpStatusToReturnCode(returnValue);
}

API_EXPORT int CALL_CONV LMS_WriteCustomBoardParam(lms_device_t* device, uint8_t param_id, float_type val, const lms_name_t units)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    std::vector<lime::CustomParameterIO> parameter{ { param_id, val, units } };

    OpStatus status = apiDevice->device->CustomParameterWrite(parameter);
    return OpStatusToReturnCode(status);
}

API_EXPORT const lms_dev_info_t* CALL_CONV LMS_GetDeviceInfo(lms_device_t* device)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return nullptr;
    }

    const auto& descriptor = apiDevice->device->GetDescriptor();

    if (apiDevice->deviceInfo == nullptr)
    {
        apiDevice->deviceInfo = new lms_dev_info_t;
    }

    CopyString(descriptor.name, apiDevice->deviceInfo->deviceName, sizeof(apiDevice->deviceInfo->deviceName));
    CopyString(descriptor.expansionName, apiDevice->deviceInfo->expansionName, sizeof(apiDevice->deviceInfo->expansionName));
    CopyString(descriptor.firmwareVersion, apiDevice->deviceInfo->firmwareVersion, sizeof(apiDevice->deviceInfo->firmwareVersion));
    CopyString(descriptor.hardwareVersion, apiDevice->deviceInfo->hardwareVersion, sizeof(apiDevice->deviceInfo->hardwareVersion));
    CopyString(descriptor.protocolVersion, apiDevice->deviceInfo->protocolVersion, sizeof(apiDevice->deviceInfo->protocolVersion));
    apiDevice->deviceInfo->boardSerialNumber = descriptor.serialNumber;
    std::string combinedGatewareVersion = descriptor.gatewareVersion + "." + descriptor.gatewareRevision;
    CopyString(combinedGatewareVersion, apiDevice->deviceInfo->gatewareVersion, sizeof(apiDevice->deviceInfo->gatewareVersion));
    CopyString(descriptor.gatewareTargetBoard,
        apiDevice->deviceInfo->gatewareTargetBoard,
        sizeof(apiDevice->deviceInfo->gatewareTargetBoard));

    return apiDevice->deviceInfo;
}

API_EXPORT const char* LMS_GetLibraryVersion()
{
    static constexpr std::size_t LIBRARY_VERSION_SIZE = 32;
    static char libraryVersion[LIBRARY_VERSION_SIZE];

    CopyString(lime::GetLibraryVersion(), libraryVersion, sizeof(libraryVersion));
    return libraryVersion;
}

API_EXPORT int CALL_CONV LMS_GetClockFreq(lms_device_t* device, size_t clk_id, float_type* freq)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    if (freq)
        *freq = apiDevice->device->GetClockFreq(clk_id, apiDevice->moduleIndex * 2);
    return *freq > 0 ? 0 : -1;
}

API_EXPORT int CALL_CONV LMS_SetClockFreq(lms_device_t* device, size_t clk_id, float_type freq)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    try
    {
        apiDevice->device->SetClockFreq(clk_id, freq, apiDevice->moduleIndex * 2);
    } catch (...)
    {
        lime::error("Failed to set clock%li.", clk_id);

        return -1;
    }

    return 0;
}

API_EXPORT int CALL_CONV LMS_GetChipTemperature(lms_device_t* dev, size_t ind, float_type* temp)
{
    LMS_APIDevice* apiDevice = CheckDevice(dev);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    if (temp)
    {
        // TODO: replace with generic RFSOC interface
        lime::LMS7002M* chip = reinterpret_cast<LMS7002M*>(apiDevice->device->GetInternalChip(apiDevice->moduleIndex));
        if (!chip)
            return -1;

        *temp = chip->GetTemperature();
    }
    return 0;
}

API_EXPORT int CALL_CONV LMS_Synchronize(lms_device_t* dev, bool toChip)
{
    LMS_APIDevice* apiDevice = CheckDevice(dev);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    try
    {
        apiDevice->device->Synchronize(toChip);
    } catch (...)
    {
        return -1;
    }

    return 0;
}

API_EXPORT int CALL_CONV LMS_EnableCache(lms_device_t* dev, bool enable)
{
    LMS_APIDevice* apiDevice = CheckDevice(dev);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    try
    {
        apiDevice->device->EnableCache(enable);
    } catch (...)
    {
        return -1;
    }

    return 0;
}

API_EXPORT int CALL_CONV LMS_GetLPFBW(lms_device_t* device, bool dir_tx, size_t chan, float_type* bandwidth)
{
    LMS_APIDevice* apiDevice = CheckDevice(device, chan);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    auto direction = dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx;

    if (bandwidth)
        *bandwidth = apiDevice->device->GetLowPassFilter(apiDevice->moduleIndex, direction, chan);

    return 0;
}

API_EXPORT int CALL_CONV LMS_SetLPF(lms_device_t* device, bool dir_tx, size_t chan, bool enabled)
{
    LMS_APIDevice* apiDevice = CheckDevice(device, chan);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    auto direction = dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx;

    try
    {
        apiDevice->device->SetLowPassFilter(
            apiDevice->moduleIndex, direction, chan, apiDevice->lastSavedLPFValue[chan][dir_tx]); // TODO: fix
    } catch (...)
    {
        lime::error("Failed to set %s channel %li LPF.", ToString(direction).c_str(), chan);

        return -1;
    }

    return 0;
}

API_EXPORT int CALL_CONV LMS_GetTestSignal(lms_device_t* device, bool dir_tx, size_t chan, lms_testsig_t* sig)
{
    LMS_APIDevice* apiDevice = CheckDevice(device, chan);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    auto direction = dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx;

    auto testSignal = apiDevice->device->GetTestSignal(apiDevice->moduleIndex, direction, chan);

    if (!testSignal.enabled)
    {
        *sig = LMS_TESTSIG_NONE;
        return 0;
    }

    if (testSignal.dcMode)
    {
        *sig = LMS_TESTSIG_DC;
        return 0;
    }

    if (testSignal.divide == lime::ChannelConfig::Direction::TestSignal::Divide::Div4 &&
        testSignal.scale == lime::ChannelConfig::Direction::TestSignal::Scale::Half)
    {
        *sig = LMS_TESTSIG_NCODIV4;
        return 0;
    }

    if (testSignal.divide == lime::ChannelConfig::Direction::TestSignal::Divide::Div8 &&
        testSignal.scale == lime::ChannelConfig::Direction::TestSignal::Scale::Half)
    {
        *sig = LMS_TESTSIG_NCODIV8;
        return 0;
    }

    if (testSignal.divide == lime::ChannelConfig::Direction::TestSignal::Divide::Div4 &&
        testSignal.scale == lime::ChannelConfig::Direction::TestSignal::Scale::Full)
    {
        *sig = LMS_TESTSIG_NCODIV4F;
        return 0;
    }

    if (testSignal.divide == lime::ChannelConfig::Direction::TestSignal::Divide::Div8 &&
        testSignal.scale == lime::ChannelConfig::Direction::TestSignal::Scale::Full)
    {
        *sig = LMS_TESTSIG_NCODIV8F;
        return 0;
    }

    return -1;
}

API_EXPORT int CALL_CONV LMS_LoadConfig(lms_device_t* device, const char* filename)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    // TODO: check status
    apiDevice->device->LoadConfig(apiDevice->moduleIndex, filename);

    return 0;
}

API_EXPORT int CALL_CONV LMS_SaveConfig(lms_device_t* device, const char* filename)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    // TODO: check status
    apiDevice->device->SaveConfig(apiDevice->moduleIndex, filename);

    return 0;
}

API_EXPORT void LMS_RegisterLogHandler(LMS_LogHandler handler)
{
    if (handler != nullptr)
    {
        lime::registerLogHandler(APIMsgHandler);
        api_msg_handler = handler;
    }
    else
        lime::registerLogHandler(nullptr);
}

API_EXPORT const char* CALL_CONV LMS_GetLastErrorMessage(void)
{
    return lime::GetLastErrorMessageCString();
}

API_EXPORT int CALL_CONV LMS_SetGFIRLPF(lms_device_t* device, bool dir_tx, size_t chan, bool enabled, float_type bandwidth)
{
    LMS_APIDevice* apiDevice = CheckDevice(device, chan);
    if (apiDevice == nullptr)
    {
        return -1;
    }
    auto direction = dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx;

    // TODO: check status
    apiDevice->device->ConfigureGFIR(apiDevice->moduleIndex, direction, chan & 1, { enabled, bandwidth });

    return 0;
}

API_EXPORT int CALL_CONV LMS_SetGFIRCoeff(
    lms_device_t* device, bool dir_tx, size_t chan, lms_gfir_t filt, const float_type* coef, size_t count)
{
    LMS_APIDevice* apiDevice = CheckDevice(device, chan);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    std::vector<double> coefficients(coef, coef + count);

    // TODO: check status
    apiDevice->device->SetGFIRCoefficients(
        apiDevice->moduleIndex, dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx, chan, static_cast<uint8_t>(filt), coefficients);

    return 0;
}

API_EXPORT int CALL_CONV LMS_GetGFIRCoeff(lms_device_t* device, bool dir_tx, size_t chan, lms_gfir_t filt, float_type* coef)
{
    LMS_APIDevice* apiDevice = CheckDevice(device, chan);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    const uint8_t count = filt == LMS_GFIR3 ? 120 : 40;

    // TODO: check status
    auto coefficients = apiDevice->device->GetGFIRCoefficients(
        apiDevice->moduleIndex, dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx, chan, static_cast<uint8_t>(filt));

    for (std::size_t i = 0; i < count; ++i)
    {
        coef[i] = coefficients.at(i);
    }

    return 0;
}

API_EXPORT int CALL_CONV LMS_SetGFIR(lms_device_t* device, bool dir_tx, size_t chan, lms_gfir_t filt, bool enabled)
{
    LMS_APIDevice* apiDevice = CheckDevice(device, chan);
    if (apiDevice == nullptr)
    {
        return -1;
    }
    auto direction = dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx;

    // TODO: check status
    apiDevice->device->SetGFIR(apiDevice->moduleIndex, direction, chan, filt, enabled);
    return 0;
}

API_EXPORT int CALL_CONV LMS_ReadParam(lms_device_t* device, struct LMS7Parameter param, uint16_t* val)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    if (val)
        *val = apiDevice->device->GetParameter(apiDevice->moduleIndex, 0, param.address, param.msb, param.lsb);

    return 0;
}

API_EXPORT int CALL_CONV LMS_WriteParam(lms_device_t* device, struct LMS7Parameter param, uint16_t val)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    // TODO: check status
    apiDevice->device->SetParameter(apiDevice->moduleIndex, 0, param.address, param.msb, param.lsb, val);

    return 0;
}

API_EXPORT int CALL_CONV LMS_SetNCOFrequency(lms_device_t* device, bool dir_tx, size_t ch, const float_type* freq, float_type pho)
{
    LMS_APIDevice* apiDevice = CheckDevice(device, ch);
    if (apiDevice == nullptr)
    {
        return -1;
    }
    auto direction = dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx;

    for (int i = 0; i < LMS_NCO_VAL_COUNT; ++i)
    {
        // TODO: check status
        apiDevice->device->SetNCOFrequency(apiDevice->moduleIndex, direction, ch, i, freq[i], pho);
    }

    return 0;
}

API_EXPORT int CALL_CONV LMS_GetNCOFrequency(lms_device_t* device, bool dir_tx, size_t chan, float_type* freq, float_type* pho)
{
    LMS_APIDevice* apiDevice = CheckDevice(device, chan);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    auto direction = dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx;
    double phaseOffset = 0.0;

    for (int i = 0; i < LMS_NCO_VAL_COUNT; ++i)
    {
        // TODO: check status
        freq[i] = apiDevice->device->GetNCOFrequency(apiDevice->moduleIndex, direction, chan, i, phaseOffset);
    }

    if (pho != nullptr)
    {
        *pho = phaseOffset;
    }

    return 0;
}

API_EXPORT int CALL_CONV LMS_SetNCOPhase(lms_device_t* device, bool dir_tx, size_t ch, const float_type* phase, float_type fcw)
{
    LMS_APIDevice* apiDevice = CheckDevice(device, ch);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    auto direction = dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx;
    // TODO: check status
    apiDevice->device->SetNCOFrequency(apiDevice->moduleIndex, direction, ch, 0, fcw);

    if (phase != nullptr)
    {
        for (unsigned i = 0; i < LMS_NCO_VAL_COUNT; i++)
        {
            uint16_t addr = dir_tx ? 0x0244 : 0x0444;
            uint16_t pho = static_cast<uint16_t>(65536 * (phase[i] / 360));
            apiDevice->device->WriteRegister(apiDevice->moduleIndex, addr + i, pho);
        }

        auto& selectionParameter = dir_tx ? SEL_TX : SEL_RX;
        apiDevice->device->SetParameter(
            apiDevice->moduleIndex, ch, selectionParameter.address, selectionParameter.msb, selectionParameter.lsb, 0);
    }

    return 0;
}

API_EXPORT int CALL_CONV LMS_GetNCOPhase(lms_device_t* device, bool dir_tx, size_t ch, float_type* phase, float_type* fcw)
{
    LMS_APIDevice* apiDevice = CheckDevice(device, ch);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    auto direction = dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx;

    if (phase != nullptr)
    {
        apiDevice->device->SetParameter(apiDevice->moduleIndex, ch, "MAC"s, ch);

        for (std::size_t i = 0; i < LMS_NCO_VAL_COUNT; ++i)
        {
            uint16_t addr = dir_tx ? 0x0244 : 0x0444;
            uint16_t pho = apiDevice->device->ReadRegister(apiDevice->moduleIndex, addr + i);

            phase[i] = 360 * pho / 65536.0;
        }
    }

    if (fcw != nullptr)
    {
        double phaseOffset = 0.0;
        *fcw = apiDevice->device->GetNCOFrequency(apiDevice->moduleIndex, direction, ch, 0, phaseOffset);
    }

    return 0;
}

API_EXPORT int CALL_CONV LMS_SetNCOIndex(lms_device_t* device, bool dir_tx, size_t chan, int ind, bool down)
{
    LMS_APIDevice* apiDevice = CheckDevice(device, chan);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    auto direction = dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx;

    OpStatus returnValue = apiDevice->device->SetNCOIndex(apiDevice->moduleIndex, direction, chan, ind, down);
    if (returnValue != OpStatus::Success)
    {
        return -1;
    }

    return 0;
}

API_EXPORT int CALL_CONV LMS_GetNCOIndex(lms_device_t* device, bool dir_tx, size_t chan)
{
    LMS_APIDevice* apiDevice = CheckDevice(device, chan);
    if (apiDevice == nullptr)
    {
        return -1;
    }
    auto direction = dir_tx ? lime::TRXDir::Tx : lime::TRXDir::Rx;

    return apiDevice->device->GetNCOIndex(apiDevice->moduleIndex, direction, chan);
}

API_EXPORT int CALL_CONV LMS_WriteLMSReg(lms_device_t* device, uint32_t address, uint16_t val)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    try
    {
        apiDevice->device->WriteRegister(apiDevice->moduleIndex, address, val);
    } catch (...)
    {
        return lime::error("Failed to write register at %04X.", address);
    }

    return 0;
}

API_EXPORT int CALL_CONV LMS_ReadLMSReg(lms_device_t* device, uint32_t address, uint16_t* val)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    try
    {
        if (val)
            *val = apiDevice->device->ReadRegister(apiDevice->moduleIndex, address);
    } catch (...)
    {
        return lime::error("Failed to read register at %04X.", address);
    }

    return 0;
}

API_EXPORT int CALL_CONV LMS_WriteFPGAReg(lms_device_t* device, uint32_t address, uint16_t val)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    try
    {
        apiDevice->device->WriteRegister(apiDevice->moduleIndex, address, val, true);
    } catch (...)
    {
        return lime::error("Failed to write register at %04X.", address);
    }

    return 0;
}

API_EXPORT int CALL_CONV LMS_ReadFPGAReg(lms_device_t* device, uint32_t address, uint16_t* val)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    try
    {
        if (val)
            *val = apiDevice->device->ReadRegister(apiDevice->moduleIndex, address, true);
    } catch (...)
    {
        return lime::error("Failed to read register at %04X.", address);
    }

    return 0;
}

API_EXPORT int CALL_CONV LMS_UploadWFM(lms_device_t* device, const void** samples, uint8_t chCount, size_t sample_count, int format)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    auto config = apiDevice->lastSavedStreamConfig;

    lime::DataFormat dataFormat;
    switch (format)
    {
    case 0:
        dataFormat = lime::DataFormat::I12;
        break;
    case 1:
        dataFormat = lime::DataFormat::I16;
        break;
    case 2:
        dataFormat = lime::DataFormat::F32;
        break;
    default:
        dataFormat = lime::DataFormat::I12;
        break;
    }

    config.format = dataFormat;

    OpStatus status = apiDevice->device->UploadTxWaveform(config, apiDevice->moduleIndex, samples, sample_count);
    return OpStatusToReturnCode(status);
}

API_EXPORT int CALL_CONV LMS_EnableTxWFM(lms_device_t* device, unsigned ch, bool active)
{
    uint16_t regAddr = 0x000D;
    uint16_t regValue = 0;

    int status = LMS_WriteFPGAReg(device, 0xFFFF, 1 << (ch / 2));
    if (status != 0)
    {
        return status;
    }

    status = LMS_ReadFPGAReg(device, regAddr, &regValue);
    if (status != 0)
    {
        return status;
    }

    regValue = regValue & ~0x6; //clear WFM_LOAD, WFM_PLAY
    regValue |= active << 1;
    status = LMS_WriteFPGAReg(device, regAddr, regValue);
    return status;
}

API_EXPORT int CALL_CONV LMS_GetProgramModes(lms_device_t* device, lms_name_t* list)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    const auto& memoryDevices = apiDevice->device->GetDescriptor().memoryDevices;
    if (list != nullptr)
    {
        std::size_t index = 0;

        for (const auto& memoryDevice : memoryDevices)
        {
            CopyString(memoryDevice.first, list[index++], sizeof(lms_name_t));
        }
    }

    return memoryDevices.size();
}

API_EXPORT int CALL_CONV LMS_Program(
    lms_device_t* device, const char* data, size_t size, const lms_name_t mode, lms_prog_callback_t callback)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    const std::string prog_mode{ mode };

    try
    {
        programmingCallback = callback;

        const auto& memoryDevice = apiDevice->device->GetDescriptor().memoryDevices.at(prog_mode);

        OpStatus status =
            memoryDevice->ownerDevice->UploadMemory(memoryDevice->memoryDeviceType, 0, data, size, ProgrammingCallback);
        return OpStatusToReturnCode(status);
    } catch (std::out_of_range& e)
    {
        lime::error("Invalid programming mode."s);

        return -1;
    }
}

API_EXPORT int CALL_CONV LMS_VCTCXOWrite(lms_device_t* device, uint16_t val)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    std::vector<lime::CustomParameterIO> parameter{ { BOARD_PARAM_DAC, static_cast<double>(val), ""s } };
    if (apiDevice->device->CustomParameterWrite(parameter) != OpStatus::Success)
    {
        return -1;
    }

    auto memoryDevice = lime::eMemoryDevice::EEPROM;
    try
    {
        const auto& dataStorage = apiDevice->device->GetDescriptor().memoryDevices.at(ToString(memoryDevice));
        try
        {
            const auto& region = dataStorage->regions.at("VCTCXO_DAC"s);
            OpStatus status = apiDevice->device->MemoryWrite(dataStorage, region, &val);
            return OpStatusToReturnCode(status);
        } catch (std::out_of_range& e)
        {
            lime::error("VCTCXO address not found."s);

            return -1;
        }
    } catch (std::out_of_range& e)
    {
        lime::error("EEPROM not found."s);

        return -1;
    } catch (...)
    {
        return -1;
    }
}

namespace {

static int VCTCXOReadFallbackPath(LMS_APIDevice* apiDevice, uint16_t* val)
{
    std::vector<lime::CustomParameterIO> parameters{ { BOARD_PARAM_DAC, 0, ""s } };

    OpStatus status = apiDevice->device->CustomParameterRead(parameters);
    if (status != OpStatus::Success)
        return OpStatusToReturnCode(status);

    if (val)
        *val = parameters.at(0).value;
    return 0;
}

} // namespace

API_EXPORT int CALL_CONV LMS_VCTCXORead(lms_device_t* device, uint16_t* val)
{
    LMS_APIDevice* apiDevice = CheckDevice(device);
    if (apiDevice == nullptr)
    {
        return -1;
    }

    auto memoryDevice = lime::eMemoryDevice::EEPROM;
    try
    {
        const auto& dataStorage = apiDevice->device->GetDescriptor().memoryDevices.at(ToString(memoryDevice));
        try
        {
            const auto& region = dataStorage->regions.at("VCTCXO_DAC"s);

            OpStatus status = apiDevice->device->MemoryRead(dataStorage, region, val);
            return OpStatusToReturnCode(status);
        } catch (std::out_of_range& e)
        {
            lime::error("VCTCXO address not found."s);
            return VCTCXOReadFallbackPath(apiDevice, val);
        }
    } catch (std::out_of_range& e)
    {
        lime::warning("EEPROM not found."s);
        return VCTCXOReadFallbackPath(apiDevice, val);
    } catch (...)
    {
        return -1;
    }
}
