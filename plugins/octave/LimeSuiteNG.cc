#include <octave/oct.h>
#include <octave/Cell.h>
#include <octave/ov-struct.h>

#include "limesuiteng/complex.h"
#include "limesuiteng/DeviceHandle.h"
#include "limesuiteng/DeviceRegistry.h"
#include "limesuiteng/Logger.h"
#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/StreamConfig.h"

#include <algorithm>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

constexpr uint8_t maxChCnt{ 2U };
constexpr float scaleFactor{ 32768.0F };

lime::SDRDevice* lmsDev = nullptr;
bool IsWFMRunning = false;

lime::complex16_t** rxBuffers = nullptr;
uint8_t rxMaxChannel = 0;

lime::complex16_t** txBuffers = nullptr;
uint8_t txMaxChannel = 0;

void FreeResources()
{
    if (lmsDev)
    {
        if (IsWFMRunning)
            lmsDev->EnableTxWaveform(0, 0, false);
        lmsDev->StreamStop(0);
        lime::DeviceRegistry::freeDevice(lmsDev);
        lmsDev = nullptr;
    }

    if (rxBuffers != nullptr)
    {
        for (uint8_t i = 0; i <= rxMaxChannel; ++i)
        {
            delete[] rxBuffers[i];
        }
    }

    delete[] rxBuffers;
    rxBuffers = nullptr;

    if (txBuffers != nullptr)
    {
        for (uint8_t i = 0; i <= txMaxChannel; ++i)
        {
            delete[] txBuffers[i];
        }
    }

    delete[] txBuffers;
    txBuffers = nullptr;
}

void PrintDeviceInfo()
{
    if (lmsDev == nullptr)
        return;
    const auto& descriptor{ lmsDev->GetDescriptor() };

    octave_stdout << "Connected to device: " << descriptor.name << " FW: " << descriptor.firmwareVersion
                  << " HW: " << descriptor.hardwareVersion << " Protocol: " << descriptor.protocolVersion
                  << " GW: " << descriptor.gatewareVersion << std::endl;
}

static void LogHandler(lime::LogLevel level, const std::string& msg)
{
    const std::unordered_map<lime::LogLevel, std::string_view> levelText{
        { lime::LogLevel::Critical, "CRITICAL: "sv },
        { lime::LogLevel::Error, "ERROR: "sv },
        { lime::LogLevel::Warning, "WARNING: "sv },
        { lime::LogLevel::Info, "INFO: "sv },
        { lime::LogLevel::Debug, "DEBUG: "sv },
        { lime::LogLevel::Verbose, "VERBOSE: "sv },
    };
    if (level >= lime::LogLevel::Debug)
        return; //do not output debug messages
    octave_stdout << levelText.at(level) << msg << std::endl;
}

DEFUN_DLD(LimeGetDeviceList, args, nargout, "LIST = LimeGetDeviceList() - Returns available device list")
{
    if (args.length() != 0)
    {
        print_usage();
        return octave_value(-1);
    }

    const auto& devices = lime::DeviceRegistry::enumerate();
    Cell c;
    for (std::size_t i = 0; i < devices.size(); ++i)
    {
        c.insert(octave_value(devices[i].Serialize(), '"'), i, 0);
    }
    return octave_value(c);
}

DEFUN_DLD(LimeInitialize, args, nargout, "LimeInitialize(DEV) - connect to device and allocate buffer memory\n\
DEV [optional] - device name to connect, obtained via LimeGetDeviceList()")
{
    FreeResources();

    const int nargin = args.length();
    if (args.length() > 1)
    {
        print_usage();
        return octave_value(-1);
    }

    std::string deviceName = ""s;
    if (nargin > 0)
    {
        deviceName = args(0).string_value();
    }

    lmsDev = lime::DeviceRegistry::makeDevice({ deviceName });
    if (lmsDev == nullptr)
    {
        return octave_value(-1);
    }

    lime::registerLogHandler(LogHandler);

    const auto status = lmsDev->Synchronize(false);
    if (status != lime::OpStatus::Success)
    {
        return octave_value(static_cast<uint8_t>(status));
    }
    PrintDeviceInfo();

    return octave_value(0);
}

DEFUN_DLD(LimeDestroy, args, nargout, "LimeDestroy() - Stop all streams, deallocate memory and disconnect device")
{
    FreeResources();
    return octave_value_list();
}

DEFUN_DLD(LimeLoadConfig, args, nargout, "LimeLoadConfig(FILENAME) - Load configuration from file FILENAME")
{
    if (lmsDev == nullptr)
    {
        octave_stdout << "LimeSuite not initialized" << std::endl;
        return octave_value(-1);
    }

    const int nargin = args.length();
    if (nargin != 1)
    {
        print_usage();
        return octave_value(-1);
    }

    const std::string filename = args(0).string_value();
    octave_stdout << "LimeLoadConfig loading: " << filename << std::endl;

    if (auto status = lmsDev->LoadConfig(0, filename); status != lime::OpStatus::Success)
    {
        return octave_value(-1);
    }

    const uint8_t chCnt = std::min(lmsDev->GetDescriptor().rfSOC.at(0).channelCount, maxChCnt);
    for (uint8_t ch = 0; ch < chCnt; ++ch) //set antenna to update RF switches
    {
        uint8_t ant = lmsDev->GetAntenna(0, lime::TRXDir::Rx, ch);
        // int ant = LMS_GetAntenna(lmsDev, LMS_CH_RX, ch);
        if (ant < 0 || lmsDev->SetAntenna(0, lime::TRXDir::Rx, ch, ant) != lime::OpStatus::Success)
            octave_stdout << "Error setting Rx antenna for ch: " << ch << std::endl;

        ant = lmsDev->GetAntenna(0, lime::TRXDir::Tx, ch);
        if (ant < 0 || lmsDev->SetAntenna(0, lime::TRXDir::Tx, ch, ant) != lime::OpStatus::Success)
            octave_stdout << "Error setting Tx antenna for ch: " << ch << std::endl;
    }

    octave_stdout << "Config loaded successfully: " << filename << std::endl;

    return octave_value(0);
}

lime::TRXDir GetDirection(std::string_view input)
{
    if (input == "rx"sv)
    {
        return lime::TRXDir::Rx;
    }
    if (input == "tx"sv)
    {
        return lime::TRXDir::Tx;
    }

    throw std::runtime_error("Invalid direction");
}

uint8_t GetChannelNumber(const std::string& input)
{
    unsigned int ret = 0;

    if (!input.empty())
    {
        std::stringstream ss{ input };
        ss >> ret;
    }

    return ret;
}

std::pair<lime::TRXDir, uint8_t> ParseDirectionAndChannel(const std::string& input)
{
    const std::string direction{ input.substr(0, 2) };
    const std::string channel{ input.substr(2) };

    return { GetDirection(direction), GetChannelNumber(channel) };
}

DEFUN_DLD(
    LimeStartStreaming, args, nargout, "LimeStartStreaming(FIFOSIZE, CHANNELS) - starts sample streaming from selected channels\n\
 FIFOSIZE [optional] - buffer size in samples to be used by library (default: 4 MSamples)\
 CHANNELS [optional] - array of channels to be used [rx0 ; rx1 ; tx0 ; tx1] (default: rx0)")
{
    if (lmsDev == nullptr)
    {
        octave_stdout << "LimeSuite not initialized" << std::endl;
        return octave_value(-1);
    }

    const int nargin = args.length();
    if (nargin > 2)
    {
        print_usage();
        return octave_value(-1);
    }

    int fifoSize = 4 * 1024 * 1024; //4MS
    if (nargin > 0 && args(0).int_value() > 0)
        fifoSize = args(0).int_value();

    lime::StreamConfig streamConfig{};
    streamConfig.format = lime::DataFormat::I16;
    streamConfig.bufferSize = fifoSize;

    if (nargin == 2)
    {
        int rowCnt = args(1).char_matrix_value().rows();
        rowCnt = std::min(rowCnt, maxChCnt * 2);
        for (int i = 0; i < rowCnt; ++i)
        {
            const std::string entry = args(1).char_matrix_value().row_as_string(i);

            try
            {
                const auto [dir, ch] = ParseDirectionAndChannel(entry);
                if (ch >= lmsDev->GetDescriptor().rfSOC.at(0).channelCount)
                {
                    throw std::runtime_error("Invalid channel number"s);
                }
                streamConfig.channels[dir].push_back(ch);
            } catch (...)
            {
                octave_stdout << "Invalid channel parameter" << std::endl;
                return octave_value(-1);
            }
        }
    }
    else
    {
        streamConfig.channels[lime::TRXDir::Rx].push_back(0);
    }

    if (!streamConfig.channels.at(lime::TRXDir::Rx).empty())
    {
        const auto& channels = streamConfig.channels.at(lime::TRXDir::Rx);
        rxMaxChannel = *std::max_element(channels.begin(), channels.end());
        rxBuffers = new lime::complex16_t*[rxMaxChannel + 1];

        for (uint8_t i = 0; i <= rxMaxChannel; ++i)
        {
            rxBuffers[i] = new lime::complex16_t[fifoSize];
        }
    }

    if (!streamConfig.channels.at(lime::TRXDir::Tx).empty())
    {
        const auto& channels = streamConfig.channels.at(lime::TRXDir::Tx);
        txMaxChannel = *std::max_element(channels.begin(), channels.end());
        txBuffers = new lime::complex16_t*[txMaxChannel + 1];

        for (uint8_t i = 0; i <= txMaxChannel; ++i)
        {
            txBuffers[i] = new lime::complex16_t[fifoSize];
        }
    }

    lmsDev->StreamSetup(streamConfig, 0);
    lmsDev->StreamStart(0);

    return octave_value_list();
}

DEFUN_DLD(LimeStopStreaming, args, nargout, "LimeStopStreaming() - Stop Receiver and Transmitter threads")
{
    if (lmsDev == nullptr)
    {
        octave_stdout << "LimeSuite not initialized" << std::endl;
        return octave_value(-1);
    }
    octave_stdout << "StopStreaming" << std::endl;
    lmsDev->StreamStop(0);
    return octave_value_list();
}

octave_value ReceiveSamples(int samplesToReceive, int offset = 0)
{
    Complex val{ 0.0F, 0.0F };
    ComplexMatrix iqData{ { rxMaxChannel + 1, samplesToReceive }, val }; // index 0 to N-1

    int samplesCollected = 0;
    int retries = 5;
    while (samplesCollected < samplesToReceive && retries--)
    {
        int samplesToRead = samplesToReceive - samplesCollected;

        int samplesRead = lmsDev->StreamRx(0, rxBuffers, samplesToReceive, nullptr);
        if (samplesRead <= 0)
        {
            octave_stdout << "Error receiving samples" << std::endl;
            return octave_value(-1);
        }

        offset -= samplesRead;
        offset = std::max(offset, 0);

        for (uint8_t ch = 0; ch <= rxMaxChannel; ++ch)
        {
            for (int i = offset; i < samplesRead; ++i)
            {
                iqData(ch, samplesCollected + i) = { rxBuffers[ch][i].real() / scaleFactor, rxBuffers[ch][i].imag() / scaleFactor };
            }
        }
        samplesCollected += samplesRead;
    }
    return octave_value(iqData);
}

DEFUN_DLD(LimeReceiveSamples, args, , "SIGNAL = LimeReceiveSamples(N) - receive N samples from all active Rx channels.")
{
    if (!rxBuffers)
    {
        octave_stdout << "Rx streaming not initialized" << std::endl;
        return octave_value_list();
    }

    const int nargin = args.length();
    if (nargin != 1)
    {
        print_usage();
        return octave_value_list();
    }

    const int samplesToReceive = args(0).int_value();
    return ReceiveSamples(samplesToReceive);
}

uint32_t TransmitSamples(const ComplexMatrix& iqData)
{
    const dim_vector iqDataSize = iqData.dims();
    const int samplesCount = iqDataSize(1);

    for (uint8_t ch = 0; ch <= txMaxChannel; ++ch)
    {
        for (int i = 0; i < samplesCount; ++i)
        {
            octave_value iqDatum = scaleFactor * iqData(ch, i);
            Complex iqDatum2 = iqDatum.complex_value();
            short i_sample = iqDatum2.real(); //
            short q_sample = iqDatum2.imag(); //
            txBuffers[ch][i].real(i_sample);
            txBuffers[ch][i].imag(q_sample);
        }
    }

    lime::StreamMeta meta{ 0, false, false };
    return lmsDev->StreamTx(0, txBuffers, samplesCount, &meta);
}

DEFUN_DLD(LimeTransmitSamples, args, , "LimeTransmitSamples(SIGNAL) - sends normalized complex SIGNAL to all active Tx channels.")
{
    if (!txBuffers)
    {
        octave_stdout << "Tx streaming not initialized" << std::endl;
        return octave_value(-1);
    }

    const int nargin = args.length();
    if (nargin != 1)
    {
        print_usage();
        return octave_value(-1);
    }

    ComplexMatrix iqData{ args(0).complex_matrix_value() };
    return octave_value(TransmitSamples(iqData));
}

DEFUN_DLD(LimeTransceiveSamples,
    args,
    ,
    "RXSIGNAL = LimeTransceiveSamples(TXSIGNAL, RXOFFSET) - transmit TXSIGNAL and receive RXSIGNAL (same length as TXSIGNAL).\n\
 RXOFFSET [optional] - number of samples to skip at the beginning of receive (default 0)")
{
    if (!rxBuffers)
    {
        octave_stdout << "Rx streaming not initialized" << std::endl;
        return octave_value_list();
    }
    if (!txBuffers)
    {
        octave_stdout << "Tx streaming not initialized" << std::endl;
        return octave_value_list();
    }

    const int nargin = args.length();
    if (nargin == 0 || nargin > 2)
    {
        print_usage();
        return octave_value_list();
    }

    //transmit part
    const ComplexMatrix iqDataTx = args(0).complex_matrix_value();
    const dim_vector iqDataSize = iqDataTx.dims();
    const int samplesCount = iqDataSize(1);
    const int samplesWrite = TransmitSamples(iqDataTx);

    if (samplesWrite != samplesCount)
        octave_stdout << "Error transmitting samples: send " << samplesWrite + "/" + samplesCount << std::endl;

    //Receive part
    int offset = nargin > 1 ? args(1).int_value() : 0;
    if (offset < 0)
    {
        octave_stdout << "Invalid RXOFFSET value" << std::endl;
        offset = 0;
    }

    return octave_value(ReceiveSamples(samplesCount, offset));
}

DEFUN_DLD(LimeLoopWFMStart, args, , "LimeLoopWFMStart(SIGNAL) - upload SIGNAL to device RAM for repeated transmitting")
{
    if (lmsDev == nullptr)
    {
        octave_stdout << "LimeSuite not initialized" << std::endl;
        return octave_value(-1);
    }

    const int nargin = args.length();
    if (nargin != 2 && nargin != 1)
    {
        print_usage();
        return octave_value_list();
    }

    const int chCount = nargin;

    ComplexRowVector iqData = args(0).complex_row_vector_value();
    dim_vector iqDataSize = iqData.dims();
    int samplesCount = iqDataSize(0) > iqDataSize(1) ? iqDataSize(0) : iqDataSize(1);
    int wfmLength = samplesCount;

    lime::complex16_t** const wfmBuffers = new lime::complex16_t*[chCount];
    for (int i = 0; i < chCount; ++i)
        wfmBuffers[i] = new lime::complex16_t[samplesCount];

    for (int ch = 0; ch < chCount; ++ch)
        for (int i = 0; i < samplesCount; ++i)
        {
            const octave_value iqDatum = 2047 * iqData(i);
            const Complex iqDatum2 = iqDatum.complex_value();
            const float i_sample = iqDatum2.real();
            const float q_sample = iqDatum2.imag();
            wfmBuffers[ch][i].real(i_sample);
            wfmBuffers[ch][i].imag(q_sample);
        }

    lime::StreamConfig streamConfig{};
    streamConfig.format = lime::DataFormat::I16;
    streamConfig.linkFormat = lime::DataFormat::I16;

    for (int i = 0; i < chCount; ++i)
    {
        streamConfig.channels.at(lime::TRXDir::Tx).push_back(i);
    }

    //NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
    lmsDev->UploadTxWaveform(streamConfig, 0, const_cast<const void**>(reinterpret_cast<void**>(wfmBuffers)), samplesCount);
    lmsDev->EnableTxWaveform(0, 0, true);
    for (int i = 0; i < chCount; ++i)
        delete wfmBuffers[i];
    delete wfmBuffers;
    IsWFMRunning = true;
    return octave_value_list();
}

DEFUN_DLD(LimeLoopWFMStop, args, , "LimeTxLoopWFMStop() - stop transmitting samples from device RAM")
{
    if (lmsDev == nullptr)
    {
        octave_stdout << "LimeSuite not initialized" << std::endl;
        return octave_value(-1);
    }
    if (IsWFMRunning)
        lmsDev->EnableTxWaveform(0, 0, false);
    IsWFMRunning = false;
    return octave_value();
}

DEFUN_DLD(LimeGetStreamStatus, args, nargout, "LimeGetStreamStatus() - Get Stream Status")
{
    if (lmsDev == nullptr)
    {
        octave_stdout << "LimeSuite not initialized" << std::endl;
        return octave_value(-1);
    }

    octave_scalar_map st;

    if (rxBuffers != nullptr)
    {
        lime::StreamStats status{};
        lmsDev->StreamStatus(0, &status, nullptr);

        st.assign("fifo_size", status.FIFO.totalCount);
        st.assign("rx_data_rate", status.dataRate_Bps);
        const std::string ch = "rx"s;
        st.assign(ch + "_fifo_filled", status.FIFO.usedCount);
        st.assign(ch + "_fifo_overruns", status.overrun);
        st.assign(ch + "_lost_packets", status.loss);
    }

    if (txBuffers != nullptr)
    {
        lime::StreamStats status{};
        lmsDev->StreamStatus(0, nullptr, &status);

        st.assign("fifo_size", status.FIFO.totalCount);
        st.assign("tx_data_rate", status.dataRate_Bps);
        const std::string ch = "tx"s;
        st.assign(ch + "_fifo_filled", status.FIFO.usedCount);
        st.assign(ch + "_fifo_underrun", status.underrun);
    }

    return octave_value(st);
}

class ResourceDeallocator
{
  public:
    ResourceDeallocator() = default;
    ~ResourceDeallocator() { FreeResources(); };
};

ResourceDeallocator gResources;
