/**
    @file   basicTX.cpp
    @author Lime Microsystems (www.limemicro.com)
    @brief  minimal TX example
 */

#include "limesuiteng/limesuiteng.hpp"
#include <iostream>
#include <chrono>
#include <string_view>
#include <cmath>
#include <signal.h>
#include "args.hxx"
#include "common.h"

using namespace lime;
using namespace std::literals::string_view_literals;

double frequencyLO = 2e9;
float sampleRate = 10e6;
static uint8_t chipIndex = 0; // device might have several RF chips

bool stopProgram(false);
void intHandler(int dummy)
{
    std::cout << "Stopping\n"sv;
    stopProgram = true;
}

static LogLevel logVerbosity = LogLevel::Verbose;
static void LogCallback(LogLevel lvl, const std::string& msg)
{
    if (lvl > logVerbosity)
        return;
    std::cout << msg << std::endl;
}

int main(int argc, char** argv)
{
    // clang-format off
    args::ArgumentParser            parser("basicTX - minimal TX example", "");
    args::ValueFlag<std::string>    deviceFlag(parser, "device", "Specifies which device to use", {'d', "device"}, "");

    args::Group                     txGroup(parser, "Transmitter"); // NOLINT(cppcoreguidelines-slicing)
    args::ValueFlag<double>         txloFlag(parser, "txlo", "Transmitter center frequency in Hz", {"txlo"});
    args::ValueFlag<std::string>    txpathFlag(parser, "antenna name", "Transmitter antenna path", {"txpath"}, "");
    // clang-format on

    try
    {
        parser.ParseCLI(argc, argv);
    } catch (const args::Help&)
    {
        std::cout << parser;
        return EXIT_SUCCESS;
    } catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    lime::registerLogHandler(LogCallback);

    const std::string devName = args::get(deviceFlag);
    SDRDevice* device = ConnectToFilteredOrDefaultDevice(devName);
    if (!device)
    {
        std::cout << "Failed to connect to device"sv << std::endl;
        return -1;
    }

    std::cout << "Connected to device: " << device->GetDescriptor().name << std::endl;
    device->SetMessageLogCallback(LogCallback);
    device->Init();

    const auto& chipDescriptor = device->GetDescriptor().rfSOC[chipIndex];

    // Default TX path when none is provided via the command lime.
    int txPath = 2;

    const std::string txAntennaName = args::get(txpathFlag);
    if (!txAntennaName.empty())
    {
        txPath = AntennaNameToIndex(chipDescriptor.pathNames.at(TRXDir::Tx), txAntennaName);
        if (txPath < 0)
        {
            DeviceRegistry::freeDevice(device);
            return EXIT_FAILURE;
        }
    }

    std::cout << "Using antenna "sv << chipDescriptor.pathNames.at(TRXDir::Tx).at(txPath) << std::endl;

    if (txloFlag)
    {
        frequencyLO = args::get(txloFlag);
    }
    std::cout << "Transmitter center frequency: " << std::fixed << std::setprecision(3) << frequencyLO / 1e6 << " MHz" << std::endl;

    // RF parameters
    SDRConfig config;
    config.channel[0].tx.enabled = true;
    config.channel[0].rx.enabled = false;
    config.channel[0].rx.centerFrequency = frequencyLO;
    config.channel[0].tx.centerFrequency = frequencyLO;
    config.channel[0].rx.sampleRate = sampleRate;
    config.channel[0].rx.oversample = 2;
    config.channel[0].tx.sampleRate = sampleRate;
    config.channel[0].tx.oversample = 2;
    config.channel[0].tx.lpf = 0;
    config.channel[0].tx.path = txPath;
    config.channel[0].tx.calibrate = true;
    config.channel[0].tx.testSignal.enabled = false;

    std::cout << "Configuring device ...\n"sv;
    try
    {
        auto t1 = std::chrono::high_resolution_clock::now();
        device->Configure(config, chipIndex);
        auto t2 = std::chrono::high_resolution_clock::now();
        std::cout << "SDR configured in "sv << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "ms\n"sv;

        // Samples data streaming configuration
        StreamConfig stream;

        stream.channels[TRXDir::Tx] = { 0 };

        stream.format = DataFormat::F32;
        stream.linkFormat = DataFormat::I16;

        device->StreamSetup(stream, chipIndex);
        device->StreamStart(chipIndex);

    } catch (std::runtime_error& e)
    {
        std::cout << "Failed to configure settings: "sv << e.what() << std::endl;
        return -1;
    } catch (std::logic_error& e)
    {
        std::cout << "Failed to configure settings: "sv << e.what() << std::endl;
        return -1;
    }

    std::cout << "Stream started ...\n"sv;
    signal(SIGINT, intHandler);

    std::vector<std::vector<complex32f_t>> txPattern(2);
    const int txPacketCount = 4;
    const int samplesInPkt = 1024;
    for (size_t i = 0; i < txPattern.size(); ++i)
    {
        txPattern[i].resize(txPacketCount * samplesInPkt);
        for (int j = 0; j < txPacketCount; ++j)
        {
            float src[4] = { 1.0, 0.0, -1.0, 0.0 };
            float ampl = 0.8;
            for (int k = 0; k < samplesInPkt; ++k)
            {
                txPattern[i][j * samplesInPkt + k].imag(src[k & 3] * ampl);
                txPattern[i][j * samplesInPkt + k].real(src[(k + 1) & 3] * ampl);
            }
        }
    }

    std::cout << "Stream started ...\n";
    const lime::complex32f_t* src[2] = { txPattern[0].data(), txPattern[1].data() };
    auto startTime = std::chrono::high_resolution_clock::now();
    auto t1 = startTime;
    auto t2 = t1;

    StreamMeta txMeta{};
    txMeta.timestamp = 0;
    txMeta.waitForTimestamp = true;
    txMeta.flushPartialPacket = true;

    uint32_t totalSamplesSent = 0;

    while (std::chrono::high_resolution_clock::now() - startTime < std::chrono::seconds(10) && !stopProgram) //run for 10 seconds
    {
        uint32_t samplesToSend = samplesInPkt * txPacketCount;
        uint32_t samplesSent = device->StreamTx(chipIndex, src, samplesToSend, &txMeta);
        if (samplesSent < 0)
        {
            std::cout << "Failure to send\n"sv;
            break;
        }
        if (samplesSent > 0)
        {
            txMeta.timestamp += samplesSent;
            totalSamplesSent += samplesSent;
        }
        //Print data rate (once per second)
        t2 = std::chrono::high_resolution_clock::now();
        if (t2 - t1 > std::chrono::seconds(1))
        {
            t1 = std::chrono::high_resolution_clock::now();
            std::cout << "TX total samples sent: "sv << totalSamplesSent << std::endl;
        }
    }

    DeviceRegistry::freeDevice(device);

    return 0;
}
