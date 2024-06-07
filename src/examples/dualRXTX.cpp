/**
    @file   dualRXTX.cpp
    @author Lime Microsystems (www.limemicro.com)
    @brief  minimal RX loopback to Tx example
 */
#include "limesuiteng/limesuiteng.hpp"
#include <iostream>
#include <chrono>
#include <string_view>
#include <cmath>
#include <signal.h>
#ifdef USE_GNU_PLOT
    #include "gnuPlotPipe.h"
#endif

using namespace lime;
using namespace std::literals::string_view_literals;

static const double frequencyLO = 1.5e9;
float sampleRate = 10e6;
static uint8_t chipIndex = 0; // device might have several RF chips

bool stopProgram(false);
void intHandler(int dummy)
{
    std::cout << "Stopping\n"sv;
    stopProgram = true;
}

static LogLevel logVerbosity = LogLevel::Error;
static void LogCallback(LogLevel lvl, const std::string& msg)
{
    if (lvl > logVerbosity)
        return;
    std::cout << msg << std::endl;
}

int main(int argc, char** argv)
{
    lime::registerLogHandler(LogCallback);
    auto handles = DeviceRegistry::enumerate();
    if (handles.size() == 0)
    {
        std::cout << "No devices found\n"sv;
        return -1;
    }
    std::cout << "Devices found :"sv << std::endl;
    for (size_t i = 0; i < handles.size(); i++)
        std::cout << i << ": "sv << handles[i].Serialize() << std::endl;
    std::cout << std::endl;

    // Use first available device
    SDRDevice* device = DeviceRegistry::makeDevice(handles.at(0));
    if (!device)
    {
        std::cout << "Failed to connect to device"sv << std::endl;
        return -1;
    }
    device->SetMessageLogCallback(LogCallback);
    device->Init();

    // RF parameters
    SDRConfig config;
    for (int c = 0; c < 2; ++c) // MIMO
    {
        config.channel[c].rx.enabled = true;
        config.channel[c].rx.centerFrequency = frequencyLO;
        config.channel[c].rx.sampleRate = sampleRate;
        config.channel[c].rx.oversample = 2;
        config.channel[c].rx.lpf = 0;
        config.channel[c].rx.path = 2; // TODO: replace with string names
        config.channel[c].rx.calibrate = false;

        config.channel[c].tx.enabled = true;
        config.channel[c].tx.sampleRate = sampleRate;
        config.channel[c].tx.oversample = 2;
        config.channel[c].tx.path = 2; // TODO: replace with string names
        config.channel[c].tx.centerFrequency = frequencyLO;
        config.channel[c].tx.calibrate = false;
    }

    // Samples data streaming configuration
    StreamConfig stream;

    stream.channels[TRXDir::Rx] = { 0, 1 };
    stream.channels[TRXDir::Tx] = { 0, 1 };

    stream.format = DataFormat::F32;
    stream.linkFormat = DataFormat::I16;

    signal(SIGINT, intHandler);

    const int samplesInBuffer = 256 * 4;
    complex32f_t** rxSamples = new complex32f_t*[2]; // allocate two channels for simplicity
    for (int i = 0; i < 2; ++i)
        rxSamples[i] = new complex32f_t[samplesInBuffer];

#ifdef USE_GNU_PLOT
    GNUPlotPipe gp;
    gp.write("set size square\n set xrange[-1:1]\n set yrange[-1:1]\n");
#endif

    std::cout << "Configuring device ...\n"sv;
    try
    {
        auto t1 = std::chrono::high_resolution_clock::now();
        device->Configure(config, chipIndex);
        auto t2 = std::chrono::high_resolution_clock::now();
        std::cout << "SDR configured in "sv << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "ms\n"sv;

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

    auto startTime = std::chrono::high_resolution_clock::now();
    auto t1 = startTime;
    auto t2 = t1;

    int totalSamplesReceived = 0;
    uint32_t totalSamplesSent = 0;
    float maxSignalAmplitude = 0;

    StreamMeta rxMeta{};
    while (std::chrono::high_resolution_clock::now() - startTime < std::chrono::seconds(10) && !stopProgram)
    {
        uint32_t samplesRead = device->StreamRx(chipIndex, rxSamples, samplesInBuffer, &rxMeta);
        totalSamplesReceived += samplesRead;

        // process samples
        for (uint32_t n = 0; n < samplesRead; ++n)
        {
            float amplitude = pow(rxSamples[0][n].real(), 2) + pow(rxSamples[0][n].imag(), 2);
            if (amplitude > maxSignalAmplitude)
                maxSignalAmplitude = amplitude;
        }

        StreamMeta txMeta{};
        txMeta.timestamp = rxMeta.timestamp + samplesInBuffer * 64;
        txMeta.waitForTimestamp = true;
        txMeta.flushPartialPacket = false;
        uint32_t samplesSent = device->StreamTx(chipIndex, rxSamples, samplesInBuffer, &txMeta);
        if (samplesSent < 0)
        {
            std::cout << "Failure to send\n"sv;
            break;
        }
        totalSamplesSent += samplesSent;

        t2 = std::chrono::high_resolution_clock::now();
        if (t2 - t1 > std::chrono::seconds(1))
        {
            t1 = t2;
            std::cout << "Total samples received: "sv << totalSamplesReceived << "  signal amplitude: "sv
                      << std::sqrt(maxSignalAmplitude) << "  total samples sent: "sv << totalSamplesSent << std::endl;

#ifdef USE_GNU_PLOT
            gp.write("plot '-' with points title 'ch 0'");
            for (std::size_t c = 1; c < stream.channels.at(TRXDir::Rx).size(); ++c)
                gp.writef(", '-' with points title 'ch %i'\n", c);
            for (std::size_t c = 0; c < stream.channels.at(TRXDir::Rx).size(); ++c)
            {
                for (uint32_t n = 0; n < samplesInBuffer; ++n)
                    gp.writef("%f %f\n", rxSamples[c][n].real(), rxSamples[c][n].imag());
                gp.write("e\n");
                gp.flush();
            }
#endif
            maxSignalAmplitude = 0;
        }
    }
    DeviceRegistry::freeDevice(device);

    for (int i = 0; i < 2; ++i)
        delete[] rxSamples[i];
    delete[] rxSamples;
    return 0;
}
