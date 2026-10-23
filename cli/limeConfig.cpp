#include "common.h"

#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/Logger.h"

#include <cassert>
#include <limits>
#include <cstring>
#include <string_view>
#include "args.hxx"

using namespace std;
using namespace lime;
using namespace lime::cli;
using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

int main(int argc, char** argv)
{
    // clang-format off
    args::ArgumentParser            parser("limeConfig - SDR parameters configuring", "");
    args::HelpFlag                  help(parser, "help", "This help", {'h', "help"});
    args::ValueFlag<std::string>    deviceFlag(parser, "device", "Specifies which device to use", {'d', "device"}, "");
    args::NargsValueFlag<int>       chipFlag(parser, "chip", "Selects destination chip/chips", {'c', "chips"}, {1, static_cast<size_t>(-1)});
    args::ValueFlag<std::string>    logFlag(parser, "log level", "Log verbosity: info, warning, error, verbose, debug", {'l', "log"}, "error");
    args::Flag                      initializeFlag(parser, "", "Reset and fully configure device using default values and the provided parameters. Without this flag only the provided parameters are changed", {'i', "initialize"});
    args::ValueFlag<std::string>    chMaskFlag(parser, "mask", "Bitmask of channels to apply the parameters to (e.g. 0x3). Default: all channels", {"chmask"});

    args::ValueFlag<double>         refclkFlag(parser, "reference clock", "Reference clock in Hz", {"refclk"});
    args::ValueFlag<std::string>    refclkSourceFlag(parser, "reference clock source", "Reference clock input source (internal/external)", {"refclk_source"});
    args::ValueFlag<double>         samplerateFlag(parser, "sample rate", "Sampling rate in Hz", {"samplerate"});

    args::Group                     rxGroup(parser, "Receiver"); // NOLINT(cppcoreguidelines-slicing)
    args::ValueFlag<bool>           rxenFlag(parser, "rx enable", "Enable receiver [0, 1]", {"rxen"});
    args::ValueFlag<double>         rxloFlag(parser, "rxlo", "Receiver center frequency in Hz", {"rxlo"});
    args::ValueFlag<std::string>    rxpathFlag(parser, "antenna name", "Receiver antenna path", {"rxpath"}, "");
    args::ValueFlag<double>         rxlpfFlag(parser, "Hz", "Receiver low pass filter bandwidth in Hz", {"rxlpf"});
    args::ValueFlag<double>         rxgfirFlag(parser, "Hz", "Receiver digital GFIR low pass filter bandwidth in Hz", {"rxgfir"});
    args::ValueFlag<uint32_t>       rxoversampleFlag(parser, "", "Receiver decimation 1,2,4,8...", {"rxoversample"});
    args::ValueFlag<bool>           rxtestsignalFlag(parser, "", "Enables receiver test signal if available", {"rxtestsignal"});
    args::ValueFlag<double>         rxGainFlag(parser, "", "Rx gain", {"rxgain"});
    args::Flag                      rxCalibrateFlag(parser, "", "Calibrates Rx DC and IQ imbalance", {"rxcalibrate"});

    args::Group                     txGroup(parser, "Transmitter"); // NOLINT(cppcoreguidelines-slicing)
    args::ValueFlag<bool>           txenFlag(parser, "tx enable", "Enable transmitter", {"txen"});
    args::ValueFlag<double>         txloFlag(parser, "txlo", "Transmitter center frequency in Hz", {"txlo"});
    args::ValueFlag<std::string>    txpathFlag(parser, "antenna name", "Transmitter antenna path", {"txpath"}, "");
    args::ValueFlag<double>         txlpfFlag(parser, "Hz", "Transmitter low pass filter bandwidth in Hz", {"txlpf"});
    args::ValueFlag<double>         txgfirFlag(parser, "Hz", "Transmitter digital GFIR low pass filter bandwidth in Hz", {"txgfir"});
    args::ValueFlag<uint32_t>       txoversampleFlag(parser, "", "Transmitter interpolation 1,2,4,8...", {"txoversample"});
    args::ValueFlag<bool>           txtestsignalFlag(parser, "", "Enables transmitter test signal if available", {"txtestsignal"});
    args::ValueFlag<double>         txGainFlag(parser, "", "Tx gain", {"txgain"});
    args::Flag                      txCalibrateFlag(parser, "", "Calibrates Tx DC and IQ imbalance", {"txcalibrate"});

    args::ValueFlag<std::string>    iniFlag(parser, "", "Path to LMS7002M .ini configuration file to use as a base", {"ini"}, "");
    // clang-format on

    try
    {
        parser.ParseCLI(argc, argv);
    } catch (const args::Help&)
    {
        cout << parser;
        return EXIT_SUCCESS;
    } catch (const std::exception& e)
    {
        cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    if (argc == 1)
    {
        std::cout << parser;
        return EXIT_SUCCESS;
    }

    const bool doConfigure = refclkFlag || refclkSourceFlag || samplerateFlag || rxenFlag || rxloFlag || rxpathFlag || rxlpfFlag ||
                             rxgfirFlag || rxoversampleFlag || rxtestsignalFlag || rxGainFlag || rxCalibrateFlag || txenFlag ||
                             txloFlag || txpathFlag || txlpfFlag || txgfirFlag || txoversampleFlag || txtestsignalFlag ||
                             txGainFlag || txCalibrateFlag;

    const std::string devName = args::get(deviceFlag);
    const bool initializeBoard = initializeFlag;
    const std::string iniFilename = args::get(iniFlag);
    const std::string rxAntennaName = args::get(rxpathFlag);
    const std::string txAntennaName = args::get(txpathFlag);

    logVerbosity = strToLogLevel(args::get(logFlag));
    std::vector<int> chipIndexes = ParseIntArray(chipFlag);

    uint32_t channelMask = std::numeric_limits<uint32_t>::max();
    if (chMaskFlag)
    {
        try
        {
            channelMask = std::stoul(args::get(chMaskFlag), nullptr, 0);
        } catch (const std::exception&)
        {
            cerr << "Invalid channels mask: "sv << args::get(chMaskFlag) << endl;
            return EXIT_FAILURE;
        }
    }

    int32_t referenceClockSource = 0;
    if (refclkSourceFlag)
    {
        const std::string clksrc = args::get(refclkSourceFlag);
        if (clksrc == "internal")
            referenceClockSource = 0;
        else if (clksrc == "external")
            referenceClockSource = 1;
        else
        {
            cerr << "Invalid reference clock source"sv << endl;
            return EXIT_FAILURE;
        }
        if (!initializeBoard)
        {
            cerr << "Reference clock source can only be changed together with --initialize"sv << endl;
            return EXIT_FAILURE;
        }
    }

    // Channel configuration with defaults and the values provided in the arguments, used for full initialization
    ChannelConfig chConfig;
    // clang-format off
    if (samplerateFlag)
    {
        chConfig.rx.sampleRate = args::get(samplerateFlag);
        chConfig.tx.sampleRate = args::get(samplerateFlag);
    }
    if (rxenFlag)           chConfig.rx.enabled = args::get(rxenFlag);
    if (rxloFlag)           chConfig.rx.centerFrequency = args::get(rxloFlag);
    if (rxlpfFlag)          chConfig.rx.lpf = args::get(rxlpfFlag);
    if (rxgfirFlag)
    {
        chConfig.rx.gfir.bandwidth = args::get(rxgfirFlag);
        chConfig.rx.gfir.enabled = true;
    }
    if (rxoversampleFlag)   chConfig.rx.oversample = args::get(rxoversampleFlag);
    if (rxtestsignalFlag)   chConfig.rx.testSignal.enabled = args::get(rxtestsignalFlag);
    if (rxGainFlag)         chConfig.rx.gain[eGainTypes::GENERIC] = args::get(rxGainFlag);
    if (rxCalibrateFlag)    chConfig.rx.calibrate = CalibrationFlag::DCIQ;

    if (txenFlag)           chConfig.tx.enabled = args::get(txenFlag);
    if (txloFlag)           chConfig.tx.centerFrequency = args::get(txloFlag);
    if (txlpfFlag)          chConfig.tx.lpf = args::get(txlpfFlag);
    if (txgfirFlag)
    {
        chConfig.tx.gfir.bandwidth = args::get(txgfirFlag);
        chConfig.tx.gfir.enabled = true;
    }
    if (txoversampleFlag)   chConfig.tx.oversample = args::get(txoversampleFlag);
    if (txtestsignalFlag)   chConfig.tx.testSignal.enabled = args::get(txtestsignalFlag);
    if (txGainFlag)         chConfig.tx.gain[eGainTypes::GENERIC] = args::get(txGainFlag);
    if (txCalibrateFlag)    chConfig.tx.calibrate = CalibrationFlag::DCIQ;
    // clang-format on

    auto handles = DeviceRegistry::enumerate();
    if (handles.size() == 0)
    {
        cerr << "No devices found"sv << endl;
        return EXIT_FAILURE;
    }

    SDRDevice* device = ConnectToFilteredOrDefaultDevice(devName);
    if (!device)
        return EXIT_FAILURE;

    // if chip index is not specified and device has only one, use it by default
    if (chipIndexes.empty() && device->GetDescriptor().rfSOC.size() == 1)
        chipIndexes.push_back(0);

    device->SetMessageLogCallback(lime::cli::LogCallback);
    lime::registerLogHandler(lime::cli::LogCallback);

    // throws on failure, unsupported parameters are reported but do not stop the rest of the configuration
    auto check = [](OpStatus status, const std::string& what, int channel = -1) {
        if (status == OpStatus::Success)
            return;
        const std::string target = channel >= 0 ? " (channel "s + std::to_string(channel) + ")"s : ""s;
        if (status == OpStatus::NotImplemented || status == OpStatus::NotSupported)
        {
            cerr << "Warning: "sv << what << target << " is not supported by the device"sv << endl;
            return;
        }
        throw std::runtime_error("failed to "s + what + target);
    };

    try
    {
        std::string configFilepath = ""s;
        if (!iniFilename.empty())
        {
            std::string_view cwd{ argv[0] };
            const size_t slash0Pos = cwd.find_last_of("/\\"sv);
            if (slash0Pos != std::string_view::npos)
            {
                cwd = cwd.substr(0, slash0Pos);
            }

            if (iniFilename[0] != '/') // is not global path
                configFilepath = std::string{ cwd } + "/"s;

            configFilepath += iniFilename;

            // .ini file replaces the defaults, so reset the chips before loading it
            if (initializeBoard && device->Init() != OpStatus::Success)
                throw std::runtime_error("failed to initialize device"s);
        }

        for (int moduleId : chipIndexes)
        {
            LMS7002M* chip = static_cast<LMS7002M*>(device->GetInternalChip(moduleId));
            if (!chip)
                throw std::runtime_error("failed to get internal chip "s + std::to_string(moduleId));
            if (!configFilepath.empty() && chip->LoadConfig(configFilepath) != OpStatus::Success)
                throw std::runtime_error("error loading file: "s + configFilepath);

            const auto& chipDescriptor = device->GetDescriptor().rfSOC[moduleId];

            int rxPathIndex = -1;
            int txPathIndex = -1;
            if (!rxAntennaName.empty())
            {
                rxPathIndex = AntennaNameToIndex(chipDescriptor.pathNames.at(TRXDir::Rx), rxAntennaName);
                if (rxPathIndex < 0)
                    throw std::runtime_error("invalid Rx antenna name"s);
            }
            if (!txAntennaName.empty())
            {
                txPathIndex = AntennaNameToIndex(chipDescriptor.pathNames.at(TRXDir::Tx), txAntennaName);
                if (txPathIndex < 0)
                    throw std::runtime_error("invalid Tx antenna name"s);
            }

            std::vector<uint8_t> channels;
            for (int i = 0; i < chipDescriptor.channelCount && i < 32; ++i)
                if (channelMask & (1u << i))
                    channels.push_back(i);

            if (channels.empty())
                throw std::runtime_error("channels mask does not select any channel of chip "s + std::to_string(moduleId));

            if (initializeBoard)
            {
                SDRConfig config;
                config.skipDefaults = !configFilepath.empty();
                config.referenceClockSource = referenceClockSource;
                if (refclkFlag)
                    config.referenceClockFreq = args::get(refclkFlag);

                // channels not selected by the mask keep the default configuration
                for (uint8_t ch : channels)
                {
                    config.channel[ch] = chConfig;
                    if (rxPathIndex >= 0)
                        config.channel[ch].rx.path = rxPathIndex;
                    if (txPathIndex >= 0)
                        config.channel[ch].tx.path = txPathIndex;
                }

                if (device->Configure(config, moduleId) != OpStatus::Success)
                    throw std::runtime_error("failed to configure device (chip "s + std::to_string(moduleId) + ")"s);
                continue;
            }

            if (!doConfigure)
                continue;

            if (refclkFlag)
                check(device->SetClockFreq(static_cast<uint8_t>(LMS7002M::ClockID::CLK_REFERENCE), args::get(refclkFlag), moduleId * 2),
                    "set reference clock"s);

            for (uint8_t ch : channels)
            {
                if (rxenFlag)
                    check(device->EnableChannel(moduleId, TRXDir::Rx, ch, args::get(rxenFlag)), "enable Rx channel"s, ch);
                if (txenFlag)
                    check(device->EnableChannel(moduleId, TRXDir::Tx, ch, args::get(txenFlag)), "enable Tx channel"s, ch);
            }

            // sample rate is common to all channels of the chip
            if (samplerateFlag || rxoversampleFlag || txoversampleFlag)
            {
                const uint8_t ch = channels.front();
                const double sampleRate =
                    samplerateFlag ? args::get(samplerateFlag) : device->GetSampleRate(moduleId, TRXDir::Rx, ch, nullptr);
                const uint8_t oversample = rxoversampleFlag ? args::get(rxoversampleFlag)
                                           : txoversampleFlag ? args::get(txoversampleFlag)
                                                              : 0;
                check(device->SetSampleRate(moduleId, TRXDir::Rx, ch, sampleRate, oversample), "set sample rate"s);
            }

            for (uint8_t ch : channels)
            {
                // clang-format off
                if (rxloFlag)         check(device->SetFrequency(moduleId, TRXDir::Rx, ch, args::get(rxloFlag)), "set Rx LO"s, ch);
                if (txloFlag)         check(device->SetFrequency(moduleId, TRXDir::Tx, ch, args::get(txloFlag)), "set Tx LO"s, ch);
                if (rxPathIndex >= 0) check(device->SetAntenna(moduleId, TRXDir::Rx, ch, rxPathIndex), "set Rx antenna"s, ch);
                if (txPathIndex >= 0) check(device->SetAntenna(moduleId, TRXDir::Tx, ch, txPathIndex), "set Tx antenna"s, ch);
                if (rxlpfFlag)        check(device->SetLowPassFilter(moduleId, TRXDir::Rx, ch, args::get(rxlpfFlag)), "set Rx LPF"s, ch);
                if (txlpfFlag)        check(device->SetLowPassFilter(moduleId, TRXDir::Tx, ch, args::get(txlpfFlag)), "set Tx LPF"s, ch);
                if (rxGainFlag)       check(device->SetGain(moduleId, TRXDir::Rx, ch, eGainTypes::GENERIC, args::get(rxGainFlag)), "set Rx gain"s, ch);
                if (txGainFlag)       check(device->SetGain(moduleId, TRXDir::Tx, ch, eGainTypes::GENERIC, args::get(txGainFlag)), "set Tx gain"s, ch);
                if (rxgfirFlag)       check(device->ConfigureGFIR(moduleId, TRXDir::Rx, ch, chConfig.rx.gfir), "set Rx GFIR"s, ch);
                if (txgfirFlag)       check(device->ConfigureGFIR(moduleId, TRXDir::Tx, ch, chConfig.tx.gfir), "set Tx GFIR"s, ch);
                if (rxtestsignalFlag) check(device->SetTestSignal(moduleId, TRXDir::Rx, ch, chConfig.rx.testSignal), "set Rx test signal"s, ch);
                if (txtestsignalFlag) check(device->SetTestSignal(moduleId, TRXDir::Tx, ch, chConfig.tx.testSignal), "set Tx test signal"s, ch);
                // clang-format on
            }

            // calibrate after all the parameters have been applied
            for (uint8_t ch : channels)
            {
                if (rxCalibrateFlag)
                    check(device->Calibrate(moduleId, TRXDir::Rx, ch, device->GetSampleRate(moduleId, TRXDir::Rx, ch, nullptr)),
                        "calibrate Rx"s,
                        ch);
                if (txCalibrateFlag)
                    check(device->Calibrate(moduleId, TRXDir::Tx, ch, device->GetSampleRate(moduleId, TRXDir::Tx, ch, nullptr)),
                        "calibrate Tx"s,
                        ch);
            }
        }
    } catch (std::runtime_error& e)
    {
        DeviceRegistry::freeDevice(device);
        cerr << "Config failed: "sv << e.what() << endl;
        return EXIT_FAILURE;
    } catch (std::logic_error& e)
    {
        DeviceRegistry::freeDevice(device);
        cerr << "Config failed: "sv << e.what() << endl;
        return EXIT_FAILURE;
    }

    DeviceRegistry::freeDevice(device);

    return EXIT_SUCCESS;
}
