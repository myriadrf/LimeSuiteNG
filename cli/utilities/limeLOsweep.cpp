// #include "boards/LMS7002M_SDRDevice.h"
#include "limesuiteng/DeviceHandle.h"
#include "limesuiteng/DeviceRegistry.h"
#include "limesuiteng/Logger.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/SDRDescriptor.h"

#include <iostream>
#include <iomanip>
#include <string>
#include <algorithm>
#include <cctype>
#include <fstream>
#include <chrono>
#include <cmath>
#include <string_view>
#include "args.hxx"
#include <thread>

#include "common.h"

using namespace lime;
using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

static void WaitForUserInput()
{
    std::cerr << "Press any key to continue" << std::endl;
    std::cin.ignore();
}

lime::LogLevel log_level = lime::LogLevel::Info;
static LogLevel strToLogLevel(const std::string_view str)
{
    if ("debug"sv == str)
        return LogLevel::Debug;
    else if ("verbose"sv == str)
        return LogLevel::Verbose;
    else if ("info"sv == str)
        return LogLevel::Info;
    else if ("warning"sv == str)
        return LogLevel::Warning;
    else if ("error"sv == str)
        return LogLevel::Error;
    else if ("critical"sv == str)
        return LogLevel::Critical;
    return LogLevel::Error;
}

static void log_func(const lime::LogLevel level, const std::string& message)
{
    if (level <= log_level)
        std::cout << message << std::endl;
}

int main(int argc, char** argv)
{
    // clang-format off
    args::ArgumentParser            parser("limeLOsweep - sweeps through the LO PLL range"s, ""s);
    args::HelpFlag                  help(parser, "help"s, "Print out help information"s, {'h', "help"s});

    args::ValueFlag<double>         startFreqFlag(parser, "LOstart"s, "Sweep start frequency"s, {'b', "LOstart"s});
    args::ValueFlag<double>         LOendFlag(parser, "LOend"s, "Sweep end frequency"s, {'e', "LOend"s});
    args::ValueFlag<double>         LOstepFlag(parser, "LOstep"s, "Sweep frequency step"s, {'s', "LOstep"s});
    args::ValueFlag<int>            stepDurationFlag(parser, "stepDuration"s, "time between steps in milliseconds"s, {'s', "stepDuration"s}, 0);
    args::Flag                      interactive(parser, "", "Wait for user input for each step", {"interactive"});

    args::ValueFlag<std::string>    deviceFlag(parser, "name", "Specifies which device to use", {'d', "device"});
    args::ValueFlag<std::string>    logFlag(parser, "level"s, "Log verbosity levels: error, warning, info, verbose, debug"s, {'l', "log"s}, "info"s);

    args::Group                     group(parser, "This group is all exclusive:"s, args::Group::Validators::AtMostOne);
    args::ValueFlag<std::string>    iniFlag(group, "config"s, "Path to LMS7002M .ini configuration file to use as a base"s, {'c', "config"s}, ""s);
    args::Flag                      initFlag(group, "init"s, "Just initialize the device"s, {'i', "init"s}, false);
    args::Flag                      dirRx(group, "rx"s, "Rx LO"s, {"rx"s}, false);
    args::Flag                      dirTx(group, "tx"s, "Tx LO"s, {"tx"s}, false);
    // clang-format on

    try
    {
        parser.ParseCLI(argc, argv);
    } catch (args::Help&)
    {
        std::cout << parser << std::endl;
        return EXIT_SUCCESS;
    } catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    log_level = strToLogLevel(args::get(logFlag));

    const auto LOstart = args::get(startFreqFlag);
    const auto LOstep = args::get(LOstepFlag);
    const auto LOend = args::get(LOendFlag);

    std::cout << "LOstart = "sv << LOstart / 1e6 << " MHz"sv << std::endl;
    std::cout << "LOstep  = "sv << LOstep / 1e6 << " MHz"sv << std::endl;
    std::cout << "LOend   = "sv << LOend / 1e6 << " MHz"sv << std::endl;

    const std::vector<DeviceHandle> handles = DeviceRegistry::enumerate();
    if (handles.size() == 0)
    {
        std::cout << "No devices found"sv << std::endl;
        return -1;
    }

    const std::string devName = args::get(deviceFlag);
    SDRDevice* device = cli::ConnectToFilteredOrDefaultDevice(devName);
    if (!device)
        return EXIT_FAILURE;

    auto& info = device->GetDescriptor();
    std::cout << "\nConnected to: "sv << info.name << " FW: "sv << info.firmwareVersion << " HW: " << info.hardwareVersion
              << " GW:" << info.gatewareVersion << "." << info.gatewareRevision << std::endl;

    const auto& configFilename = args::get(iniFlag);
    if (configFilename.length() > 0)
    {
        if (device->LoadConfig(0, configFilename) != OpStatus::Success)
        {
            return -1;
        }
    }
    else if (args::get(initFlag))
    {
        device->Init();
    }

    lime::registerLogHandler(log_func);
    int delay_ms = args::get(stepDurationFlag);
    lime::TRXDir direction = TRXDir::Tx;
    if (args::get(dirRx))
        direction = TRXDir::Rx;

    for (double freq = LOstart; freq <= LOend; freq += LOstep)
    {
        if (log_level >= LogLevel::Verbose)
            std::cerr << (direction == TRXDir::Tx ? "Tx" : "Rx") << " LO: " << freq << " Hz, tune time: ";
        auto t1 = std::chrono::high_resolution_clock::now();
        if (device->SetFrequency(0, direction, 0, freq) != OpStatus::Success)
        {
            std::cerr << "Failed to tune LO freq: " << freq << std::endl;
        }
        auto t2 = std::chrono::high_resolution_clock::now();

        if (log_level >= LogLevel::Verbose)
            std::cerr << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "us" << std::endl;

        if (interactive)
            WaitForUserInput();
        else if (delay_ms > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
    DeviceRegistry::freeDevice(device);
    return 0;
}
