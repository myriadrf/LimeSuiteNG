#include "boards/LMS7002M_SDRDevice.h"
#include "FPGA_common/FPGA_common.h"
#include "limesuiteng/DeviceHandle.h"
#include "limesuiteng/DeviceRegistry.h"
#include "limesuiteng/LMS7002M.h"
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
#include "args/args.hxx"

using namespace lime;
using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

namespace {

bool ConfigureCGEN(SDRDevice* device, double freqMHz)
{
    std::cout << "Set CGEN "sv << freqMHz / 1e6 << " MHz: "sv;
    LMS7002M* lms = static_cast<LMS7002M*>(device->GetInternalChip(0));
    lms->Modify_SPI_Reg_bits(LMS7002MCSR::MAC, 1, true);
    int interp = lms->Get_SPI_Reg_bits(LMS7002MCSR::HBI_OVR_TXTSP);
    int decim = lms->Get_SPI_Reg_bits(LMS7002MCSR::HBD_OVR_RXTSP);
    if (lms->SetInterfaceFrequency(freqMHz, interp, decim) != OpStatus::Success)
    {
        std::cout << "LMS VCO Fail"sv << std::endl;
        return false;
    }

    auto fpgaTxPLL = lms->GetReferenceClk_TSP(TRXDir::Tx);
    if (interp != 7)
        fpgaTxPLL /= std::pow(2.0, interp);
    auto fpgaRxPLL = lms->GetReferenceClk_TSP(TRXDir::Rx);
    if (decim != 7)
        fpgaRxPLL /= std::pow(2.0, decim);
    auto chipInd = lms->GetActiveChannelIndex() / 2;
    auto fpga = static_cast<LMS7002M_SDRDevice*>(device)->GetFPGA();
    if (fpga)
    {
        OpStatus status = fpga->SetInterfaceFreq(fpgaTxPLL, fpgaRxPLL, chipInd);
        if (status != OpStatus::Success)
        {
            std::cout << "FPGA Fail"sv << std::endl;
            return false;
        }
    }
    std::cout << "OK"sv << std::endl;
    return true;
}

lime::LogLevel log_level = lime::LogLevel::Info;
LogLevel strToLogLevel(const std::string_view str)
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

void log_func(const lime::LogLevel level, const std::string& message)
{
    if (level <= log_level)
        std::cout << message << std::endl;
}

} // namespace

int main(int argc, char** argv)
{
    // clang-format off
    args::ArgumentParser            parser("pll_sweep - sweeps through the PLL range"s, ""s);
    args::HelpFlag                  help(parser, "help"s, "This help"s, {'h', "help"s});

    args::ValueFlag<double>         startFreqFlag(parser, "beginFreq"s, "Sweep start frequency"s, {'b', "beginFreq"s}, 50e6);
    args::ValueFlag<double>         endFreqFlag(parser, "endFreq"s, "Sweep end frequency"s, {'e', "endFreq"s}, 500e6);
    args::ValueFlag<double>         stepFreqFlag(parser, "stepFreq"s, "Sweep frequency step"s, {'s', "stepFreq"s}, 1e6);

    args::ValueFlag<int>            deviceFlag(parser, "device"s, "Specifies which device to use"s, {'d', "device"s}, -1);
    args::ValueFlag<std::string>    logFlag(parser, "level"s, "Log verbosity levels: error, warning, info, verbose, debug"s, {'l', "log"s}, "info"s);

    args::Group                     group(parser, "This group is all exclusive:"s, args::Group::Validators::AtMostOne);
    args::ValueFlag<std::string>    iniFlag(group, "config"s, "Path to LMS7002M .ini configuration file to use as a base"s, {'c', "config"s}, ""s);
    args::Flag                      initFlag(group, "init"s, "Just initialize the device"s, {'i', "init"s}, false);
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

    const auto beginFreq = args::get(startFreqFlag);
    const auto stepFreq = args::get(stepFreqFlag);
    const auto endFreq = args::get(endFreqFlag);

    std::cout << "beginFreq = "sv << beginFreq / 1e6 << " MHz"sv << std::endl;
    std::cout << "stepFreq  = "sv << stepFreq / 1e6 << " MHz"sv << std::endl;
    std::cout << "endFreq   = "sv << endFreq / 1e6 << " MHz"sv << std::endl;

    SDRDevice* device;
    const std::vector<DeviceHandle> handles = DeviceRegistry::enumerate();
    if (handles.size() == 0)
    {
        std::cout << "No devices found"sv << std::endl;
        return -1;
    }
    if (handles.size() == 1) //open the only available device
        device = DeviceRegistry::makeDevice(handles.at(0));
    else //display device selection
    {
        const auto deviceIndex = args::get(deviceFlag);
        if (deviceIndex < 0)
        {
            std::cout << "Device list:"sv << std::endl;
            for (size_t i = 0; i < handles.size(); i++)
                std::cout << std::setw(2) << i << ". "sv << handles[i].name << std::endl;
            std::cout << "Select device index (0-"sv << handles.size() - 1 << "): "sv;
            int selection = 0;
            std::cin >> selection;
            selection = selection % handles.size();
            device = DeviceRegistry::makeDevice(handles.at(selection));
        }
        else
            device = DeviceRegistry::makeDevice(handles.at(deviceIndex));
    }
    if (device == nullptr)
    {
        std::cout << "Failed to connected to device"sv << std::endl;
        return -1;
    }
    auto& info = device->GetDescriptor();
    std::cout << "\nConnected to: "sv << info.name << " FW: "sv << info.firmwareVersion << " HW: " << info.hardwareVersion
              << std::endl;

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
    int errors = 0;
    for (double freq = beginFreq; freq < endFreq; freq += stepFreq)
        if (ConfigureCGEN(device, freq) == false)
            errors++;
    std::cout << "Errors: "sv << errors << std::endl;
    DeviceRegistry::freeDevice(device);
    return 0;
}
