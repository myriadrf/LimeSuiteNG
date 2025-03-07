#include "common.h"

#include "limesuiteng/SDRDescriptor.h"

#include <cassert>
#include <cstring>
#include <filesystem>
#include "args.hxx"

using namespace std;
using namespace lime;
using namespace lime::cli;

static int32_t FindChipSelectByName(SDRDevice* device, const std::string_view chipName)
{
    if (!device)
        return -1;
    const auto chipMap = device->GetDescriptor().spiSlaveIds;
    if (chipName.size() == 0)
    {
        cerr << "specify SPI chip select, -c, --chip :"sv << endl;
        for (const auto& nameIds : chipMap)
            cerr << "\t"sv << nameIds.first << endl;
        return -1;
    }

    auto iter = chipMap.find(std::string{ chipName });
    if (iter == chipMap.end())
    {
        cerr << "Device does not contain target chip ("sv << chipName << "). Available list:"sv << endl;
        for (const auto& nameIds : chipMap)
            cerr << "\t"sv << nameIds.first << endl;
        return -1;
    }
    return iter->second;
}

static void PrintMISO(std::ostream& stream, const std::vector<uint32_t>& miso)
{
    stream << std::hex << std::setfill('0');
    for (uint32_t value : miso)
        stream << std::setw(8) << value << std::endl;
}

static uint32_t hex2int(const std::string_view hexstr)
{
    uint32_t value = 0;
    sscanf(hexstr.data(), "%X", &value);
    return value;
}

static void TestSPITiming(SDRDevice* device, int chipSelect)
{
    uint32_t mosi[16];
    uint32_t miso[16];

    const int tryCount = 100;
    const int batchSize = 16;

    // single register read
    for (int i = 0; i < 16; ++i)
        mosi[i] = 0x00200020;
    auto t1 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < tryCount; ++i)
        device->SPI(chipSelect, mosi, miso, 1);
    auto t2 = std::chrono::high_resolution_clock::now();
    // std::cout << "value " << miso[0] << std::endl;
    double transactionTime = double(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()) / tryCount;
    std::cout << "SPI x1 read:\t" << transactionTime << "us\n";

    // batch register read
    for (int i = 0; i < 16; ++i)
        mosi[i] = 0x00200020;
    t1 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < tryCount; ++i)
        device->SPI(chipSelect, mosi, miso, batchSize);
    t2 = std::chrono::high_resolution_clock::now();
    // std::cout << "value " << miso[0] << std::endl;
    double batchTime = double(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()) / tryCount;
    std::cout << "SPI x" << batchSize << " batch read:\t" << batchTime << "us\n";

    double registerTime = (batchTime - transactionTime) / (batchSize - 1);
    std::cout << "Register time:\t" << registerTime << "us\n";
    std::cout << "Transaction overhead:\t" << transactionTime - registerTime << "us\n";

    // single register write
    for (int i = 0; i < 16; ++i)
        mosi[i] = 0x8020FFFD;
    t1 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < tryCount; ++i)
        device->SPI(chipSelect, mosi, nullptr, 1);
    t2 = std::chrono::high_resolution_clock::now();
    // std::cout << "value " << miso[0] << std::endl;
    transactionTime = double(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()) / tryCount;
    std::cout << "SPI x1 write:\t" << transactionTime << "us\n";

    // batch register write
    t1 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < tryCount; ++i)
        device->SPI(chipSelect, mosi, nullptr, batchSize);
    t2 = std::chrono::high_resolution_clock::now();
    // std::cout << "value " << miso[0] << std::endl;
    batchTime = double(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()) / tryCount;
    std::cout << "SPI x" << batchSize << " batch write:\t" << batchTime << "us\n";

    registerTime = (batchTime - transactionTime) / (batchSize - 1);
    std::cout << "Register time:\t" << registerTime << "us\n";
    std::cout << "Transaction overhead:\t" << transactionTime - registerTime << "us\n";
}

static void TestLOtiming(SDRDevice* device)
{
    int tryCount = 0;
    auto t1 = std::chrono::high_resolution_clock::now();
    // for (int i=0; i<tryCount; ++i)
    for (double freq = 30e6; freq < 3.5e9; freq += 61.44e5)
    {
        tryCount++;
        // OpStatus res = device->SetFrequency(0, TRXDir::Rx, 0, 1000e6+3000000*i);
        // printf("F: %f\n", freq/1e6);
        OpStatus res = device->SetFrequency(0, TRXDir::Rx, 0, freq);
        if (res != OpStatus::Success)
        {
            std::cout << "Failure" << std::endl;
            //return;
        }
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    // std::cout << "value " << miso[0] << std::endl;
    double transactionTime = double(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()) / tryCount;
    std::cout << "LO tune time:\t" << transactionTime << "us\n";
    std::cout << "total time:\t" << double(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()) << "us\n";
}

int main(int argc, char** argv)
{
    // clang-format off
    args::ArgumentParser                    parser("limeSPI - Control status registers I/O", "");
    args::HelpFlag                          help(parser, "help", "This help", {'h', "help"});
    args::ValueFlag<std::string>    logFlag(parser, "log level", "Log verbosity: info, warning, error, verbose, debug", {'l', "log"}, "error");

    args::Group                             arguments(parser, "arguments", args::Group::Validators::DontCare, args::Options::Global); // NOLINT(cppcoreguidelines-slicing)
    args::ValueFlag<std::string>            deviceFlag(arguments, "name", "Specifies which device to use", {'d', "device"}, "");
    args::ValueFlag<std::string>            chipFlag(arguments, "name", "Selects destination chip", {'c', "chip"}, "");
    // clang-format on

    try
    {
        parser.ParseCLI(argc, argv);
    } catch (args::Help&)
    {
        cout << parser << endl;
        return EXIT_SUCCESS;
    } catch (const std::exception& e)
    {
        cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    const std::string devName = args::get(deviceFlag);
    const std::string chipName = args::get(chipFlag);

    auto handles = DeviceRegistry::enumerate();
    if (handles.size() == 0)
    {
        cerr << "No devices found"sv << endl;
        return EXIT_FAILURE;
    }

    SDRDevice* device = ConnectToFilteredOrDefaultDevice(devName);
    if (!device)
        return EXIT_FAILURE;

    // logVerbosity = lime::LogLevel::Debug;
    logVerbosity = strToLogLevel(args::get(logFlag));

    device->SetMessageLogCallback(lime::cli::LogCallback);
    lime::registerLogHandler(lime::cli::LogCallback);

    device->Init();

    const auto chipMap = device->GetDescriptor().spiSlaveIds;
    /* for (auto cs : chipMap)
    {
        cout << cs.first <<  ":\n";
        TestSPITiming(device, cs.second);
    }*/
    TestLOtiming(device);

    DeviceRegistry::freeDevice(device);
    return EXIT_SUCCESS;
}
