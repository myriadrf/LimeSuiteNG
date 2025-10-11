#include "common.h"

#include "limesuiteng/SDRDescriptor.h"

#include <cassert>
#include <cstring>
#include <filesystem>
#include "args.hxx"

using namespace std;
using namespace lime;
using namespace lime::cli;

static uint32_t FindbusIdByName(SDRDevice* device, const std::string_view busName)
{
    if (!device)
        return -1;
    const auto busMap = device->GetDescriptor().i2cBusIds;
    if (busName.size() == 0)
    {
        cerr << "specify I2C bus, -b, --bus :"sv << endl;
        for (const auto& nameIds : busMap)
            cerr << "\t"sv << nameIds.first << endl;
        return -1;
    }

    auto iter = busMap.find(std::string{ busName });
    if (iter == busMap.end())
    {
        cerr << "Device does not contain bus ("sv << busName << "). Available list:"sv << endl;
        for (const auto& nameIds : busMap)
            cerr << "\t"sv << nameIds.first << endl;
        return -1;
    }
    return iter->second;
}

static std::vector<uint8_t> hexToArray(const std::string_view hexstr)
{
    std::vector<uint8_t> data;
    data.reserve(hexstr.length() / 2);
    for (size_t i = 0; i < hexstr.length(); i += 2)
    {
        char ctemp[3] = {};
        memcpy(ctemp, &hexstr[i], 2);
        uint32_t value = 0;
        sscanf(ctemp, "%X", &value);
        data.push_back(value);
    }
    return data;
}

static uint32_t hex2int(const std::string_view hexstr)
{
    uint32_t value = 0;
    sscanf(hexstr.data(), "%X", &value);
    return value;
}

int main(int argc, char** argv)
{
    // clang-format off
    args::ArgumentParser                    parser("limeI2C - I2C bus comms", "");
    args::HelpFlag                          help(parser, "help", "This help", {'h', "help"});

    args::Group                             commands(parser, "commands"); // NOLINT(cppcoreguidelines-slicing) 
    args::Command                           readCmd(commands, "read", "Reading operation");
    args::Command                           writeCmd(commands, "write", "Do writing operation");

    args::Group                             arguments(parser, "arguments", args::Group::Validators::DontCare, args::Options::Global); // NOLINT(cppcoreguidelines-slicing)
    args::ValueFlag<std::string>            deviceFlag(arguments, "device", "Specifies which device to use", {'d', "device"}, "");
    args::ValueFlag<std::string>            busFlag(arguments, "bus", "I2C bus index", {'b', "bus"}, "");
    args::ValueFlag<std::string>            i2cAddressHexFlag(arguments, "soc address", "I2C device address", {'a', "address"}, "");
    args::ValueFlag<std::string>            registerOffsetHexFlag(arguments, "register offset", "device register offset", {'o', "offset"}, "");
    args::ValueFlag<std::string>            dataHexFlag(arguments, "data", "data payload", {'d', "data"}, "");
    args::ValueFlag<int>                    readLength(arguments, "length", "data payload length", {'l', "length"}, 0);
    args::ValueFlag<std::string>            logFlag(parser, "", "Log verbosity: info, warning, error, verbose, debug", {'l', "log"}, "error", args::Options{});

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

    logVerbosity = strToLogLevel(args::get(logFlag));

    const std::string devName = args::get(deviceFlag);
    const std::string busName = args::get(busFlag);
    const std::string hexInput = args::get(dataHexFlag);

    auto handles = DeviceRegistry::enumerate();
    if (handles.size() == 0)
    {
        cerr << "No devices found"sv << endl;
        return EXIT_FAILURE;
    }

    SDRDevice* device = ConnectToFilteredOrDefaultDevice(devName);
    if (!device)
        return EXIT_FAILURE;

    device->SetMessageLogCallback(lime::cli::LogCallback);
    lime::registerLogHandler(lime::cli::LogCallback);

    int32_t busId = FindbusIdByName(device, busName);
    if (busId < 0)
    {
        DeviceRegistry::freeDevice(device);
        return EXIT_FAILURE;
    }

    uint32_t socAddress = hex2int(args::get(i2cAddressHexFlag));

    uint16_t registerOffset = 0;
    std::vector<uint8_t> registerOffsetBytes;
    if (registerOffsetHexFlag)
    {
        registerOffsetBytes = hexToArray(args::get(registerOffsetHexFlag));
        for (uint8_t byte : registerOffsetBytes)
        {
            registerOffset <<= 8;
            registerOffset |= byte;
        }
    }

    std::vector<uint8_t> data;
    if (dataHexFlag)
        data = hexToArray(args::get(dataHexFlag));

    try
    {
        if (writeCmd)
        {
            device->I2CWrite(busId, socAddress, registerOffset, registerOffsetBytes.size(), data.data(), data.size());
        }
        else if (readCmd)
        {
            int length = args::get(readLength);
            data.resize(length);
            device->I2CRead(busId, socAddress, registerOffset, registerOffsetBytes.size(), data.data(), length);
            for (auto b : data)
                printf("%02X ", uint16_t(b));
            printf("\n");
        }

    } catch (std::runtime_error& e)
    {
        DeviceRegistry::freeDevice(device);
        cerr << "I2C failed: "sv << e.what() << endl;
        return EXIT_FAILURE;
    }

    DeviceRegistry::freeDevice(device);
    return EXIT_SUCCESS;
}
