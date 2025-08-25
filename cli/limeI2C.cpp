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

static uint32_t hex2int(const std::string_view hexstr)
{
    uint32_t value = 0;
    sscanf(hexstr.data(), "%X", &value);
    return value;
}

static int parseWriteInput(std::string_view hexstr, std::vector<uint32_t>& mosi)
{
    static const std::string_view delimiters = " \n,"sv;
    mosi.clear();

    const uint32_t spiWriteBit = 1 << 31;
    int tokenCount = 0;

    std::size_t position = 0;
    while (position != std::string_view::npos)
    {
        position = hexstr.find_first_of(delimiters);
        std::string_view token = hexstr.substr(0, position);
        int tokenLength = token.size();
        if (tokenLength <= 8 && tokenLength > 4) // write instruction
        {
            uint32_t value = hex2int(token);
            mosi.push_back(spiWriteBit | value);
        }
        else if (tokenLength != 0)
        {
            std::cerr << "Invalid input value: "sv << token << std::endl;
        }
        ++tokenCount;
        hexstr = hexstr.substr(position + 1);
    }
    return tokenCount;
}

static int parseReadInput(std::string_view hexstr, std::vector<uint32_t>& mosi)
{
    static const std::string_view delimiters = " \n,"sv;
    mosi.clear();

    int tokenCount = 0;

    std::size_t position = 0;
    while (position != std::string_view::npos)
    {
        position = hexstr.find_first_of(delimiters);
        std::string_view token = hexstr.substr(0, position);
        int tokenLength = token.size();
        if (tokenLength <= 4 && tokenLength > 0) // read instruction
        {
            uint32_t value = hex2int(token);
            mosi.push_back(value);
        }
        else if (tokenLength != 0)
        {
            std::cerr << "Invalid input value: "sv << token << std::endl;
        }
        ++tokenCount;
        hexstr = hexstr.substr(position + 1);
    }
    return tokenCount;
}

static std::string ReadFile(const std::string& fileName)
{
    std::vector<char> buffer;
    std::ifstream inputFile(fileName);
    if (!inputFile.is_open())
    {
        cerr << "Failed to open file: "sv << fileName << endl;
        exit(EXIT_FAILURE);
    }
    inputFile.seekg(0, std::ios::end);
    long fileSize = inputFile.tellg();
    inputFile.seekg(0, std::ios::beg);

    buffer.resize(fileSize);
    inputFile.read(&buffer[0], fileSize);
    inputFile.close();
    buffer[fileSize] = 0;
    return buffer.data();
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
    args::ValueFlag<std::string>            socAddressFlag(arguments, "soc address", "I2C device address", {'a', "address"}, "");
    args::ValueFlag<std::string>            registerFlag(arguments, "register offset", "device register offset", {'r', "register"}, "");
    args::ValueFlag<std::string>            dataFlag(arguments, "data", "device register offset", {'p', "payload"}, "");

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
    const std::string busName = args::get(busFlag);
    const std::string hexInput = args::get(dataFlag);

    if (hexInput.empty())
    {
        cerr << "No input provided"sv << endl;
        return EXIT_FAILURE;
    }

    auto handles = DeviceRegistry::enumerate();
    if (handles.size() == 0)
    {
        cerr << "No devices found"sv << endl;
        return EXIT_FAILURE;
    }

    SDRDevice* device = ConnectToFilteredOrDefaultDevice(devName);
    if (!device)
        return EXIT_FAILURE;

    int32_t busId = FindbusIdByName(device, busName);
    if (busId < 0)
    {
        DeviceRegistry::freeDevice(device);
        return EXIT_FAILURE;
    }

    uint32_t socAddress = hex2int(args::get(socAddressFlag));
    uint32_t registerOffset = hex2int(args::get(registerFlag));
    uint32_t data = hex2int(args::get(dataFlag));

    std::vector<uint8_t> buffer;
    buffer.push_back(data);

    try
    {

        if (writeCmd)
        {
            //parseWriteInput(hexInput, buffer);
            device->I2CWrite(busId, socAddress, registerOffset, buffer.data(), buffer.size());
        }
        else if (readCmd)
        {
            //parseReadInput(hexInput, mosi);
            device->I2CRead(busId, socAddress, registerOffset, buffer.data(), buffer.size());
        }

    } catch (std::runtime_error& e)
    {
        DeviceRegistry::freeDevice(device);
        cerr << "I2C failed: "sv << e.what() << endl;
        return EXIT_FAILURE;
    }

    for (int i = 0; i < buffer.size(); ++i)
        printf(" 0x%02X", static_cast<uint16_t>(buffer[i]));
    printf("\n");

    DeviceRegistry::freeDevice(device);
    return EXIT_SUCCESS;
}
