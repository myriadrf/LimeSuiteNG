#include "common.h"

#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/LMS7002M.h"

#include <cassert>
#include <cstring>
#include <filesystem>
#include "args.hxx"

#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>

using namespace std;
using namespace lime;
using namespace lime::cli;

static std::string exec(const char* cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe)
    {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr)
    {
        result += buffer.data();
    }
    return result;
}

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

static uint32_t hex2int(const std::string_view hexstr)
{
    uint32_t value = 0;
    sscanf(hexstr.data(), "%X", &value);
    return value;
}

class PowerDetector
{
  public:
    PowerDetector(const std::string& spiDev)
        : spiDev(spiDev)
    {
        char command[512];
        gpio25handle = exec("gpiofind GPIO25");
        if (gpio25handle.empty())
        {
            printf("GPIO25 not found\n");
            exit(1);
        }
        gpio25handle.pop_back(); // remove new line

        gpio26handle = exec("gpiofind GPIO26");
        if (gpio26handle.empty())
        {
            printf("GPIO26 not found\n");
            exit(1);
        }
        gpio26handle.pop_back(); // remove new line

        rfpd_en_handle = exec("gpiofind RFPD_EN");
        if (rfpd_en_handle.empty())
        {
            printf("RFPD_EN not found\n");
            exit(1);
        }
        rfpd_en_handle.pop_back(); // remove new line

        sprintf(command, "gpioset %s=1", rfpd_en_handle.c_str());
        exec(command);

        sprintf(command, "spi-config -d %s -s 400000", spiDev.c_str());
        spiConfigHandle = popen(command, "we");

        sprintf(command, "gpioset %s=1", gpio25handle.c_str());
        printf("cmd: %s\n", command);
        exec(command);
        sprintf(command, "gpioset %s=1", gpio26handle.c_str());
        exec(command);
    }

    uint16_t ReadValue(uint16_t channel)
    {
        const std::string& gpioHandle = channel == 0 ? gpio25handle : gpio26handle;
        char command[512];
        std::string binaryData;
        for (int r = 0; r < 4; ++r)
        {
            sprintf(command, "gpioset %s=0", gpioHandle.c_str());
            exec(command);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            sprintf(command, "gpioset %s=1", gpioHandle.c_str());
            exec(command);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            sprintf(command, "gpioset %s=0", gpioHandle.c_str());
            exec(command);

            sprintf(command, "spi-pipe --device=%s -b 2 -n 1 < /dev/zero", spiDev.c_str());
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            binaryData = exec(command);
            if (binaryData.size() < 2)
            {
                // printf("Bad SPI read, got %i bytes, retry\n", binaryData.size());
                continue;
            }
            break;
        }
        uint16_t value = uint16_t(binaryData[0]) << 8 | binaryData[1];
        // printf("SPI Read %04X\n", value);

        sprintf(command, "gpioset %s=1", gpioHandle.c_str());
        exec(command);
        return value;
    }

    ~PowerDetector()
    {
        if (spiConfigHandle)
            pclose(spiConfigHandle);
    }

  private:
    FILE* spiConfigHandle;
    std::string gpio25handle;
    std::string gpio26handle;
    std::string rfpd_en_handle;
    std::string spiDev;
};

int main(int argc, char** argv)
{
    // clang-format off
    args::ArgumentParser            parser("limePDET", "");
    args::HelpFlag                  help(parser, "help", "This help", {'h', "help"});
    args::Flag                      readFlag(parser, "read", "Only read power detector values", {'r', "read"});
    args::ValueFlag<uint>           powerFlag(parser, "power", "Power target threshold", {'p', "power"});
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
        cerr << e.what() << endl;
        return EXIT_FAILURE;
    }

    if (!powerFlag && !readFlag)
    {
        cerr << "missing power threshold target" << endl;
        return EXIT_FAILURE;
    }

    auto handles = DeviceRegistry::enumerate();
    if (handles.size() == 0)
    {
        cerr << "No devices found"sv << endl;
        return EXIT_FAILURE;
    }

    SDRDevice* device = ConnectToFilteredOrDefaultDevice("");
    if (!device)
        return EXIT_FAILURE;

    int32_t chipSelect = FindChipSelectByName(device, "LMS7002M");
    if (chipSelect < 0)
    {
        DeviceRegistry::freeDevice(device);
        return EXIT_FAILURE;
    }

    lime::LMS7002M* soc = reinterpret_cast<lime::LMS7002M*>(device->GetInternalChip(0));

    PowerDetector pdet("/dev/spidev0.0");

    uint16_t powerLevel[2] = { 0, 0 };
    if (readFlag)
    {
        for (int c = 0; c < 2; ++c)
        {
            powerLevel[c] = pdet.ReadValue(c);
            printf("Ch.%s power:%i\n", c == 0 ? "A" : "B", powerLevel[c]);
        }
        DeviceRegistry::freeDevice(device);
        return EXIT_SUCCESS;
    }

    uint16_t gainValue[2];
    uint16_t expectedPowerThreshold = args::get(powerFlag);
    bool powerReached[2] = { 0, 0 };
    for (int g = 30; g >= 0; g--)
    {
        for (int c = 0; c < 2; ++c)
        {
            if (powerLevel[c] < expectedPowerThreshold)
            {
                gainValue[c] = g;
                soc->SetActiveChannel(c == 0 ? LMS7002M::Channel::ChA : LMS7002M::Channel::ChB);
                soc->Modify_SPI_Reg_bits(LMS7002MCSR::LOSS_LIN_TXPAD_TRF, g);
                soc->Modify_SPI_Reg_bits(LMS7002MCSR::LOSS_MAIN_TXPAD_TRF, g);
                powerLevel[c] = pdet.ReadValue(c);
                printf("gain:%02i Ch.%s power:%04X (%i)\n", gainValue[c], c == 0 ? "A" : "B", powerLevel[c], powerLevel[c]);
            }
            else
                powerReached[c] = true;
        }
        printf("\n");
        if (powerReached[0] && powerReached[1])
            break;
    }
    soc->SetActiveChannel(LMS7002M::Channel::ChA);

    printf("Results:\n");
    for (int c = 0; c < 2; ++c)
        printf("Ch.%s gain:%02i  power:%04X (%i)\n", c == 0 ? "A" : "B", gainValue[c], powerLevel[c], powerLevel[c]);

    DeviceRegistry::freeDevice(device);
    return EXIT_SUCCESS;
}
