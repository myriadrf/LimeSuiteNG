#include "cli/common.h"
#include <getopt.h>
#include <filesystem>
#include "args/args.hxx"

using namespace std;
using namespace lime;

SDRDevice* device = nullptr;
bool terminateProgress(false);

void intHandler(int sig)
{
    if (terminateProgress == true)
    {
        cerr << "Force exiting"sv << endl;
        exit(-1);
    }
    terminateProgress = true;
}

static void PrintMemoryDevices(SDRDescriptor descriptor)
{
    for (const auto& memoryDevice : descriptor.memoryDevices)
    {
        cerr << '\t' << memoryDevice.first << endl;
    }
}

static const std::shared_ptr<DataStorage> FindMemoryDeviceByName(SDRDevice* device, const std::string_view targetName)
{
    if (!device)
    {
        return std::make_shared<DataStorage>(nullptr, eMemoryDevice::COUNT);
    }

    const auto descriptor = device->GetDescriptor();
    const auto memoryDevices = descriptor.memoryDevices;
    if (targetName.size() == 0)
    {
        // If name is not specified, but only single choice is available, use that.
        if (memoryDevices.size() == 1)
        {
            return memoryDevices.begin()->second;
        }

        cerr << "Multiple targets found. Specify memory device target, -t, --target\nAvailable targets:"sv << endl;
        PrintMemoryDevices(descriptor);
        return std::make_shared<DataStorage>(nullptr, eMemoryDevice::COUNT);
    }

    try
    {
        return memoryDevices.at(std::string{ targetName });
    } catch (const std::out_of_range& e)
    { /*Handled outside catch block*/
    }

    cerr << "Device does not contain target device ("sv << targetName << "). Available list:"sv << endl;
    PrintMemoryDevices(descriptor);

    return std::make_shared<DataStorage>(nullptr, eMemoryDevice::COUNT);
}

static auto lastProgressUpdate = std::chrono::steady_clock::now();
bool progressCallBack(std::size_t bsent, std::size_t btotal, const std::string& statusMessage)
{
    float percentage = 100.0 * bsent / btotal;
    const bool hasCompleted = bsent == btotal;
    auto now = std::chrono::steady_clock::now();
    // no need to spam the text with each callback
    if (now - lastProgressUpdate > std::chrono::milliseconds(10) || hasCompleted)
    {
        lastProgressUpdate = now;
        std::cout << "[" << std::fixed << std::setprecision(0) << percentage << "] bytes written " << bsent << "/" << btotal << "\r"
                  << std::flush;
    }
    if (hasCompleted)
        cout << endl;
    if (!terminateProgress)
    {
        return false;
    }

    printf("\nAborting programming will corrupt firmware and will need external programmer to fix it. Are you sure? [y/n]: ");
    fflush(stdout);
    std::string answer;
    while (1)
    {
        std::getline(std::cin, answer);
        if (answer[0] == 'y')
        {
            cout << "\naborting..."sv << endl;
            return true;
        }
        else if (answer[0] == 'n')
        {
            terminateProgress = false;
            cout << "\ncontinuing..."sv << endl;
            return false;
        }
        else
        {
            cout << "Invalid option("sv << answer << "), [y/n]: "sv;
        }
    }
    return false;
}

int main(int argc, char** argv)
{
    // clang-format off
    args::ArgumentParser            parser("limeFLASH - Program gateware file to device flash memory", "");
    args::HelpFlag                  help(parser, "help", "This help", {'h', "help"});
    args::ValueFlag<std::string>    deviceFlag(parser, "device", "Specifies which device to use", {'d', "device"}, "");
    args::ValueFlag<std::string>    targetFlag(parser, "TARGET", "Specifies which target to use", {'t', "target"}, "");
    args::Flag                      listFlag(parser, "list", "list available device's targets", {'l', "list"});
    args::Positional<std::string>   fileFlag(parser, "file path", "Input file path", args::Options::Required);
    // clang-format on

    try
    {
        parser.ParseCLI(argc, argv);
    } catch (args::Help&)
    {
        cout << parser << endl;
        return EXIT_SUCCESS;
    } catch (const args::RequiredError& e)
    {
        if (!listFlag)
        {
            cerr << e.what() << std::endl;
            return EXIT_FAILURE;
        }
    } catch (const std::exception& e)
    {
        cerr << e.what() << endl;
        return EXIT_FAILURE;
    }

    auto handles = DeviceRegistry::enumerate();
    if (handles.size() == 0)
    {
        cerr << "No devices found"sv << endl;
        return EXIT_FAILURE;
    }

    const std::string filePath = args::get(fileFlag);
    std::string devName = args::get(deviceFlag);
    std::string targetName = args::get(targetFlag);
    signal(SIGINT, intHandler);

    SDRDevice* device = ConnectToFilteredOrDefaultDevice(devName);
    if (!device)
        return EXIT_FAILURE;

    if (listFlag)
    {
        cerr << "Available targets:" << endl;
        PrintMemoryDevices(device->GetDescriptor());
        return EXIT_SUCCESS;
    }

    auto memorySelect = FindMemoryDeviceByName(device, targetName);
    if (memorySelect->ownerDevice == nullptr)
    {
        DeviceRegistry::freeDevice(device);
        return EXIT_FAILURE;
    }

    std::vector<char> data;
    std::ifstream inputFile;
    inputFile.open(filePath, std::ifstream::in | std::ifstream::binary);
    if (!inputFile)
    {
        DeviceRegistry::freeDevice(device);
        cerr << "Failed to open file: "sv << filePath << endl;
        return EXIT_FAILURE;
    }
    inputFile.seekg(0, std::ios_base::end);
    auto cnt = inputFile.tellg();
    inputFile.seekg(0, std::ios_base::beg);
    cerr << "File size : "sv << cnt << " bytes."sv << endl;
    data.resize(cnt);
    inputFile.read(data.data(), cnt);
    inputFile.close();

    if (memorySelect->ownerDevice->UploadMemory(memorySelect->memoryDeviceType, 0, data.data(), data.size(), progressCallBack) !=
        OpStatus::Success)
    {
        DeviceRegistry::freeDevice(device);
        cout << "Device programming failed."sv << endl;
        return EXIT_FAILURE;
    }
    if (terminateProgress)
        cerr << "Programming aborted."sv << endl;
    else
        cerr << "Programming completed."sv << endl;

    DeviceRegistry::freeDevice(device);
    return EXIT_SUCCESS;
}
