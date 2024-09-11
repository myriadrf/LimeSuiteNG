#include <string>
#include <iostream>

#include "args.hxx"
#include "gtest/gtest.h"

#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/SDRDescriptor.h"
#include "common.h"

using ::testing::InitGoogleTest;
using namespace std;
using namespace lime;
using namespace lime::cli;

namespace lime::testing {
static std::string gTestDeviceHandleArgument;
const char* GetTestDeviceHandleArgument()
{
    return gTestDeviceHandleArgument.c_str();
}
} // namespace lime::testing

int main(int argc, char** argv)
{
    // GTest removes it's own recognized paramters from argv.
    InitGoogleTest(&argc, argv);

    args::ArgumentParser parser("Unit tests running on hardware", "");
    args::HelpFlag help(parser, "help", "Display this help menu", { 'h', "help" });
    args::ValueFlag<std::string> deviceFlag(parser, "name", "Specifies which device to use", { 'd', "device" });
    args::ValueFlag<std::string> logFlag(
        parser, "", "Log verbosity: info, warning, error, verbose, debug", { 'l', "log" }, "error", args::Options{});

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
    cli::logVerbosity = strToLogLevel(args::get(logFlag));

    auto handles = DeviceRegistry::enumerate();

    {
        const std::string devName = args::get(deviceFlag);
        DeviceHandle deserializedHandle(std::string{ devName });
        std::vector<DeviceHandle> filteredHandles;
        for (const DeviceHandle& h : handles)
        {
            // compare hint as if it was in serialized handle form.
            // if it's not, compare using basic text search among handle fields
            if (h.IsEqualIgnoringEmpty(deserializedHandle) || FuzzyHandleMatch(h, devName))
                filteredHandles.push_back(h);
        }
        handles = std::move(filteredHandles);
    }

    if (handles.empty())
    {
        std::cerr << "No devices detected."sv << std::endl;
        return EXIT_FAILURE;
    }
    if (handles.size() > 1)
    {
        std::cerr << "Multiple devices detected, specify which one to use with -d, --device:"sv << std::endl;
        for (const DeviceHandle& h : handles)
            std::cerr << "\t"sv << h.Serialize() << std::endl;
        return EXIT_FAILURE;
    }
    lime::testing::gTestDeviceHandleArgument = handles.at(0).Serialize();

    SDRDevice* device = DeviceRegistry::makeDevice(handles.at(0));
    if (!device)
    {
        std::cerr << "Failed to connect to: "sv << handles.at(0).Serialize() << std::endl;
        return EXIT_FAILURE;
    }

    const lime::SDRDescriptor d = device->GetDescriptor();
    cout << "Using " << d.name << " HW:" << d.hardwareVersion << " GW:" << d.gatewareVersion << "." << d.gatewareRevision
         << " FW:" << d.firmwareVersion << endl;
    DeviceRegistry::freeDevice(device);

    int testsStatus = RUN_ALL_TESTS();
    return testsStatus;
}
