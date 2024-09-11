#include <string>
#include <iostream>

#include "args.hxx"
#include "gtest/gtest.h"

#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/SDRDescriptor.h"
#include "common.h"

using ::testing::InitGoogleTest;
using namespace std;

namespace lime::testing {
static lime::SDRDevice* gTestDevice = nullptr;
lime::SDRDevice* GetTestDevice()
{
    return gTestDevice;
}
} // namespace lime::testing

int main(int argc, char** argv)
{
    // GTest removes it's own recognized paramters from argv.
    InitGoogleTest(&argc, argv);

    args::ArgumentParser parser("Unit tests running on hardware", "");
    args::HelpFlag help(parser, "help", "Display this help menu", { 'h', "help" });
    args::ValueFlag<std::string> deviceFlag(parser, "name", "Specifies which device to use", { 'd', "device" });

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

    const std::string devName = args::get(deviceFlag);
    lime::testing::gTestDevice = ConnectToFilteredOrDefaultDevice(devName);
    if (!lime::testing::gTestDevice)
        return EXIT_FAILURE;

    const lime::SDRDescriptor d = lime::testing::gTestDevice->GetDescriptor();
    cout << "Using " << d.name << " HW:" << d.hardwareVersion << " GW:" << d.gatewareVersion << "." << d.gatewareRevision
         << " FW:" << d.firmwareVersion << endl;

    return RUN_ALL_TESTS();
}
