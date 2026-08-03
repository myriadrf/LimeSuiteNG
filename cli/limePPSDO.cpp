#include "limePPSDO.hpp"

using namespace std;
using namespace lime;
using namespace lime::cli;

static std::atomic<bool> cleanUp(false);
static void keyBoardInt(int param)
{
    cleanUp = true;
}

static const std::unordered_map<lime::eLMS_DEV, ppsdo_reg_list_t> SDR_PPSDO_Registers = {
    { lime::eLMS_DEV::LMS_DEV_LIMESDR_XTRX,
        {
            { PPSDORegistersID::PPSDO_ENABLE, 0x00000000F000B000 },
            { PPSDORegistersID::PPSDO_CONFIG_ONE_S_TARGET, 0x00000000F000B004 },
            { PPSDORegistersID::PPSDO_CONFIG_ONE_S_TOL, 0x00000000F000B008 },
            { PPSDORegistersID::PPSDO_CONFIG_TEN_S_TARGET, 0x00000000F000B00C },
            { PPSDORegistersID::PPSDO_CONFIG_TEN_S_TOL, 0x00000000F000B010 },
            { PPSDORegistersID::PPSDO_CONFIG_HUNDRED_S_TARGET, 0x00000000F000B014 },
            { PPSDORegistersID::PPSDO_CONFIG_HUNDRED_S_TOL, 0x00000000F000B018 },
            { PPSDORegistersID::PPSDO_STATUS_ONE_S_ERROR, 0x00000000F000B01C },
            { PPSDORegistersID::PPSDO_STATUS_TEN_S_ERROR, 0x00000000F000B020 },
            { PPSDORegistersID::PPSDO_STATUS_HUNDRED_S_ERROR, 0x00000000F000B024 },
            { PPSDORegistersID::PPSDO_STATUS_DAC_TUNED_VAL, 0x00000000F000B028 },
            { PPSDORegistersID::PPSDO_STATUS_ACCURACY, 0x00000000F000B02C },
            { PPSDORegistersID::PPSDO_STATUS_PPS_ACTIVE, 0x00000000F000B030 },
            { PPSDORegistersID::PPSDO_STATUS_STATE, 0x00000000F000B034 },
        } },
    { lime::eLMS_DEV::LMS_DEV_LIMESDRMINI_V2,
        {
            { PPSDORegistersID::PPSDO_ENABLE, 0x00000000F0002800 },
            { PPSDORegistersID::PPSDO_CONFIG_ONE_S_TARGET, 0x00000000F0002804 },
            { PPSDORegistersID::PPSDO_CONFIG_ONE_S_TOL, 0x00000000F0002808 },
            { PPSDORegistersID::PPSDO_CONFIG_TEN_S_TARGET, 0x00000000F000280C },
            { PPSDORegistersID::PPSDO_CONFIG_TEN_S_TOL, 0x00000000F0002810 },
            { PPSDORegistersID::PPSDO_CONFIG_HUNDRED_S_TARGET, 0x00000000F0002814 },
            { PPSDORegistersID::PPSDO_CONFIG_HUNDRED_S_TOL, 0x00000000F0002818 },
            { PPSDORegistersID::PPSDO_STATUS_ONE_S_ERROR, 0x00000000F000281C },
            { PPSDORegistersID::PPSDO_STATUS_TEN_S_ERROR, 0x00000000F0002820 },
            { PPSDORegistersID::PPSDO_STATUS_HUNDRED_S_ERROR, 0x00000000F0002824 },
            { PPSDORegistersID::PPSDO_STATUS_DAC_TUNED_VAL, 0x00000000F0002828 },
            { PPSDORegistersID::PPSDO_STATUS_ACCURACY, 0x00000000F000282C },
            { PPSDORegistersID::PPSDO_STATUS_PPS_ACTIVE, 0x00000000F0002830 },
            { PPSDORegistersID::PPSDO_STATUS_STATE, 0x00000000F0002834 },
        } }
};

MonitorResults::MonitorResults()
{
    dumpCount = 0;
    enableStatus = "false";
    error_1s = 0;
    error_10s = 0;
    error_100s = 0;
    dac = 0;
}

static uint64_t setField(uint64_t currRegValue, uint64_t newBitValue, int bitOffset, int size)
{
    uint64_t mask = ((1ULL << size) - 1ULL) << bitOffset;
    return ((currRegValue & ~mask) | ((newBitValue << bitOffset) & mask));
}

static uint64_t getField(uint64_t currRegValue, int bitOffset, int size)
{
    uint64_t mask = ((1ULL << size) - 1ULL) << bitOffset;
    return ((currRegValue & mask) >> bitOffset);
}

// ##################################################
// #### PPSDODriver member functions definitions ####
// ##################################################

void PPSDODriver::destroyCSR()
{
    if (mCSR_interface != nullptr)
        delete mCSR_interface;
}

bool PPSDODriver::isCSRImplemented()
{
    return (mCSR_interface != nullptr ? true : false);
}

uint64_t PPSDODriver::readRegister(uint64_t address, OpStatus* status)
{
    uint64_t value = mCSR_interface->ioRead64(address, status);
    lime::debug("DEBUG: Read 0x%016" PRIx64 " from register 0x%016" PRIx64 "", value, address);
    return value;
}

OpStatus PPSDODriver::writeRegister(uint64_t address, uint64_t value)
{
    lime::debug("DEBUG: Writing 0x%016" PRIx64 " value to register 0x%016" PRIx64 "", value, address);
    return mCSR_interface->ioWrite64(address, value);
}

uint64_t PPSDODriver::getSigned32bit(uint64_t address, OpStatus* status)
{
    uint64_t value = this->readRegister(address, status);
    if (*status != OpStatus::Success)
        return 0ULL;

    if (value & (1ULL << 31))
        value -= (1ULL << 32);

    return value;
}

uint64_t PPSDODriver::get_1s_error(OpStatus* status)
{
    return this->getSigned32bit(mpPPSDORegisterList->at(PPSDORegistersID::PPSDO_STATUS_ONE_S_ERROR), status);
}

uint64_t PPSDODriver::get_10s_error(OpStatus* status)
{
    return this->getSigned32bit(mpPPSDORegisterList->at(PPSDORegistersID::PPSDO_STATUS_TEN_S_ERROR), status);
}

uint64_t PPSDODriver::get_100s_error(OpStatus* status)
{
    return this->getSigned32bit(mpPPSDORegisterList->at(PPSDORegistersID::PPSDO_STATUS_HUNDRED_S_ERROR), status);
}

uint64_t PPSDODriver::getDacValue(OpStatus* status)
{
    return this->readRegister(mpPPSDORegisterList->at(PPSDORegistersID::PPSDO_STATUS_DAC_TUNED_VAL), status);
}

static const array<string, 4> accuracyLevelList = { "Disabled/Lowest", "1s Tune", "2s Tune", "3s Tune (Highest)" };

static string stateToStr(uint64_t state)
{
    if (state == 1ULL)
        return "Fine tune"s;
    else if (state == 0ULL)
        return "Coarse tune"s;

    return "Unknown"s;
}

static string accToStr(uint64_t accuracy)
{
    if (accuracy < 4)
        return accuracyLevelList[accuracy];

    return string("Unknown("s + std::to_string(accuracy) + ")"s);
}

OpStatus PPSDODriver::getStatus(string& prState, string& prAccuracy, string& prTpulse)
{
    OpStatus status = OpStatus::Success;
    uint64_t state = this->readRegister(mpPPSDORegisterList->at(PPSDORegistersID::PPSDO_STATUS_STATE), &status);
    if (status != OpStatus::Success)
    {
        mPPSDOStatusMsg = "Failed to read PPSDO_STATUS_STATE register with error: ";
        return status;
    }
    prState = stateToStr(state);

    uint64_t accuracy = this->readRegister(mpPPSDORegisterList->at(PPSDORegistersID::PPSDO_STATUS_ACCURACY), &status);
    if (status != OpStatus::Success)
    {
        mPPSDOStatusMsg = "Failed to read PPSDO_STATUS_ACCURACY register with error: ";
        return status;
    }
    prAccuracy = accToStr(accuracy);

    uint64_t tpulse = this->readRegister(mpPPSDORegisterList->at(PPSDORegistersID::PPSDO_STATUS_PPS_ACTIVE), &status);
    if (status != OpStatus::Success)
    {
        mPPSDOStatusMsg = "Failed to read PPSDO_STATUS_PPS_ACTIVE register with error: ";
        return status;
    }
    prTpulse = (tpulse ? "true" : "false");

    return status;
}

bool PPSDODriver::getEnabled(OpStatus* status)
{
    uint64_t value = this->readRegister(mpPPSDORegisterList->at(PPSDORegistersID::PPSDO_ENABLE), status);
    return static_cast<bool>(value & 1ULL);
}

OpStatus PPSDODriver::setEnabled(bool enable)
{
    OpStatus status = OpStatus::Success;
    uint64_t currRegValue = this->readRegister(mpPPSDORegisterList->at(PPSDORegistersID::PPSDO_ENABLE), &status);
    if (status != OpStatus::Success)
        return status;

    currRegValue = setField(currRegValue, static_cast<uint64_t>(enable), CONTROL_EN_OFFSET, CONTROL_EN_SIZE);
    status = this->writeRegister(mpPPSDORegisterList->at(PPSDORegistersID::PPSDO_ENABLE), currRegValue);

    return status;
}

string PPSDODriver::getPPSDOStatusMsg()
{
    return mPPSDOStatusMsg;
}

uint64_t PPSDODriver::getPPSDORegAddress(PPSDORegistersID id)
{
    return mpPPSDORegisterList->at(id);
}

static MediaType getDeviceMediaType(string& media)
{
    if (media.find("USB") != string::npos)
        return MediaType::USB;
    else if (media.find("PCIe") != string::npos)
        return MediaType::PCIE;

    return MediaType::UNDEFINED;
}

bool PPSDODriver::updatePPSDORegList(vector<DeviceHandle>& handles, string& devName)
{
    bool regListUpdated = false;
    DeviceHandle& mHandle = handles.front();

    // Select the same handle the connection used, not just the first in the list;
    // matching a different device would program the wrong CSR addresses
    if (!devName.empty())
    {
        const DeviceHandle wanted(std::string{ devName });
        bool matched = false;
        for (auto& iter : handles)
        {
            if (iter.IsEqualIgnoringEmpty(wanted) || lime::cli::FuzzyHandleMatch(iter, devName))
            {
                mHandle = iter;
                matched = true;
                break;
            }
        }
        if (!matched)
        {
            lime::error("limePPSDO: no device matches '%s'", devName.c_str());
            return false;
        }
    }

    switch (getDeviceMediaType(mHandle.media))
    {
    case MediaType::USB:
        if (mHandle.addr == "0403:601f"s || mHandle.addr == "374d:0019"s)
        {
            mpPPSDORegisterList = &SDR_PPSDO_Registers.find(eLMS_DEV::LMS_DEV_LIMESDRMINI_V2)->second;
            regListUpdated = true;
        }
        lime::debug("DEBUG: Selected CSR register list for LimeSDR Mini V2");
        break;

    case MediaType::PCIE:
        if (mHandle.name.find("XTRX"s) != string::npos)
        {
            mpPPSDORegisterList = &SDR_PPSDO_Registers.find(eLMS_DEV::LMS_DEV_LIMESDR_XTRX)->second;
            regListUpdated = true;
        }
        lime::debug("DEBUG: Selected CSR register list for LimeSDR XTRX");
        break;

    default:
        mpPPSDORegisterList = nullptr;
        break;
    }

    return regListUpdated;
}

// ###############################################
// #### limePPSDO helper function definitions ####
// ###############################################

static int printHeader()
{
    string header = "|    Dump    | Enabled |   1s Error   |  10s Error  |  100s Error  |  DAC Value  |    State    |      "
                    "Accuracy      | TPulse |";
    cout << setw(header.size()) << setfill('-') << "-" << setfill(' ') << endl;
    cout << header << endl;
    cout << setw(header.size()) << setfill('-') << "-" << setfill(' ') << endl;
    return header.size();
}

static string formatToFit(MonitorResults* results, int length)
{
    string formatedMessage(length, ' ');
    const char* format =
        "  %10" PRIu32 "    %5s    %11" PRIi64 "    %11" PRIi64 "   %11" PRIi64 "    0x%08" PRIX64 "    %11s   %18s   %5s";
    snprintf(formatedMessage.data(),
        formatedMessage.size(),
        format,
        results->dumpCount,
        results->enableStatus.c_str(),
        results->error_1s,
        results->error_10s,
        results->error_100s,
        results->dac,
        results->ppsdoState.c_str(),
        results->ppsdoAccuracy.c_str(),
        results->ppsdoTpulse.c_str());

    return formatedMessage;
}

static OpStatus runMonitoring(PPSDODriver* pDriver, uint32_t numDumps, std::chrono::milliseconds delay, int banner_interval)
{
    OpStatus status = OpStatus::Success;
    MonitorResults results;
    cout << "Monitoring PPSDO regulation loop (press Ctrl+C to stop):\n";
    uint32_t dumpCount = 0;
    bool continueLoop = false;
    int headerLength = 0;
    if (numDumps == 0)
        headerLength = printHeader();

    do
    {
        results.enableStatus = (pDriver->getEnabled(&status) ? "true" : "false");
        if (status != OpStatus::Success)
        {
            lime::error("ERROR: Monitoring mode failed to read PPSDO enable status with error: "s + ToString(status));
            return status;
        }

        results.error_1s = static_cast<int64_t>(pDriver->get_1s_error(&status));
        if (status != OpStatus::Success)
        {
            lime::error("ERROR: Monitoring mode failed to read PPSDO 1s error value with error: "s + ToString(status));
            return status;
        }

        results.error_10s = static_cast<int64_t>(pDriver->get_10s_error(&status));
        if (status != OpStatus::Success)
        {
            lime::error("ERROR: Monitoring mode failed to read PPSDO 10s error value with error: "s + ToString(status));
            return status;
        }

        results.error_100s = static_cast<int64_t>(pDriver->get_100s_error(&status));
        if (status != OpStatus::Success)
        {
            lime::error("ERROR: Monitoring mode failed to read PPSDO 100s error value with error: "s + ToString(status));
            return status;
        }

        results.dac = pDriver->getDacValue(&status);
        if (status != OpStatus::Success)
        {
            lime::error("ERROR: Monitoring mode failed to read PPSDO DAC value with error: "s + ToString(status));
            return status;
        }

        status = pDriver->getStatus(results.ppsdoState, results.ppsdoAccuracy, results.ppsdoTpulse);
        if (status != OpStatus::Success)
        {
            lime::error("ERROR: Monitoring mode failed to read PPSDO Status values. Reason: "s + pDriver->getPPSDOStatusMsg() +
                        ToString(status));
            return status;
        }

        if ((dumpCount % banner_interval == 0 && dumpCount != numDumps))
            headerLength = printHeader();

        results.dumpCount = ++dumpCount;
        cout << formatToFit(&results, headerLength) << endl;

        std::this_thread::sleep_for(delay);
        continueLoop = (dumpCount < numDumps || numDumps == 0);

    } while (continueLoop && !cleanUp);

    cout << "Monitoring finished\n";
    return status;
}

static void dumpRegisters(PPSDODriver* pDriver, int numberOfDumps, double delay)
{
    cout << "Note: Raw register dump not available via named CSRs, use --check for full status\n";
}

static OpStatus resetPPSDO(PPSDODriver* pDriver, std::chrono::milliseconds rstDelay)
{
    OpStatus status = OpStatus::Success;
    cout << "Resetting PPSDO...\n";
    status = pDriver->setEnabled(false);
    if (status != OpStatus::Success)
    {
        lime::error("ERROR: Failed to reset PPSDO with error: "s + ToString(status));
        return status;
    }

    std::this_thread::sleep_for(rstDelay);
    status = pDriver->setEnabled(true);
    if (status != OpStatus::Success)
    {
        lime::error("ERROR: Failed to reset PPSDO with error: "s + ToString(status));
        return status;
    }
    cout << "PPSDO reset complete (re-enabled)\n";
    return status;
}

static OpStatus enablePPSDO(PPSDODriver* pDriver, double clk, double ppm)
{
    OpStatus status = OpStatus::Success;
    double freq = clk * 1e6;
    lime::info("INFO: Selected frequency freq = "s + to_string(freq) + " Hz;"s);

    uint64_t target_1s = static_cast<uint64_t>(freq);
    lime::info("INFO: target_1s = freq = "s + to_string(target_1s) + ";"s);

    if (target_1s > std::numeric_limits<uint32_t>::max())
    {
        lime::error("ERROR: Calculated frequency target values are not achievable. Please check --clk-freq argument."s);
        status = OpStatus::Error;
        return status;
    }

    uint64_t target_10s = static_cast<uint64_t>(freq * 10);
    lime::info("INFO: target_10s = freq * 10 = "s + to_string(freq) + " * 10 = "s + to_string(target_10s) + ";"s);

    uint64_t target_100s = static_cast<uint64_t>(freq * 100);
    lime::info("INFO: target_100s = freq * 100 = "s + to_string(freq) + " * 100 = "s + to_string(target_100s) + ";"s);

    uint64_t tol_1s_hz = static_cast<uint64_t>(ceil(freq * ppm / 1e6));
    lime::info("INFO: tol_1s_hz = ceil(freq * ppm / 1e6) = ceil("s + to_string(freq) + " * "s + to_string(ppm) + " / 1e6) = "s +
               to_string(tol_1s_hz) + " Hz;"s);

    uint64_t tol_10s_hz = static_cast<uint64_t>(ceil(freq * ppm / 1e5));
    lime::info("INFO: tol_10s_hz = ceil(freq * ppm / 1e5) = ceil("s + to_string(freq) + " * "s + to_string(ppm) + " / 1e5) = "s +
               to_string(tol_10s_hz) + " Hz;"s);

    uint64_t tol_100s_hz = static_cast<uint64_t>(ceil(freq * ppm / 1e4));
    lime::info("INFO: tol_100s_hz = ceil(freq * ppm / 1e4) = ceil("s + to_string(freq) + " * "s + to_string(ppm) + " / 1e4) = "s +
               to_string(tol_100s_hz) + " Hz;"s);

    status = pDriver->writeRegister(pDriver->getPPSDORegAddress(PPSDORegistersID::PPSDO_CONFIG_ONE_S_TARGET), target_1s);
    if (status != OpStatus::Success)
    {
        lime::error("ERROR: PPSDO enable failed to write PPSDO_CONFIG_ONE_S_TARGET register with error: "s + ToString(status));
        return status;
    }

    status = pDriver->writeRegister(pDriver->getPPSDORegAddress(PPSDORegistersID::PPSDO_CONFIG_ONE_S_TOL), tol_1s_hz);
    if (status != OpStatus::Success)
    {
        lime::error("ERROR: PPSDO enable failed to write PPSDO_CONFIG_ONE_S_TOL register with error: "s + ToString(status));
        return status;
    }

    status = pDriver->writeRegister(pDriver->getPPSDORegAddress(PPSDORegistersID::PPSDO_CONFIG_TEN_S_TARGET), target_10s);
    if (status != OpStatus::Success)
    {
        lime::error("ERROR: PPSDO enable failed to write PPSDO_CONFIG_TEN_S_TARGET register with error: "s + ToString(status));
        return status;
    }

    status = pDriver->writeRegister(pDriver->getPPSDORegAddress(PPSDORegistersID::PPSDO_CONFIG_TEN_S_TOL), tol_10s_hz);
    if (status != OpStatus::Success)
    {
        lime::error("ERROR: PPSDO enable failed to write PPSDO_CONFIG_TEN_S_TOL register with error: "s + ToString(status));
        return status;
    }

    status = pDriver->writeRegister(pDriver->getPPSDORegAddress(PPSDORegistersID::PPSDO_CONFIG_HUNDRED_S_TARGET), target_100s);
    if (status != OpStatus::Success)
    {
        lime::error("ERROR: PPSDO enable failed to write PPSDO_CONFIG_HUNDRED_S_TARGET register with error: "s + ToString(status));
        return status;
    }

    status = pDriver->writeRegister(pDriver->getPPSDORegAddress(PPSDORegistersID::PPSDO_CONFIG_HUNDRED_S_TOL), tol_100s_hz);
    if (status != OpStatus::Success)
    {
        lime::error("ERROR: PPSDO enable failed to write PPSDO_CONFIG_HUNDRED_S_TOL register with error: "s + ToString(status));
        return status;
    }

    uint64_t control = 0;
    uint64_t clk_sel =
        0; //(ceil(clk) == 10.0 || floor(clk) == 10.0 ) ? 1ULL : 0ULL;   // TODO: Update this later with the new revision of data sheet, for now defaulting to clk_sel 0
    control = setField(control, clk_sel, CONTROL_CLK_SEL_OFFSET, CONTROL_CLK_SEL_SIZE);
    control = setField(control, 1ULL, CONTROL_EN_OFFSET, CONTROL_EN_SIZE);

    status = pDriver->writeRegister(pDriver->getPPSDORegAddress(PPSDORegistersID::PPSDO_ENABLE), control);
    if (status != OpStatus::Success)
    {
        lime::error("ERROR: PPSDO enable failed to write PPSDO_ENABLE register with error: "s + ToString(status));
        return status;
    }

    cout << "PPSDO enabled: CLK_SEL = " << clk_sel << " (" << clk << " MHz), " << ppm << " ppm tolerance (1s tol = " << tol_1s_hz
         << " Hz, 10s = " << tol_10s_hz << " Hz, 100s = " << tol_100s_hz << " Hz).\n";

    return status;
}

static OpStatus disablePPSDO(PPSDODriver* pDriver)
{
    OpStatus status = pDriver->writeRegister(pDriver->getPPSDORegAddress(PPSDORegistersID::PPSDO_ENABLE), 0ULL);
    if (status != OpStatus::Success)
    {
        lime::error("ERROR: PPSDO disable failed to write PPSDO_ENABLE register with error: "s + ToString(status));
        return status;
    }

    cout << "PPSDO disabled\n";
    return status;
}

int main(int argc, char** argv)
{
    signal(SIGINT, keyBoardInt);
    // clang-format off
   args::ArgumentParser                    parser("limePPSDO - PPS Disciplined Oscilator control and monitoring. Currently supported PPS sources - GPS", "");
   args::HelpFlag                          help(parser, "help", "This help", {'h', "help"});

   args::Group                             commands(parser, "COMMANDS", args::Group::Validators::AtLeastOne); // NOLINT(cppcoreguidelines-slicing)
   args::Flag                              check(commands, "check", "Run monitoring mode", {"check"} );
   args::Flag                              dump(commands, "dump", "Dump registers", {"dump"});
   args::Flag                              reset(commands, "reset", "Reset PPSDO", {"reset"});
   args::Flag                              enable(commands, "enable", "Configure and enable PPSDO", {"enable"});
   args::Flag                              disable(commands, "disable", "Disable PPSDO", {"disable"});

   args::Group                             arguments(parser, "ARGUMENTS", args::Group::Validators::DontCare, args::Options::Global); // NOLINT(cppcoreguidelines-slicing)
   args::ValueFlag<std::string>            logFlag(arguments, "", "Enable additional device, API and limePPSDO app log output. Log verbosity: info, warning, error, verbose, debug. Log level \'info\' prints intermediate calculations. Log level \'debug\' prints detailed CSR register R/W operations.", {'l', "log"}, "error");
   args::ValueFlag<std::string>            deviceFlag(arguments, "name", "Specifies which device to use", {"device"}, "");
   args::ValueFlag<uint32_t>               num(arguments, "iter", "Number of iterations (for --check: 0 for infinite; for --dump: default 1 if not specified)", {'n', "num"}, 0);
   args::ValueFlag<double>                 delay(arguments, "time", "Delay between iterations (seconds, for --check and --dump)", {'d', "delay"}, 1.0);
   args::ValueFlag<int>                    banner(arguments, "interval", "Banner repeat interval (for --check)", {'b', "banner"}, 10);
   args::ValueFlag<double>                 reset_delay(arguments, "time", "Delay after disable before re-enable (seconds, for --reset)", {'r', "reset-delay"}, 2.0);
   args::ValueFlag<double>                 clk_freq(arguments, "MHz", "Clock frequency in MHz", {'c', "clk-freq"});
   args::ValueFlag<double>                 ppm(arguments, "ppm", "Tolerance in ppm", {'p', "ppm"});
    // clang-format on

    try
    {
        parser.ParseCLI(argc, argv);

        if (enable)
        {
            double clk = args::get(clk_freq);
            double m_ppm = args::get(ppm);
            if (clk <= 0 || m_ppm <= 0)
                throw args::UsageError(
                    "ERROR: Enable command requires flags --clk-freq and --ppm to be set for each LimeSDR device!");
        }

        if (check)
        {
            int mbanner = args::get(banner);
            if (mbanner == 0)
                throw args::UsageError("ERROR: Check command --banner flag cannot be set to 0!");
        }

    } catch (args::Help&)
    {
        cout << parser << endl;
        return EXIT_SUCCESS;
    } catch (args::ValidationError& e)
    {
        cerr << "ERROR: Select atleast one COMMAND from the list!" << endl;
        cerr << parser << endl;
        return EXIT_FAILURE;
    } catch (args::UsageError& e)
    {
        cerr << e.what() << endl;
        return EXIT_FAILURE;
    } catch (const std::exception& e)
    {
        cerr << parser << endl;
        cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    std::string devName = args::get(deviceFlag);

    auto handles = DeviceRegistry::enumerate();
    if (handles.size() == 0)
    {
        cerr << "No devices found"sv << endl;
        return EXIT_FAILURE;
    }

    SDRDevice* device = ConnectToFilteredOrDefaultDevice(devName);
    if (!device)
    {
        cerr << "ERROR: Failed to connect to SDR device!\n";
        return EXIT_FAILURE;
    }

    PPSDODriver driver(device->getICSR());
    if (!driver.isCSRImplemented())
    {
        cerr << "ERROR: Selected SDR device does not support CSR interface!\n";
        DeviceRegistry::freeDevice(device);
        return EXIT_FAILURE;
    }

    logVerbosity = strToLogLevel(args::get(logFlag));
    device->SetMessageLogCallback(lime::cli::LogCallback);
    lime::registerLogHandler(lime::cli::LogCallback);
    registerLogHandler(lime::cli::CStyleLogCallback);

    if (!driver.updatePPSDORegList(handles, devName))
    {
        driver.destroyCSR();
        DeviceRegistry::freeDevice(device);
        lime::error("ERROR: Failed to select CSR register list for selected SDR device!");
        return EXIT_FAILURE;
    }

    OpStatus runStatus = OpStatus::Success;
    try
    {
        if (reset)
        {
            int resetDelay = static_cast<int>(args::get(reset_delay) * 1000.0);
            runStatus = resetPPSDO(&driver, std::chrono::milliseconds(resetDelay));
        }

        if (enable && runStatus == OpStatus::Success)
        {
            runStatus = enablePPSDO(&driver, args::get(clk_freq), args::get(ppm));
        }

        if (check && runStatus == OpStatus::Success)
        {
            int convDelay = static_cast<int>(args::get(delay) * 1000.0);
            runStatus = runMonitoring(&driver, args::get(num), std::chrono::milliseconds(convDelay), args::get(banner));
        }

        if (dump && runStatus == OpStatus::Success)
        {
            dumpRegisters(&driver, args::get(num), args::get(delay));
        }

        if (disable && runStatus == OpStatus::Success)
        {
            runStatus = disablePPSDO(&driver);
        }

    } catch (const std::exception& e)
    {
        driver.destroyCSR();
        DeviceRegistry::freeDevice(device);
        std::cerr << e.what() << '\n';
        return EXIT_FAILURE;
    }

    if (cleanUp)
        cerr << "Keyboard interrupt Ctrl+C detected! Cleaning up ...\n";

    driver.destroyCSR();
    DeviceRegistry::freeDevice(device);
    return runStatus == OpStatus::Success ? EXIT_SUCCESS : EXIT_FAILURE;
}