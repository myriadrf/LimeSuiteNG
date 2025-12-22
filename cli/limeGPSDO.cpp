#include "limeGPSDO.hpp"

using namespace std;
using namespace lime;
using namespace lime::cli;

MonitorResults::MonitorResults()
{
   dumpCount = 0;
   enableStatus = "false";
   error_1s = 0;
   error_10s = 0;
   error_100s = 0;
   dac = 0;
}

// ##################################################
// #### GPSDODriver member functions definitions ####
// ##################################################

void GPSDODriver::destroyCSR()
{
   if (mCSR_interface != nullptr) 
      delete mCSR_interface;
}

bool GPSDODriver::isCSRImplemented()
{
   return (mCSR_interface != nullptr ? true : false);
}

uint64_t GPSDODriver::readRegister(uint64_t address, OpStatus * status)
{
   uint64_t value = mCSR_interface->ioRead64(address, status);
   lime::debug("DEBUG: Read 0x%016" PRIx64 " from register 0x%016" PRIx64 "", value, address);
   return value;
}

OpStatus GPSDODriver::writeRegister(uint64_t address, uint64_t value)
{
   lime::debug("DEBUG: Writing 0x%016" PRIx64 " value to register 0x%016" PRIx64 "", value, address);
   return mCSR_interface->ioWrite64(address, value);
}

uint64_t GPSDODriver::getSigned32bit(uint64_t address, OpStatus * status)
{
   uint64_t value = this->readRegister(address, status);
   if(*status != OpStatus::Success)
      return 0ULL;
   
   if(value & (1ULL << 31))
      value -= (1ULL << 32);

   return value;
}

uint64_t GPSDODriver::get_1s_error(OpStatus * status)
{
   return this->getSigned32bit((*mpGPSDORegisterList).find(GPSDORegistersID::PPSDO_STATUS_ONE_S_ERROR)->second, status);
}

uint64_t GPSDODriver::get_10s_error(OpStatus * status)
{
   return this->getSigned32bit((*mpGPSDORegisterList).find(GPSDORegistersID::PPSDO_STATUS_TEN_S_ERROR)->second, status);
}

uint64_t GPSDODriver::get_100s_error(OpStatus * status)
{
   return this->getSigned32bit((*mpGPSDORegisterList).find(GPSDORegistersID::PPSDO_STATUS_HUNDRED_S_ERROR)->second, status);
}

uint64_t GPSDODriver::getDacValue(OpStatus * status)
{
   return this->readRegister((*mpGPSDORegisterList).find(GPSDORegistersID::PPSDO_STATUS_DAC_TUNED_VAL)->second, status);
}

static const array<string,4> accuracyLevelList = {"Disabled/Lowest", "1s Tune", "2s Tune", "3s Tune (Highest)"};

static string stateToStr(uint64_t state)
{
   if(state == 1ULL)
      return "Fine tune"s;
   else if(state == 0ULL)
      return "Coarse tune"s;
   
   return "Unknown"s;
}

static string accToStr(uint64_t accuracy)
{
   if(accuracy < 4)
      return accuracyLevelList[accuracy];
   
   return string("Unknown("s + std::to_string(accuracy) + ")"s);
}

OpStatus GPSDODriver::getStatus(string& prState, string& prAccuracy, string& prTpulse)
{
   OpStatus status = OpStatus::Success;
   uint64_t state = this->readRegister((*mpGPSDORegisterList).find(GPSDORegistersID::PPSDO_STATUS_STATE)->second, &status);
   if(status != OpStatus::Success)
   {
      mGPSDOStatusMsg = "Failed to read PPSDO_STATUS_STATE register with error: ";
      return status;
   }
   prState = stateToStr(state);

   uint64_t accuracy = this->readRegister((*mpGPSDORegisterList).find(GPSDORegistersID::PPSDO_STATUS_ACCURACY)->second, &status);
   if(status != OpStatus::Success)
   {
      mGPSDOStatusMsg = "Failed to read PPSDO_STATUS_ACCURACY register with error: ";
      return status;
   }
   prAccuracy = accToStr(accuracy);

   uint64_t tpulse = this->readRegister((*mpGPSDORegisterList).find(GPSDORegistersID::PPSDO_STATUS_PPS_ACTIVE)->second, &status);
   if(status != OpStatus::Success)
   {
      mGPSDOStatusMsg = "Failed to read PPSDO_STATUS_PPS_ACTIVE register with error: ";
      return status;
   }
   prTpulse = (tpulse ? "true" : "false");

   return status;   
}


bool GPSDODriver::getEnabled(OpStatus * status)
{
   uint64_t value = this->readRegister((*mpGPSDORegisterList).find(GPSDORegistersID::PPSDO_ENABLE)->second, status);
   return static_cast<bool>(value & 1ULL);
}

OpStatus GPSDODriver::setEnabled(bool enable)
{
   OpStatus status = OpStatus::Success;
   uint64_t currRegValue = this->readRegister((*mpGPSDORegisterList).find(GPSDORegistersID::PPSDO_ENABLE)->second, &status);
   if(status != OpStatus::Success)
      return status;
   
   currRegValue = setField(currRegValue, static_cast<uint64_t>(enable), CONTROL_EN_OFFSET, CONTROL_EN_SIZE);
   status = this->writeRegister((*mpGPSDORegisterList).find(GPSDORegistersID::PPSDO_ENABLE)->second, currRegValue);

   return status;
}

string GPSDODriver::getGPSDOStatusMsg()
{
   return mGPSDOStatusMsg;
}

uint64_t setField(uint64_t currRegValue, uint64_t newBitValue, int bitOffset, int size)
{
   uint64_t mask = ((1ULL << size) - 1ULL) << bitOffset;
   return ((currRegValue & ~mask) | ((newBitValue << bitOffset) & mask));
}

uint64_t getField(uint64_t currRegValue, int bitOffset, int size)
{
   uint64_t mask = ((1ULL << size) - 1ULL) << bitOffset;
   return ((currRegValue & mask) >> bitOffset);
}

uint64_t GPSDODriver::getGPSDORegAddress(GPSDORegistersID id)
{
   return (*mpGPSDORegisterList).find(id)->second;
}

static MediaType getDeviceMediaType(string& media)
{
   if(media.find("USB") != string::npos)
      return MediaType::USB;
   else if(media.find("PCIe") != string::npos)
      return MediaType::PCIE;
   
   return MediaType::UNDEFINED;
}

bool GPSDODriver::updateGPSDORegList(vector<DeviceHandle>& handles, string& devName)
{
   bool regListUpdated = false;
   DeviceHandle& mHandle = handles.front();

   // Select the correct handle from list
   if(!devName.empty())
   {
      for(auto& iter : handles)
      {
         if(iter.name == devName)
         {
            mHandle = iter;
            break;
         }  
      }
   }

   switch (getDeviceMediaType(mHandle.media))
   {
   case MediaType::USB:
      if(mHandle.addr == "0403:601f"s || mHandle.addr == "374d:0019"s)
      {   
         mpGPSDORegisterList = &SDR_GPSDO_Registers[eLMS_DEV::LMS_DEV_LIMESDRMINI_V2];
         regListUpdated = true;
      }
      lime::debug("DEBUG: Selected CSR register list for LimeSDR Mini V2");
      break;
   
   case MediaType::PCIE:
      if(mHandle.name.find("XTRX"s) != string::npos)
      {
         mpGPSDORegisterList = &SDR_GPSDO_Registers[eLMS_DEV::LMS_DEV_LIMESDR_XTRX];
         regListUpdated = true;
      }
      lime::debug("DEBUG: Selected CSR register list for LimeSDR XTRX");
      break;
   
   default:
      mpGPSDORegisterList = nullptr;
      break;
   }

   return regListUpdated;
}

// ###############################################
// #### limeGPSDO helper function definitions ####
// ###############################################

static int printHeader()
{
   string header = "|    Dump    | Enabled |   1s Error   |  10s Error  |  100s Error  |  DAC Value  |    State    |      Accuracy      | TPulse |";
   cout << setw(header.size()) << setfill('-') << "-" << setfill(' ') << endl;
   cout << header << endl;
   cout << setw(header.size()) << setfill('-') << "-" << setfill(' ') << endl;
   return header.size();
}

static string formatToFit(MonitorResults * results, int length)
{
   string formatedMessage(length, ' ');
   const char * format = "  %10i    %5s    %11" PRIi64 "    %11" PRIi64 "   %11"  PRIi64 "    0x%08" PRIX64 "    %11s   %18s   %5s";
   snprintf(formatedMessage.data(), formatedMessage.size(), format,
             results->dumpCount, results->enableStatus.c_str(), results->error_1s,
             results->error_10s, results->error_100s, results->dac, results->gpsdoState.c_str(),
             results->gpsdoAccuracy.c_str(), results->gpsdoTpulse.c_str());
   
   return formatedMessage;
}

static OpStatus runMonitoring(GPSDODriver * pDriver, uint32_t numDumps, std::chrono::milliseconds delay, int banner_interval)
{
   OpStatus status = OpStatus::Success;
   MonitorResults results;
   cout << "Monitoring GPSDO regulation loop (press Ctrl+C to stop):\n";
   uint32_t dumpCount = 0;
   bool continueLoop = false;
   int headerLength = 0;
   if(numDumps == 0)
      headerLength = printHeader();

   do
   {
      results.enableStatus = (pDriver->getEnabled(&status) ? "true" : "false");
      if(status != OpStatus::Success)
      {
         lime::error("ERROR: Monitoring mode failed to read GPSDO enable status with error: "s + ToString(status));
         return status;
      }

      results.error_1s = static_cast<int64_t>(pDriver->get_1s_error(&status));
      if(status != OpStatus::Success)
      {
         lime::error("ERROR: Monitoring mode failed to read GPSDO 1s error value with error: "s + ToString(status));
         return status;
      }

      results.error_10s = static_cast<int64_t>(pDriver->get_10s_error(&status));
      if(status != OpStatus::Success)
      {
         lime::error("ERROR: Monitoring mode failed to read GPSDO 10s error value with error: "s + ToString(status));
         return status;
      }

      results.error_100s = static_cast<int64_t>(pDriver->get_100s_error(&status));
      if(status != OpStatus::Success)
      {
         lime::error("ERROR: Monitoring mode failed to read GPSDO 100s error value with error: "s + ToString(status));
         return status;
      }

      results.dac = pDriver->getDacValue(&status);
      if(status != OpStatus::Success)
      {
         lime::error("ERROR: Monitoring mode failed to read GPSDO DAC value with error: "s + ToString(status));
         return status;
      }
      
      status = pDriver->getStatus(results.gpsdoState, results.gpsdoAccuracy, results.gpsdoTpulse);
      if(status != OpStatus::Success)
      {
         lime::error("ERROR: Monitoring mode failed to read GPSDO Status values. Reason: "s + pDriver->getGPSDOStatusMsg() + ToString(status));
         return status;
      }

      if((dumpCount % banner_interval == 0 && dumpCount != numDumps))
         headerLength = printHeader();

      results.dumpCount = ++dumpCount;
      cout << formatToFit(&results, headerLength) << endl;
      
      std::this_thread::sleep_for(delay);
      continueLoop = (dumpCount < numDumps || numDumps == 0);

   } while (continueLoop && !cleanUp);

   cout << "Monitoring finished\n";
   return status;
}

static void dumpRegisters(GPSDODriver * pDriver, int numberOfDumps, double delay)
{
   cout << "Note: Raw register dump not available via named CSRs, use --check for full status\n";
}

static OpStatus resetGPSDO(GPSDODriver * pDriver, std::chrono::milliseconds rstDelay)
{
   OpStatus status = OpStatus::Success;
   cout << "Resetting GPSDO...\n";
   status = pDriver->setEnabled(false);
   if(status != OpStatus::Success)
   {
      lime::error("ERROR: Failed to reset GPSDO with error: "s + ToString(status));
      return status;
   }
   
   std::this_thread::sleep_for(rstDelay);
   status = pDriver->setEnabled(true);
   if(status != OpStatus::Success)
   {
      lime::error("ERROR: Failed to reset GPSDO with error: "s + ToString(status));
      return status;
   }
   cout << "GPSDO reset complete (re-enabled)\n";
   return status;
}

static OpStatus enableGPSDO(GPSDODriver * pDriver, double clk, double ppm)
{
   OpStatus status = OpStatus::Success;
   double freq = clk * 1e6;
   lime::info("INFO: Selected frequency freq = "s + to_string(freq) + " Hz;"s);

   uint64_t target_1s = static_cast<uint64_t>(freq);
   lime::info("INFO: target_1s = freq = "s + to_string(target_1s) + ";"s);

   if(target_1s > std::numeric_limits<uint32_t>::max())
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
   lime::info("INFO: tol_1s_hz = ceil(freq * ppm / 1e6) = ceil("s + to_string(freq) + " * "s + to_string(ppm) + " / 1e6) = "s + to_string(tol_1s_hz) + " Hz;"s);

   uint64_t tol_10s_hz = static_cast<uint64_t>(ceil(freq * ppm / 1e5));
   lime::info("INFO: tol_10s_hz = ceil(freq * ppm / 1e5) = ceil("s + to_string(freq) + " * "s + to_string(ppm) + " / 1e5) = "s + to_string(tol_10s_hz) + " Hz;"s);

   uint64_t tol_100s_hz = static_cast<uint64_t>(ceil(freq * ppm / 1e4));
   lime::info("INFO: tol_100s_hz = ceil(freq * ppm / 1e4) = ceil("s + to_string(freq) + " * "s + to_string(ppm) + " / 1e4) = "s + to_string(tol_100s_hz) + " Hz;"s);

   status = pDriver->writeRegister(pDriver->getGPSDORegAddress(GPSDORegistersID::PPSDO_CONFIG_ONE_S_TARGET), target_1s);
   if(status != OpStatus::Success)
   {
      lime::error("ERROR: GPSDO enable failed to write PPSDO_CONFIG_ONE_S_TARGET register with error: "s + ToString(status));
      return status;
   }
      
   status = pDriver->writeRegister(pDriver->getGPSDORegAddress(GPSDORegistersID::PPSDO_CONFIG_ONE_S_TOL), tol_1s_hz);
   if(status != OpStatus::Success)
   {
      lime::error("ERROR: GPSDO enable failed to write PPSDO_CONFIG_ONE_S_TOL register with error: "s + ToString(status));
      return status;
   }

   status = pDriver->writeRegister(pDriver->getGPSDORegAddress(GPSDORegistersID::PPSDO_CONFIG_TEN_S_TARGET), target_10s);
   if(status != OpStatus::Success)
   {
      lime::error("ERROR: GPSDO enable failed to write PPSDO_CONFIG_TEN_S_TARGET register with error: "s + ToString(status));
      return status;
   }

   status = pDriver->writeRegister(pDriver->getGPSDORegAddress(GPSDORegistersID::PPSDO_CONFIG_TEN_S_TOL), tol_10s_hz);
   if(status != OpStatus::Success)
   {
      lime::error("ERROR: GPSDO enable failed to write PPSDO_CONFIG_TEN_S_TOL register with error: "s + ToString(status));
      return status;
   }

   status = pDriver->writeRegister(pDriver->getGPSDORegAddress(GPSDORegistersID::PPSDO_CONFIG_HUNDRED_S_TARGET), target_100s);
   if(status != OpStatus::Success)
   {
      lime::error("ERROR: GPSDO enable failed to write PPSDO_CONFIG_HUNDRED_S_TARGET register with error: "s + ToString(status));
      return status;
   }
   
   status = pDriver->writeRegister(pDriver->getGPSDORegAddress(GPSDORegistersID::PPSDO_CONFIG_HUNDRED_S_TOL), tol_100s_hz);
   if(status != OpStatus::Success)
   {
      lime::error("ERROR: GPSDO enable failed to write PPSDO_CONFIG_HUNDRED_S_TOL register with error: "s + ToString(status));
      return status;
   }

   uint64_t control = 0;
   uint64_t clk_sel = 0; //(ceil(clk) == 10.0 || floor(clk) == 10.0 ) ? 1ULL : 0ULL;   // TODO: Update this later with the new revision of data sheet, for now defaulting to clk_sel 0
   control = setField(control, clk_sel, CONTROL_CLK_SEL_OFFSET, CONTROL_CLK_SEL_SIZE);
   control = setField(control, 1ULL, CONTROL_EN_OFFSET, CONTROL_EN_SIZE);

   status = pDriver->writeRegister(pDriver->getGPSDORegAddress(GPSDORegistersID::PPSDO_ENABLE), control);
   if(status != OpStatus::Success)
   {
      lime::error("ERROR: GPSDO enable failed to write PPSDO_ENABLE register with error: "s + ToString(status));
      return status;
   }

   cout << "GPSDO enabled: CLK_SEL = " << clk_sel << " (" << clk << " MHz), " << ppm << " ppm tolerance (1s tol = " << tol_1s_hz << " Hz, 10s = "
        << tol_10s_hz << " Hz, 100s = " << tol_100s_hz << " Hz).\n";

   return status;
}

static OpStatus disableGPSDO(GPSDODriver * pDriver)
{
   OpStatus status = pDriver->writeRegister(pDriver->getGPSDORegAddress(GPSDORegistersID::PPSDO_ENABLE), 0ULL);
   if(status != OpStatus::Success)
   {
      lime::error("ERROR: GPSDO disable failed to write PPSDO_ENABLE register with error: "s + ToString(status));
      return status;
   }

   cout << "GPSDO disabled\n";
   return status;
}

int main(int argc, char** argv)
{
   signal(SIGINT, keyBoardInt);
   // clang-format off
   args::ArgumentParser                    parser("limeGPSDO - GPS Disciplined Oscilator control and monitoring", "");
   args::HelpFlag                          help(parser, "help", "This help", {'h', "help"});

   args::Group                             commands(parser, "COMMANDS", args::Group::Validators::AtLeastOne); // NOLINT(cppcoreguidelines-slicing)
   args::Flag                              check(commands, "check", "Run monitoring mode", {"check"} );
   args::Flag                              dump(commands, "dump", "Dump registers", {"dump"});
   args::Flag                              reset(commands, "reset", "Reset GPSDO", {"reset"});
   args::Flag                              enable(commands, "enable", "Configure and enable GPSDO", {"enable"});
   args::Flag                              disable(commands, "disable", "Disable GPSDO", {"disable"});

   args::Group                             arguments(parser, "ARGUMENTS", args::Group::Validators::DontCare, args::Options::Global); // NOLINT(cppcoreguidelines-slicing)
   args::ValueFlag<std::string>            logFlag(arguments, "", "Enable additional device, API and limeGPSDO app log output. Log verbosity: info, warning, error, verbose, debug. Log level \'info\' prints intermediate calculations in limeGPSDO app. Log level \'debug\' prints detailed CSR register R/W operations.", {'l', "log"}, "error");
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

      if(enable)
      {
         double clk = args::get(clk_freq);
         double m_ppm = args::get(ppm);
         if(clk == 0 && m_ppm == 0)
            throw args::UsageError("ERROR: Enable command requires flags --clk-freq and --ppm to be set for each LimeSDR device!");
      }

      if(check)
      {
         int mbanner = args::get(banner);
         if (mbanner == 0)
            throw args::UsageError("ERROR: Check command --banner flag cannot be set to 0!");
      }

   } catch (args::Help&)
   {
      cout << parser << endl;
      return EXIT_SUCCESS;
   }
   catch (args::ValidationError& e)
   {
      std::cout << "ERROR: Select atleast one COMMAND from the list!" << endl;
      cout << parser << endl;
      return EXIT_SUCCESS;
   }
   catch (args::UsageError& e)
   {
      std::cout << e.what() << endl;
      return EXIT_SUCCESS;
   }
   catch (const std::exception& e)
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

   GPSDODriver driver(device->getICSR());
   if(!driver.isCSRImplemented())
   {
      cerr << "ERROR: Selected SDR device does not support CSR interface!\n";
      DeviceRegistry::freeDevice(device);
      return EXIT_FAILURE;
   }

   logVerbosity = strToLogLevel(args::get(logFlag));
   device->SetMessageLogCallback(lime::cli::LogCallback);
   lime::registerLogHandler(lime::cli::LogCallback);
   registerLogHandler(lime::cli::CStyleLogCallback);

   if(!driver.updateGPSDORegList(handles, devName))
   {
      driver.destroyCSR();
      DeviceRegistry::freeDevice(device);
      lime::error("ERROR: Failed to select CSR register list for selected SDR device!");
      return EXIT_FAILURE;
   }

   OpStatus runStatus = OpStatus::Success;
   try
   {
      if(reset)
      {
         int resetDelay = static_cast<int>(args::get(reset_delay) * 1000.0);
         runStatus = resetGPSDO(&driver, std::chrono::milliseconds(resetDelay));
      }

      if(enable && runStatus == OpStatus::Success)
      {
         runStatus = enableGPSDO(&driver, args::get(clk_freq), args::get(ppm));
      }

      if(check && runStatus == OpStatus::Success)
      {
         int convDelay = static_cast<int>(args::get(delay) * 1000.0);
         runStatus = runMonitoring(&driver, args::get(num), std::chrono::milliseconds(convDelay), args::get(banner));
      }

      if(dump && runStatus == OpStatus::Success)
      {
         dumpRegisters(&driver, args::get(num), args::get(delay));
      }

      if(disable && runStatus == OpStatus::Success)
      {
         runStatus = disableGPSDO(&driver);
      }

   }
   catch(const std::exception& e)
   {
      driver.destroyCSR();
      DeviceRegistry::freeDevice(device);
      std::cerr << e.what() << '\n';
      return EXIT_FAILURE;
   }

   if(cleanUp)
      cerr << "Keyboard interrupt Ctrl+C detected! Cleaning up ...\n";

      
   driver.destroyCSR();
   DeviceRegistry::freeDevice(device);
   return EXIT_SUCCESS;
}