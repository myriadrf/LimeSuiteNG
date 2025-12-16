#include "limeGPSDO.hpp"

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

uint64_t GPSDODriver::readRegister(uint64_t address, OpStatus * status)
{
   return mCSR_interface->ioRead64(address, status);
}

OpStatus GPSDODriver::writeRegister(uint64_t address, uint64_t value)
{
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
   return this->getSigned32bit(GPSDODriver::reg_pps_1s_err, status);
}

uint64_t GPSDODriver::get_10s_error(OpStatus * status)
{
   return this->getSigned32bit(GPSDODriver::reg_pps_10s_err, status);
}

uint64_t GPSDODriver::get_100s_error(OpStatus * status)
{
   return this->getSigned32bit(GPSDODriver::reg_pps_100s_err, status)   ;
}

uint64_t GPSDODriver::getDacValue(OpStatus * status)
{
   return this->readRegister(GPSDODriver::reg_dac_tuned_val, status);
}

OpStatus GPSDODriver::getStatus(array<string, 3>& GPSDOStatus)
{
   OpStatus status = OpStatus::Success;
   uint64_t state = this->readRegister(GPSDODriver::reg_status_state, &status);
   if(status != OpStatus::Success)
   {
      mGPSDOStatusMsg = "Failed to read PPSDO_STATUS_STATE register with error: ";
      return status;
   }

   uint64_t accuracy = this->readRegister(GPSDODriver::reg_status_accuracy, &status);
   if(status != OpStatus::Success)
   {
      mGPSDOStatusMsg = "Failed to read PPSDO_STATUS_ACCURACY register with error: ";
      return status;
   }

   uint64_t tpulse = this->readRegister(GPSDODriver::reg_status_pps_active, &status);
   if(status != OpStatus::Success)
   {
      mGPSDOStatusMsg = "Failed to read PPSDO_STATUS_PPS_ACTIVE register with error: ";
      return status;
   }

   this->formatGPSDOStatus(state, accuracy, tpulse, GPSDOStatus);
   return status;   
}


bool GPSDODriver::getEnabled(OpStatus * status)
{
   uint64_t value = this->readRegister(GPSDODriver::reg_control, status);
   return static_cast<bool>(value & 1ULL);
}

OpStatus GPSDODriver::setEnabled(bool enable)
{
   OpStatus status = OpStatus::Success;
   uint64_t currRegValue = this->readRegister(GPSDODriver::reg_control, &status);
   if(status != OpStatus::Success)
      return status;
   
   currRegValue = setField(currRegValue, static_cast<uint64_t>(enable), CONTROL_EN_OFFSET, CONTROL_EN_SIZE);
   status = this->writeRegister(GPSDODriver::reg_control, currRegValue);

   return status;
}

string GPSDODriver::getDriverErr()
{ 
   return mCurrDriverErrMsg; 
}

void GPSDODriver::setDriverErr(string& msg)
{
   mCurrDriverErrMsg = msg;
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
   return (*mpGPSDORegisterList)[id];
}

void GPSDODriver::updateGPSDORegList(string& devName)
{
   mpGPSDORegisterList = &SDR_GPSDO_Registers[devName];
}

void GPSDODriver::formatGPSDOStatus(uint64_t state, uint64_t accuracy, uint64_t tpulse, array<string, 3>& GPSDOStatus)
{
   if(state == 1ULL)
      GPSDOStatus[GPSDO_STATE] = "Fine tune";
   else if(state == 0ULL)
      GPSDOStatus[GPSDO_STATE] = "Coarse tune";
   else
      GPSDOStatus[GPSDO_STATE] = "Unknown";
   
   if(accuracy < 4)
      GPSDOStatus[GPSDO_ACCURACY] = GPSDODriver::accuracyLevelList[accuracy];
   else
   {
      GPSDOStatus[GPSDO_ACCURACY] += "Unknown(";
      GPSDOStatus[GPSDO_ACCURACY] += std::to_string(accuracy);
      GPSDOStatus[GPSDO_ACCURACY] += ")";
   }

   GPSDOStatus[GPSDO_TPULSE] = tpulse ? "true" : "false";
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

static OpStatus runMonitoring(GPSDODriver * pDriver, int numDumps, std::chrono::milliseconds delay, int banner_interval)
{
   OpStatus status = OpStatus::Success;
   MonitorResults results;
   string formatedMsg;
   cout << "Monitoring GPSDO regulation loop (press Ctrl+C to stop):\n";
   int dumpCount = 0;
   bool continueLoop = false;
   int headerLength = 0;
   if(numDumps == 0)
      headerLength = printHeader();

   do
   {
      results.enableStatus = (pDriver->getEnabled(&status) ? "true" : "false");
      if(status != OpStatus::Success)
      {
         formatedMsg = formatedMsg + "Monitoring mode failed to read GPSDO enable status with error: " + ToString(status) + "\n";
         pDriver->setDriverErr(formatedMsg);
         return status;
      }

      results.error_1s = static_cast<int64_t>(pDriver->get_1s_error(&status));
      if(status != OpStatus::Success)
      {
         formatedMsg = formatedMsg + "Monitoring mode failed to read GPSDO 1s error value with error: " + ToString(status) + "\n";
         pDriver->setDriverErr(formatedMsg);
         return status;
      }

      results.error_10s = static_cast<int64_t>(pDriver->get_10s_error(&status));
      if(status != OpStatus::Success)
      {
         formatedMsg = formatedMsg + "Monitoring mode failed to read GPSDO 10s error value with error: " + ToString(status) + "\n";
         pDriver->setDriverErr(formatedMsg);
         return status;
      }

      results.error_100s = static_cast<int64_t>(pDriver->get_100s_error(&status));
      if(status != OpStatus::Success)
      {
         formatedMsg = formatedMsg + "Monitoring mode failed to read GPSDO 100s error value with error: " + ToString(status) + "\n";
         pDriver->setDriverErr(formatedMsg);
         return status;
      }

      results.dac = pDriver->getDacValue(&status);
      if(status != OpStatus::Success)
      {
         formatedMsg = formatedMsg + "Monitoring mode failed to read GPSDO DAC value with error: " + ToString(status) + "\n";
         pDriver->setDriverErr(formatedMsg);
         return status;
      }
      
      array<string, 3> GPSDOStatus;
      status = pDriver->getStatus(GPSDOStatus);
      if(status != OpStatus::Success)
      {
         formatedMsg = formatedMsg + "Monitoring mode failed to read GPSDO Status values. Reason: " + pDriver->getGPSDOStatusMsg() + ToString(status) + "\n";
         pDriver->setDriverErr(formatedMsg);
         return status;
      }
      results.gpsdoState = GPSDOStatus[GPSDO_STATE];
      results.gpsdoAccuracy = GPSDOStatus[GPSDO_ACCURACY];
      results.gpsdoTpulse = GPSDOStatus[GPSDO_TPULSE];

      if((dumpCount % banner_interval == 0 && dumpCount != numDumps))
         headerLength = printHeader();

      results.dumpCount = ++dumpCount;
      cout << formatToFit(&results, headerLength) << endl;
      
      std::this_thread::sleep_for(delay);
      continueLoop = (dumpCount < numDumps || numDumps == 0);

      // This is for infinite loop. To prevent UB, reset counter
      if(dumpCount == std::numeric_limits<int>::max())
         dumpCount = 0;

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
   string formatedMsg;
   cout << "Resetting GPSDO...\n";
   OpStatus status = pDriver->setEnabled(false);
   if(status != OpStatus::Success)
   {
      formatedMsg = formatedMsg + "Failed to reset GPSDO with error: " + ToString(status) + "\n";
      pDriver->setDriverErr(formatedMsg);
      return status;
   }
   
   std::this_thread::sleep_for(rstDelay);
   cout << "GPSDO reset complete (re-enabled)\n";
   return status;
}

static OpStatus enableGPSDO(GPSDODriver * pDriver, double clk, double ppm)
{
   OpStatus status = OpStatus::Success;
   string formatedMsg;
   double freq = clk * 1e6;

   uint64_t target_1s = static_cast<uint64_t>(freq);
   uint64_t target_10s = static_cast<uint64_t>(freq * 10);
   uint64_t target_100s = static_cast<uint64_t>(freq * 100);

   uint64_t tol_1s_hz = static_cast<uint64_t>(round(freq * ppm / 1e6));
   uint64_t tol_10s_hz = tol_1s_hz * 10;
   uint64_t tol_100s_hz = tol_1s_hz * 100;

   status = pDriver->writeRegister(GPSDODriver::reg_pps_1s_target, target_1s);
   if(status != OpStatus::Success)
   {
      formatedMsg = formatedMsg + "GPSDO enable failed to write PPSDO_CONFIG_ONE_S_TARGET register with error: " + ToString(status) + "\n";
      pDriver->setDriverErr(formatedMsg);
      return status;
   }
      
   status = pDriver->writeRegister(GPSDODriver::reg_pps_1s_err_tol, tol_1s_hz);
   if(status != OpStatus::Success)
   {
      formatedMsg = formatedMsg + "GPSDO enable failed to write PPSDO_CONFIG_ONE_S_TOL register with error: " + ToString(status) + "\n";
      pDriver->setDriverErr(formatedMsg);
      return status;
   }

   status = pDriver->writeRegister(GPSDODriver::reg_pps_10s_target, target_10s);
   if(status != OpStatus::Success)
   {
      formatedMsg = formatedMsg + "GPSDO enable failed to write PPSDO_CONFIG_TEN_S_TARGET register with error: " + ToString(status) + "\n";
      pDriver->setDriverErr(formatedMsg);
      return status;
   }

   status = pDriver->writeRegister(GPSDODriver::reg_pps_10s_err_tol, tol_10s_hz);
   if(status != OpStatus::Success)
   {
      formatedMsg = formatedMsg + "GPSDO enable failed to write PPSDO_CONFIG_TEN_S_TOL register with error: " + ToString(status) + "\n";
      pDriver->setDriverErr(formatedMsg);
      return status;
   }

   status = pDriver->writeRegister(GPSDODriver::reg_pps_100s_target, target_100s);
   if(status != OpStatus::Success)
   {
      formatedMsg = formatedMsg + "GPSDO enable failed to write PPSDO_CONFIG_HUNDRED_S_TARGET register with error: " + ToString(status) + "\n";
      pDriver->setDriverErr(formatedMsg);
      return status;
   }
   
   status = pDriver->writeRegister(GPSDODriver::reg_pps_100s_err_tol, tol_100s_hz);
   if(status != OpStatus::Success)
   {
      formatedMsg = formatedMsg + "GPSDO enable failed to write PPSDO_CONFIG_HUNDRED_S_TOL register with error: " + ToString(status) + "\n";
      pDriver->setDriverErr(formatedMsg);
      return status;
   }

   uint64_t control = 0;
   uint64_t clk_sel = 0; //(ceil(clk) == 10.0 || floor(clk) == 10.0 ) ? 1ULL : 0ULL;   // TODO: Update this later with the new revision of data sheet, for now defaulting to clk_sel 0
   control = setField(control, clk_sel, CONTROL_CLK_SEL_OFFSET, CONTROL_CLK_SEL_SIZE);
   control = setField(control, 1ULL, CONTROL_EN_OFFSET, CONTROL_EN_SIZE);

   status = pDriver->writeRegister(GPSDODriver::reg_control, control);
   if(status != OpStatus::Success)
   {
      formatedMsg = formatedMsg + "GPSDO enable failed to write PPSDO_ENABLE register with error: " + ToString(status) + "\n";
      pDriver->setDriverErr(formatedMsg);
      return status;
   }

   cout << "GPSDO enabled: CLK_SEL = " << clk_sel << " (" << clk << " MHz), " << ppm << " ppm tolerance (1s tol = " << tol_1s_hz << " Hz, 10s = "
        << tol_10s_hz << " Hz, 100s = " << tol_100s_hz << " Hz).\n";

   return status;
}

static OpStatus disableGPSDO(GPSDODriver * pDriver)
{
   string formatedMsg;
   OpStatus status = pDriver->writeRegister(GPSDODriver::reg_control, 0ULL);
   if(status != OpStatus::Success)
   {
      formatedMsg = formatedMsg + "GPSDO disable failed to write PPSDO_ENABLE register with error: " + ToString(status) + "\n";
      pDriver->setDriverErr(formatedMsg);
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

   // Note: Using commands as flags, because we need support for multi command invocation, which is not possible using args::Commands.
   // Note: Using group validator: AtLeastOne to throw incase no command is selected.
   args::Group                             commands(parser, "COMMANDS", args::Group::Validators::AtLeastOne); // NOLINT(cppcoreguidelines-slicing)
   args::Flag                              check(commands, "check", "Run monitoring mode", {"check"} );
   args::Flag                              dump(commands, "dump", "Dump registers", {"dump"});
   args::Flag                              reset(commands, "reset", "Reset GPSDO", {"reset"});
   args::Flag                              enable(commands, "enable", "Configure and enable GPSDO", {"enable"});
   args::Flag                              disable(commands, "disable", "Disable GPSDO", {"disable"});

   args::Group                             arguments(parser, "ARGUMENTS", args::Group::Validators::DontCare, args::Options::Global); // NOLINT(cppcoreguidelines-slicing)
   args::ValueFlag<std::string>            logFlag(arguments, "", "Log verbosity: info, warning, error, verbose, debug", {'l', "log"}, "error");
   args::ValueFlag<std::string>            deviceFlag(arguments, "name", "Specifies which device to use", {"device"}, "");
   args::ValueFlag<int>                    num(arguments, "iter", "Number of iterations (for --check: 0 for infinite; for --dump: default 1 if not specified)", {'n', "num"}, 0);
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
            throw args::UsageError("Error: Enable command requires flags --clk-freq and --ppm to be set for each LimeSDR device!");
      }

   } catch (args::Help&)
   {
      cout << parser << endl;
      return EXIT_SUCCESS;
   }
   catch (args::ValidationError& e)
   {
      std::cout << "Error: Select atleast one COMMAND from the list!" << endl;
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
      return EXIT_FAILURE;

   // In case device name is not specified, select the SDR name of the first device handle
   if(devName.empty())
      devName = handles.front().name;

   GPSDODriver driver(device->getICSR());
   GPSDODriver::updateGPSDORegList(devName);

   logVerbosity = strToLogLevel(args::get(logFlag));
   device->SetMessageLogCallback(lime::cli::LogCallback);
   lime::registerLogHandler(lime::cli::LogCallback);

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
         double clk = args::get(clk_freq);
         double m_ppm = args::get(ppm);
         if(m_ppm < 0.1)
            cout << "Warning: Selected " << m_ppm << " ppm may yield to incorrect error tolerance values! Recommended minimum ppm is 0.1!\n";

         runStatus = enableGPSDO(&driver, clk, m_ppm);
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

   if(runStatus != OpStatus::Success)
      cerr << driver.getDriverErr();
      
   driver.destroyCSR();
   DeviceRegistry::freeDevice(device);
   return EXIT_SUCCESS;
}