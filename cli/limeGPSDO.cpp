#include "common.h"

#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/ToString.h"
#include "comms/ICSR.h"

#include <cassert>
#include <cstring>
#include <filesystem>
#include <array>
#include <csignal>
#include <cmath>
#include "args.hxx"

using namespace std;
using namespace lime;
using namespace lime::cli;

//  Status bit fields.
#define STATUS_STATE_OFFSET    0
#define STATUS_STATE_SIZE      4
#define STATUS_ACCURACY_OFFSET 4
#define STATUS_ACCURACY_SIZE   4
#define STATUS_TPULSE_OFFSET   8
#define STATUS_TPULSE_SIZE     1

//  Control bit fields
#define CONTROL_EN_OFFSET      0
#define CONTROL_EN_SIZE        1
#define CONTROL_CLK_SEL_OFFSET 1
#define CONTROL_CLK_SEL_SIZE   1

// GPSDOStatus array IDs
#define GPSDO_STATE    0
#define GPSDO_ACCURACY 1
#define GPSDO_TPULSE   2

static uint64_t setField(uint64_t currRegValue, uint64_t newBitValue, int bitOffset, int size);
static uint64_t getField(uint64_t currRegValue, int bitOffset, int size);

class GPSDODriver
{
   public:

   const array<string,4> accuracyLevelList = {"Disabled/Lowest\n", "1s Tune\n", "2s Tune\n", "3s Tune (Highest)\n"}; 
   static constexpr uint64_t reg_control            = 0x00000000F000B000;
   static constexpr uint64_t reg_pps_1s_target      = 0x00000000F000B004;
   static constexpr uint64_t reg_pps_1s_err_tol     = 0x00000000F000B008;
   static constexpr uint64_t reg_pps_10s_target     = 0x00000000F000B00C;
   static constexpr uint64_t reg_pps_10s_err_tol    = 0x00000000F000B010;
   static constexpr uint64_t reg_pps_100s_target    = 0x00000000F000B014;
   static constexpr uint64_t reg_pps_100s_err_tol   = 0x00000000F000B018;

   static constexpr uint64_t reg_pps_1s_err         = 0x00000000F000B01C;
   static constexpr uint64_t reg_pps_10s_err        = 0x00000000F000B020;
   static constexpr uint64_t reg_pps_100s_err       = 0x00000000F000B024;
   static constexpr uint64_t reg_dac_tuned_val      = 0x00000000F000B028;
   static constexpr uint64_t reg_status_accuracy    = 0x00000000F000B02C;
   static constexpr uint64_t reg_status_pps_active  = 0x00000000F000B030;
   static constexpr uint64_t reg_status_state       = 0x00000000F000B034;

   GPSDODriver() : mCSR_interface(nullptr) {}
   GPSDODriver(ICSR * interface) : mCSR_interface(interface) {}
   ~GPSDODriver() { if (mCSR_interface != nullptr) delete mCSR_interface;}

   uint64_t readRegister(uint64_t address, OpStatus * status);
   OpStatus writeRegister(uint64_t address, uint64_t value);
   uint64_t getSigned32bit(uint64_t address, OpStatus * status);

   uint64_t get_1s_error(OpStatus * status);
   uint64_t get_10s_error(OpStatus * status);
   uint64_t get_100s_error(OpStatus * status);
   uint64_t getDacValue(OpStatus * status);
   OpStatus getStatus(array<string, 3>& GPSDOStatus);
   bool getEnabled(OpStatus * status);
   OpStatus setEnabled(bool enable);

   // Logger members
   string getDriverErr();
   void setDriverErr(string& msg);
   string getGPSDOStatusMsg();

   private:
   void formatGPSDOStatus(uint64_t state, uint64_t accuracy, uint64_t tpulse, array<string, 3>& GPSDOStatus);

   ICSR * mCSR_interface;
   std::string mGPSDOStatusMsg;
   std::string mCurrDriverErrMsg;
};

// ##################################################
// #### GPSDODriver member functions definitions ####
// ##################################################

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
   
   if(value & (1 << 31))
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

static OpStatus runMonitoring(GPSDODriver * pDriver, int numDumps, std::chrono::milliseconds delay, int banner_interval)
{
   OpStatus status = OpStatus::Success;
   string formatedMsg;
   const string header = "Dump | Enabled | 1s Error | 10s Error | 100s Error | DAC Value | State | Accuracy | TPulse";
   cout << "Monitoring GPSDO regulation loop (press Ctrl+C to stop):\n";
   cout << header << endl;
   int dumpCount = 0;
   do
   {
      bool enableStatus = pDriver->getEnabled(&status);
      if(status != OpStatus::Success)
      {
         formatedMsg = formatedMsg + "Monitoring mode failed to read GPSDO enable status with error: " + ToString(status) + "\n";
         pDriver->setDriverErr(formatedMsg);
         return status;
      }

      uint64_t error_1s = pDriver->get_1s_error(&status);
      if(status != OpStatus::Success)
      {
         formatedMsg = formatedMsg + "Monitoring mode failed to read GPSDO 1s error value with error: " + ToString(status) + "\n";
         pDriver->setDriverErr(formatedMsg);
         return status;
      }

      uint64_t error_10s = pDriver->get_10s_error(&status);
      if(status != OpStatus::Success)
      {
         formatedMsg = formatedMsg + "Monitoring mode failed to read GPSDO 10s error value with error: " + ToString(status) + "\n";
         pDriver->setDriverErr(formatedMsg);
         return status;
      }

      uint64_t error_100s = pDriver->get_100s_error(&status);
      if(status != OpStatus::Success)
      {
         formatedMsg = formatedMsg + "Monitoring mode failed to read GPSDO 100s error value with error: " + ToString(status) + "\n";
         pDriver->setDriverErr(formatedMsg);
         return status;
      }

      uint64_t dac = pDriver->getDacValue(&status);
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

      cout << dumpCount << (enableStatus ? "true"s : "false"s) << error_1s << error_10s << error_100s << dac << GPSDOStatus[GPSDO_STATE] << GPSDOStatus[GPSDO_ACCURACY] << GPSDOStatus[GPSDO_TPULSE] << endl;

      ++dumpCount;
      if(dumpCount % banner_interval == 0)
         cout << header << endl;
      
      std::this_thread::sleep_for(delay);

   } while (dumpCount < numDumps);

   // TODO: Add Ctrl+C signal handler

   cout << "Monitoring finished\n";
   return status;
}

static void dumpRegisters(GPSDODriver * pDriver, int numberOfDumps = 1, int delay = 1)
{
   cout << "Note: Raw register dump not available via named CSRs — use --check for full status\n";
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

static OpStatus enableGPSDO(GPSDODriver * pDriver, double clk = 30.72, double ppm = 0.1)
{
   OpStatus status = OpStatus::Success;
   string formatedMsg;
   double freq = clk * 1e6;
   cout << "freq = " << freq << endl;

   uint64_t target_1s = static_cast<uint64_t>(freq);
   cout << "target_1s = " << target_1s << endl;
   uint64_t target_10s = static_cast<uint64_t>(freq * 10);
   cout << "target_10s = " << target_10s << endl;
   uint64_t target_100s = static_cast<uint64_t>(freq * 100);
   cout << "target_100s = " << target_100s << endl;

   uint64_t tol_1s_hz = static_cast<uint64_t>(round(freq * ppm / 1e6));
   cout << "tol_1s_hz = " << tol_1s_hz << endl;
   uint64_t tol_10s_hz = tol_1s_hz * 10;
   cout << "tol_10s_hz = " << tol_10s_hz << endl;
   uint64_t tol_100s_hz = tol_1s_hz * 100;
   cout << "tol_100s_hz = " << tol_100s_hz << endl;

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
   uint64_t clk_sel = (ceil(clk) == 10.0f || floor(clk) == 10.0f ) ? 1ULL : 0ULL;
   control = setField(control, clk_sel, CONTROL_CLK_SEL_OFFSET, CONTROL_CLK_SEL_SIZE);
   control = setField(control, 1ULL, CONTROL_EN_OFFSET, CONTROL_EN_SIZE);

   status = pDriver->writeRegister(GPSDODriver::reg_control, control);
   if(status != OpStatus::Success)
   {
      formatedMsg = formatedMsg + "GPSDO enable failed to write PPSDO_ENABLE register with error: " + ToString(status) + "\n";
      pDriver->setDriverErr(formatedMsg);
      return status;
   }

   cout << "GPSDO enabled: CLK_SEL = " << clk_sel << "(" << clk << "MHz), " << ppm << "ppm tolerance (1s tol = " << tol_1s_hz << " Hz, 10s = "
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
   // clang-format off
   args::ArgumentParser                    parser("limeGPSDO - \"Insert description here\"", "");
   args::HelpFlag                          help(parser, "help", "This help", {'h', "help"});

   args::Group                             commands(parser, "commands"); // NOLINT(cppcoreguidelines-slicing) 
   args::Command                           check(commands, "check", "Run monitoring mode");
   args::Command                           dump(commands, "dump", "Dump registers");
   args::Command                           reset(commands, "reset", "Reset GPSDO");
   args::Command                           enable(commands, "enable", "Configure and enable GPSDO");
   args::Command                           disable(commands, "disable", "Disable GPSDO");


   args::Group                             arguments(parser, "arguments", args::Group::Validators::DontCare, args::Options::Global); // NOLINT(cppcoreguidelines-slicing)
   args::ValueFlag<std::string>            deviceFlag(arguments, "name", "Specifies which device to use", {"device"}, "");
   args::ValueFlag<std::string>            num(arguments, "iter", "Number of iterations (for --check: 0 for infinite; for --dump: default 1 if not specified)", {'n', "num"}, "");
   args::ValueFlag<std::string>            delay(arguments, "time", "Delay between iterations (seconds, for --check and --dump)", {'d', "delay"}, "");
   args::ValueFlag<std::string>            banner(arguments, "interval", "Banner repeat interval (for --check)", {'b', "banner"}, "");
   args::ValueFlag<std::string>            reset_delay(arguments, "time", "Delay after disable before re-enable (seconds, for --reset)", {'r', "reset-delay"}, "");
   args::ValueFlag<std::string>            clk_freq(arguments, "Mhz", "Clock frequency in MHz (10 or 30.72)", {'c', "clk_freq"}, "");
   args::ValueFlag<std::string>            ppm(arguments, "ppm", "Tolerance in ppm", {'p', "ppm"}, "");
   // // clang-format on

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

   auto handles = DeviceRegistry::enumerate();
   if (handles.size() == 0)
   {
      cerr << "No devices found"sv << endl;
      return EXIT_FAILURE;
   }

   SDRDevice* device = ConnectToFilteredOrDefaultDevice(devName);
   if (!device)
      return EXIT_FAILURE;

   GPSDODriver driver(device->getICSR());

   OpStatus runStatus = OpStatus::Success;
   try
   {
      if(dump)
         dumpRegisters(&driver);
      else if(enable)
        runStatus = enableGPSDO(&driver);
      else if(disable)
        runStatus = disableGPSDO(&driver);
      else if(reset)
        runStatus = resetGPSDO(&driver, std::chrono::milliseconds(1000));
      else if(check)
        runStatus = runMonitoring(&driver, 0, std::chrono::milliseconds(1000), 1);     // TODO: Pass the actual arguments
      else
      {
         cerr << "No command specified! Aborting limeGPSDO execution!";
         driver.~GPSDODriver();
         DeviceRegistry::freeDevice(device);
         return EXIT_FAILURE;
      }

   }
   catch(const std::exception& e)
   {
      driver.~GPSDODriver();
      DeviceRegistry::freeDevice(device);
      std::cerr << e.what() << '\n';
   }

   if(runStatus != OpStatus::Success)
      cerr << driver.getDriverErr();
      
   driver.~GPSDODriver();
   DeviceRegistry::freeDevice(device);
   return EXIT_SUCCESS;
}