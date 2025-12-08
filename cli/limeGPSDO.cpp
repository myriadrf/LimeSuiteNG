#include "common.h"

#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/ToString.h"
#include "comms/ICSR.h"

#include <cassert>
#include <cstring>
#include <filesystem>
#include <array>
#include <csignal>
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
   OpStatus getStatus(array<string,3>& GPSDOStatus);
   bool getEnabled(OpStatus * status);
   OpStatus setEnabled(bool enable);
   string getDriverErr();
   void setDriverErr(string& msg);

   private:
   uint64_t setField(uint64_t currRegValue, uint64_t newBitValue, int bitOffset, int size);
   uint64_t getField(uint64_t currRegValue, int bitOffset, int size);
   void formatGPSDOStatus(uint64_t state, uint64_t accuracy, uint64_t tpulse, array<string, 3>& GPSDOStatus);

   ICSR * mCSR_interface;
   std::string mGetStatusMsg;
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
      mGetStatusMsg = "Failed to read PPSDO_STATUS_STATE register ";
      return status;
   }

   uint64_t accuracy = this->readRegister(GPSDODriver::reg_status_accuracy, &status);
   if(status != OpStatus::Success)
   {
      mGetStatusMsg = "Failed to read PPSDO_STATUS_ACCURACY register ";
      return status;
   }

   uint64_t tpulse = this->readRegister(GPSDODriver::reg_status_pps_active, &status);
   if(status != OpStatus::Success)
   {
      mGetStatusMsg = "Failed to read PPSDO_STATUS_PPS_ACTIVE register ";
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
   
   currRegValue = this->setField(currRegValue, static_cast<uint64_t>(enable), CONTROL_EN_OFFSET, CONTROL_EN_SIZE);
   status = this->writeRegister(GPSDODriver::reg_control, currRegValue);

   return status;
}

string GPSDODriver::getDriverErr()
{ 
   auto tmp = mCurrDriverErrMsg;
   mCurrDriverErrMsg.clear();
   return tmp; 
}

void GPSDODriver::setDriverErr(string& msg)
{
   mCurrDriverErrMsg = msg;
}

uint64_t GPSDODriver::setField(uint64_t currRegValue, uint64_t newBitValue, int bitOffset, int size)
{
   uint64_t mask = ((1ULL << size) - 1ULL) << bitOffset;
   return ((currRegValue & ~mask) | ((newBitValue << bitOffset) & mask));
}

uint64_t GPSDODriver::getField(uint64_t currRegValue, int bitOffset, int size)
{
   uint64_t mask = ((1ULL << size) - 1ULL) << bitOffset;
   return ((currRegValue & mask) >> bitOffset);
}

void GPSDODriver::formatGPSDOStatus(uint64_t state, uint64_t accuracy, uint64_t tpulse, array<string, 3>& GPSDOStatus)
{
   if(state == 1ULL)
      GPSDOStatus[0] = "Fine tune";
   else if(state == 0ULL)
      GPSDOStatus[0] = "Coarse tune";
   else
      GPSDOStatus[0] = "Unknown";
   
   if(accuracy < 4)
      GPSDOStatus[1] = GPSDODriver::accuracyLevelList[accuracy];
   else
   {
      GPSDOStatus[1] += "Unknown(";
      GPSDOStatus[1] += std::to_string(accuracy);
      GPSDOStatus[1] += ")";
   }

   GPSDOStatus[2] = tpulse ? "true" : "false";
}

// ###############################################
// #### limeGPSDO helper function definitions ####
// ###############################################

void runMonitoring(GPSDODriver * pDriver)
{

}

void dumpRegisters(GPSDODriver * pDriver, int numberOfDumps = 1, int delay = 1)
{

}

void resetGPSDO(GPSDODriver * pDriver, int rstDelay = 2)
{

}

void enableGPSDO(GPSDODriver * pDriver, double clk = 30.72, double ppm = 0.1)
{

}

void disableGPSDO(GPSDODriver * pDriver)
{

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
         enableGPSDO(&driver);
      else if(disable)
         disableGPSDO(&driver);
      else if(reset)
         resetGPSDO(&driver);
      else if(check)
         runMonitoring(&driver);
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