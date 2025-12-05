#include "common.h"

#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/ToString.h"
#include "comms/ICSR.h"

#include <cassert>
#include <cstring>
#include <filesystem>
#include <array>
#include "args.hxx"

using namespace std;
using namespace lime;
using namespace lime::cli;


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
   std::string getStatus();
   uint64_t getEnabled(OpStatus * status);
   uint64_t setEnabled(OpStatus * status);

   private:
   ICSR * mCSR_interface;
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
      return 0LL;
   
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

std::string GPSDODriver::getStatus()
{
   OpStatus status = OpStatus::Success;
   std::string statusMsg;
   uint64_t state = this->readRegister(GPSDODriver::reg_status_state, &status);
   if(status != OpStatus::Success)
   {
      statusMsg += "Failed to read PPSDO_STATUS_STATE register with error:";
      statusMsg += ToString(status);
      statusMsg += "\n";
      return statusMsg;
   }

   uint64_t accuracy = this->readRegister(GPSDODriver::reg_status_accuracy, &status);
   if(status != OpStatus::Success)
   {
      statusMsg += "Failed to read PPSDO_STATUS_ACCURACY register with error:";
      statusMsg += ToString(status);
      statusMsg += "\n";
      return statusMsg;
   }

   uint64_t tpulse = this->readRegister(GPSDODriver::reg_status_pps_active, &status);
   if(status != OpStatus::Success)
   {
      statusMsg += "Failed to read PPSDO_STATUS_PPS_ACTIVE register with error:";
      statusMsg += ToString(status);
      statusMsg += "\n";
      return statusMsg;
   }

   // TODO: Move this to formating function
   statusMsg += "state: ";
   if(state == 1ULL)
      statusMsg += "Fine tune\n";
   else if(state == 0ULL)
      statusMsg += "Coarse tune\n";
   else
      statusMsg += "Unknown\n";
   
   statusMsg += "accuracy: ";
   if(accuracy < 4)
      statusMsg += GPSDODriver::accuracyLevelList[accuracy];
   else
   {
      statusMsg += "Unknown(";
      statusMsg += std::to_string(accuracy);
      statusMsg +=")\n";
   }

   statusMsg += "tpulse_active: ";
   statusMsg += tpulse ? "true\n" : "false\n";
   
   return statusMsg;   
}

uint64_t GPSDODriver::getEnabled(OpStatus * status)
{
   uint64_t value;
   return value;
}

uint64_t GPSDODriver::setEnabled(OpStatus * status)
{
   uint64_t value;
   return value;
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
      
   driver.~GPSDODriver();
   DeviceRegistry::freeDevice(device);
   return EXIT_SUCCESS;
}