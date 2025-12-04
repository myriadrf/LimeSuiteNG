#include "common.h"

#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/ToString.h"
#include "comms/ICSR.h"

#include <cassert>
#include <cstring>
#include <filesystem>
#include "args.hxx"

using namespace std;
using namespace lime;
using namespace lime::cli;


class GPSDODriver
{
   public:
   GPSDODriver() CSR_interface(nullptr) : {}
   GPSDODriver(ICSR * interface) CSR_interface(interface) : {}
   ~GPSDODriver() { if (CSR_interface != nullptr) delete CSR_interface;}

   uint64_t readRegister();
   void writeRegister();
   int32_t getSigned32bit();
   int get_1s_error();
   int get_10s_error();
   int get_100s_error();
   int getDacValue();
   int getStatus();
   int getEnabled();
   int setEnabled();

   private:
   ICSR * CSR_interface;
};

void runMonitoring()
{

}

void dumpRegisters()
{

}

void resetGPSDO()
{

}

void enableGPSDO()
{

}

void disableGPSDO()
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


}