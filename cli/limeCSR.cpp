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


static void PrintCSRegReadResults(std::ostream& stream, const std::vector<uint64_t>& rd_data, const std::vector<uint64_t>& wr_data)
{
   stream << std::hex << std::setfill('0');
   for(int i = 0; wr_data.size(); ++i)
      stream << std::setw(16) << wr_data[i] << std::setw(16) << rd_data[i] << std::endl;
}

static uint64_t hex2ULLint(const std::string_view hexstr)
{
   uint64_t value = 0;
   sscanf(hexstr.data(), "%016llX", &value);
   return value;
}

static int parseWriteInput(std::string_view hexstr, std::vector<uint64_t>& wr_data)
{
   static const std::string_view delimiters = " \n,"sv;
   wr_data.clear();
   int tokenCount = 0;

   std::size_t position = 0;
   while (position != std::string_view::npos)
   {
      position = hexstr.find_first_of(delimiters);
      std::string_view token = hexstr.substr(0, position);
      int tokenLength = token.size();
      if (tokenLength <= 32 && tokenLength > 16) // write instruction
      {
         uint64_t addr = hex2ULLint(token.substr(0,16));
         uint64_t value = hex2ULLint(token.substr(16,32));
         wr_data.push_back(addr);
         wr_data.push_back(value);
      }
      else if (tokenLength != 0)
      {
         std::cerr << "Invalid input value: "sv << token << std::endl;
      }
      ++tokenCount;
      hexstr = hexstr.substr(position + 1);
   }
   return tokenCount;
}

static int parseReadInput(std::string_view hexstr, std::vector<uint64_t>& wr_data)
{
   static const std::string_view delimiters = " \n,"sv;
   wr_data.clear();
   int tokenCount = 0;

   std::size_t position = 0;
   while (position != std::string_view::npos)
   {
      position = hexstr.find_first_of(delimiters);
      std::string_view token = hexstr.substr(0, position);
      int tokenLength = token.size();
      if (tokenLength <= 16 && tokenLength > 0) // read instruction
      {
         uint64_t addr = hex2ULLint(token);
         wr_data.push_back(addr);
      }
      else if (tokenLength != 0)
      {
         std::cerr << "Invalid input value: "sv << token << std::endl;
      }
      ++tokenCount;
      hexstr = hexstr.substr(position + 1);
   }
   return tokenCount;
}

static std::string ReadFile(const std::string& fileName)
{
   std::vector<char> buffer;
   std::ifstream inputFile(fileName);
   if (!inputFile.is_open())
   {
      cerr << "Failed to open file: "sv << fileName << endl;
      exit(EXIT_FAILURE);
   }
   inputFile.seekg(0, std::ios::end);
   long fileSize = inputFile.tellg();
   inputFile.seekg(0, std::ios::beg);

   buffer.resize(fileSize);
   inputFile.read(&buffer[0], fileSize);
   inputFile.close();
   buffer[fileSize] = 0;
   return buffer.data();
}

int main(int argc, char** argv)
{
   // clang-format off
   args::ArgumentParser                    parser("limeCSR - Configuration Space Registers I/O", "");
   args::HelpFlag                          help(parser, "help", "This help", {'h', "help"});

   args::Group                             commands(parser, "commands"); // NOLINT(cppcoreguidelines-slicing) 
   args::Command                           read(commands, "read", "Reading operation");
   args::Command                           write(commands, "write", "Do writing operation");

   args::Group                             arguments(parser, "arguments", args::Group::Validators::DontCare, args::Options::Global); // NOLINT(cppcoreguidelines-slicing)
   args::ValueFlag<std::string>            deviceFlag(arguments, "name", "Specifies which device to use", {'d', "device"}, "");
   args::Group                             writeGroup(arguments, "Data options");
   args::ValueFlag<std::string>            fileFlag(arguments, "file", "File", {'f', "file"});
   args::ValueFlag<std::string>            streamFlag(arguments, "stream", "Stream", {'s', "stream"});
   // clang-format on

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

   if (fileFlag == streamFlag)
   {
      cerr << "Either -s or -f must be provided ONCE" << endl;
      return EXIT_FAILURE;
   }

   const std::string devName = args::get(deviceFlag);
   const std::string hexInput = streamFlag ? args::get(streamFlag) : ReadFile(args::get(fileFlag));

   if (hexInput.empty())
   {
      cerr << "No input provided"sv << endl;
      return EXIT_FAILURE;
   }

   auto handles = DeviceRegistry::enumerate();
   if (handles.size() == 0)
   {
      cerr << "No devices found"sv << endl;
      return EXIT_FAILURE;
   }

   SDRDevice* device = ConnectToFilteredOrDefaultDevice(devName);
   if (!device)
      return EXIT_FAILURE;

   std::vector<uint64_t> wr_data;
   std::vector<uint64_t> rd_data;
   
   if(write)
      parseWriteInput(hexInput, wr_data);
   else if(read)
   {
      parseReadInput(hexInput, wr_data);
      rd_data.resize(wr_data.size());
   }
   
   ICSR * CSR_interface = device->getICSR();

   try
   {   
      if(write)
      {
         for(int i = 0; i < wr_data.size(); i+=2)
         {
            OpStatus status = CSR_interface->ioWrite64(wr_data[i], wr_data[i+1]);
            if(status != OpStatus::Success)
               cerr << "CSR write failed for register address 0x" << hex << wr_data[i] << dec << " with error: " << ToString(status) << endl;
         }
      }
      else if(read)
      {
         OpStatus status = OpStatus::Success;
         for(int i = 0; i < wr_data.size(); ++i)
         {
            rd_data[i] = CSR_interface->ioRead64(wr_data[i], &status);
            if(status != OpStatus::Success)
               cerr << "CSR read failed for register address 0x" << hex << wr_data[i] << dec << " with error: " << ToString(status) << endl;
         }
      }
      

   } catch (std::runtime_error& e)
   {
      delete CSR_interface;
      DeviceRegistry::freeDevice(device);
      cerr << "CSR failed: "sv << e.what() << endl;
      return EXIT_FAILURE;
   }

   if (read)
      PrintCSRegReadResults(cout, rd_data, wr_data);

   delete CSR_interface;
   DeviceRegistry::freeDevice(device);
   return EXIT_SUCCESS;
}
