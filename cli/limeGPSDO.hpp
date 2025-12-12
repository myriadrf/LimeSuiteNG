#ifndef __LIME_GPSDO__
#define __LIME_GPSDO__

#include "common.h"

#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/ToString.h"
#include "comms/ICSR.h"

#include <cassert>
#include <cstring>
#include <filesystem>
#include <array>
#include <csignal>
#include <unordered_map>
#include <cmath>
#include <atomic>
#include <limits>
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

enum class GPSDORegistersID;

using gpsdo_reg_list_t = unordered_map<GPSDORegistersID, uint64_t>;

static uint64_t setField(uint64_t currRegValue, uint64_t newBitValue, int bitOffset, int size);
static uint64_t getField(uint64_t currRegValue, int bitOffset, int size);
atomic<bool> cleanUp(false);

void keyBoardInt(int param)
{
   cleanUp = true;
}

class GPSDODriver
{
   public:

   const array<string,4> accuracyLevelList = {"Disabled/Lowest", "1s Tune", "2s Tune", "3s Tune (Highest)"}; 
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
   void destroyCSR();

   // CSR interface access
   uint64_t readRegister(uint64_t address, OpStatus * status);
   OpStatus writeRegister(uint64_t address, uint64_t value);
   uint64_t getSigned32bit(uint64_t address, OpStatus * status);

   // Register access
   uint64_t get_1s_error(OpStatus * status);
   uint64_t get_10s_error(OpStatus * status);
   uint64_t get_100s_error(OpStatus * status);
   uint64_t getDacValue(OpStatus * status);
   OpStatus getStatus(array<string, 3>& GPSDOStatus);
   bool getEnabled(OpStatus * status);
   OpStatus setEnabled(bool enable);

   // GPSDO Register address list manipulation
   static uint64_t getGPSDORegAddress(GPSDORegistersID id);
   static void updateGPSDORegList(string& devName);

   // Logger members
   string getDriverErr();
   void setDriverErr(string& msg);
   string getGPSDOStatusMsg();

   private:
   void formatGPSDOStatus(uint64_t state, uint64_t accuracy, uint64_t tpulse, array<string, 3>& GPSDOStatus);

   ICSR * mCSR_interface;
   std::string mGPSDOStatusMsg;
   std::string mCurrDriverErrMsg;
   static gpsdo_reg_list_t * mpGPSDORegisterList;
};

gpsdo_reg_list_t * GPSDODriver::mpGPSDORegisterList = nullptr;

enum class GPSDORegistersID : int
{
   PPSDO_ENABLE = 0,
   PPSDO_CONFIG_ONE_S_TARGET = 1,
   PPSDO_CONFIG_ONE_S_TOL = 2,
   PPSDO_CONFIG_TEN_S_TARGET = 3,
   PPSDO_CONFIG_TEN_S_TOL = 4,
   PPSDO_CONFIG_HUNDRED_S_TARGET = 5,
   PPSDO_CONFIG_HUNDRED_S_TOL = 6,
   PPSDO_STATUS_ONE_S_ERROR = 7,
   PPSDO_STATUS_TEN_S_ERROR = 8,
   PPSDO_STATUS_HUNDRED_S_ERROR = 9,
   PPSDO_STATUS_DAC_TUNED_VAL = 10,
   PPSDO_STATUS_ACCURACY = 11,
   PPSDO_STATUS_PPS_ACTIVE = 12,
   PPSDO_STATUS_STATE = 13
};

static unordered_map<string, gpsdo_reg_list_t> SDR_GPSDO_Registers = 
{
   {"LimeSDR XTRX", 
      {
         {GPSDORegistersID::PPSDO_ENABLE,                   0x00000000F000B000},
         {GPSDORegistersID::PPSDO_CONFIG_ONE_S_TARGET,      0x00000000F000B004},
         {GPSDORegistersID::PPSDO_CONFIG_ONE_S_TOL,         0x00000000F000B008},
         {GPSDORegistersID::PPSDO_CONFIG_TEN_S_TARGET,      0x00000000F000B00C},
         {GPSDORegistersID::PPSDO_CONFIG_TEN_S_TOL,         0x00000000F000B010},
         {GPSDORegistersID::PPSDO_CONFIG_HUNDRED_S_TARGET,  0x00000000F000B014},
         {GPSDORegistersID::PPSDO_CONFIG_HUNDRED_S_TOL,     0x00000000F000B018},
         {GPSDORegistersID::PPSDO_STATUS_ONE_S_ERROR,       0x00000000F000B01C},
         {GPSDORegistersID::PPSDO_STATUS_TEN_S_ERROR,       0x00000000F000B020},
         {GPSDORegistersID::PPSDO_STATUS_HUNDRED_S_ERROR,   0x00000000F000B024},
         {GPSDORegistersID::PPSDO_STATUS_DAC_TUNED_VAL,     0x00000000F000B028},
         {GPSDORegistersID::PPSDO_STATUS_ACCURACY,          0x00000000F000B02C},
         {GPSDORegistersID::PPSDO_STATUS_PPS_ACTIVE,        0x00000000F000B030},
         {GPSDORegistersID::PPSDO_STATUS_STATE,             0x00000000F000B034},
      }
   },
   {"LimeSDR Mini V2", 
      {
         {GPSDORegistersID::PPSDO_ENABLE,                   0x00000000F0002800},
         {GPSDORegistersID::PPSDO_CONFIG_ONE_S_TARGET,      0x00000000F0002804},
         {GPSDORegistersID::PPSDO_CONFIG_ONE_S_TOL,         0x00000000F0002808},
         {GPSDORegistersID::PPSDO_CONFIG_TEN_S_TARGET,      0x00000000F000280C},
         {GPSDORegistersID::PPSDO_CONFIG_TEN_S_TOL,         0x00000000F0002810},
         {GPSDORegistersID::PPSDO_CONFIG_HUNDRED_S_TARGET,  0x00000000F0002814},
         {GPSDORegistersID::PPSDO_CONFIG_HUNDRED_S_TOL,     0x00000000F0002818},
         {GPSDORegistersID::PPSDO_STATUS_ONE_S_ERROR,       0x00000000F000281C},
         {GPSDORegistersID::PPSDO_STATUS_TEN_S_ERROR,       0x00000000F0002820},
         {GPSDORegistersID::PPSDO_STATUS_HUNDRED_S_ERROR,   0x00000000F0002824},
         {GPSDORegistersID::PPSDO_STATUS_DAC_TUNED_VAL,     0x00000000F0002828},
         {GPSDORegistersID::PPSDO_STATUS_ACCURACY,          0x00000000F000282C},
         {GPSDORegistersID::PPSDO_STATUS_PPS_ACTIVE,        0x00000000F0002830},
         {GPSDORegistersID::PPSDO_STATUS_STATE,             0x00000000F0002834},
      }
   }
};

#endif