#ifndef __LIME_GPSDO__
#define __LIME_GPSDO__

#include "common.h"

#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/ToString.h"
#include "comms/ICSR.h"
#include "protocols/LMSBoards.h"

#include <cassert>
#include <cstring>
#include <filesystem>
#include <array>
#include <csignal>
#include <unordered_map>
#include <cmath>
#include <atomic>
#include <limits>
#include <cstdio>
#include <cinttypes>
#include "args.hxx"

//  Status bit fields.
#define STATUS_STATE_OFFSET 0
#define STATUS_STATE_SIZE 4
#define STATUS_ACCURACY_OFFSET 4
#define STATUS_ACCURACY_SIZE 4
#define STATUS_TPULSE_OFFSET 8
#define STATUS_TPULSE_SIZE 1

//  Control bit fields
#define CONTROL_EN_OFFSET 0
#define CONTROL_EN_SIZE 1
#define CONTROL_CLK_SEL_OFFSET 1
#define CONTROL_CLK_SEL_SIZE 1

enum class GPSDORegistersID;

using gpsdo_reg_list_t = std::unordered_map<GPSDORegistersID, uint64_t>;

class GPSDODriver
{
  public:
    GPSDODriver()
        : mCSR_interface(nullptr)
        , mpGPSDORegisterList(nullptr)
    {
    }
    GPSDODriver(lime::ICSR* interface)
        : mCSR_interface(interface)
        , mpGPSDORegisterList(nullptr)
    {
    }
    void destroyCSR();
    bool isCSRImplemented();

    // CSR interface access
    uint64_t readRegister(uint64_t address, lime::OpStatus* status);
    lime::OpStatus writeRegister(uint64_t address, uint64_t value);
    uint64_t getSigned32bit(uint64_t address, lime::OpStatus* status);

    // Register access
    uint64_t get_1s_error(lime::OpStatus* status);
    uint64_t get_10s_error(lime::OpStatus* status);
    uint64_t get_100s_error(lime::OpStatus* status);
    uint64_t getDacValue(lime::OpStatus* status);
    lime::OpStatus getStatus(std::string& prState, std::string& prAccuracy, std::string& prTpulse);
    bool getEnabled(lime::OpStatus* status);
    lime::OpStatus setEnabled(bool enable);

    // GPSDO Register address list manipulation
    uint64_t getGPSDORegAddress(GPSDORegistersID id);
    bool updateGPSDORegList(std::vector<lime::DeviceHandle>& handles, std::string& devName);

    // Logger members
    std::string getGPSDOStatusMsg();

  private:
    lime::ICSR* mCSR_interface;
    std::string mGPSDOStatusMsg;
    const gpsdo_reg_list_t* mpGPSDORegisterList;
};

struct MonitorResults {
    MonitorResults();

    uint32_t dumpCount;
    std::string enableStatus;
    int64_t error_1s;
    int64_t error_10s;
    int64_t error_100s;
    uint64_t dac;
    std::string gpsdoState;
    std::string gpsdoAccuracy;
    std::string gpsdoTpulse;
};

enum class GPSDORegistersID : int {
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

enum class MediaType : uint8_t { USB = 0, PCIE = 1, UNDEFINED = 2 };
#endif