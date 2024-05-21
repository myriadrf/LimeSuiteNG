/**
@file LMS7002M.cpp
@author Lime Microsystems (www.limemicro.com)
@brief Implementation of LMS7002M transceiver configuring
*/

#define _USE_MATH_DEFINES
#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/embedded/lms7002m/lms7002m.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <ciso646>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string_view>
#include <unordered_set>
#include <thread>

#ifdef __GNUC__
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wold-style-cast"
#endif
#include "cpp-feather-ini-parser/INI.h"
#ifdef __GNUC__
    #pragma GCC diagnostic pop
#endif

#include "DSP/GFIR/lms_gfir.h"
#include "limesuiteng/types.h"
#include "comms/IComms.h"
#include "LMS7002M_RegistersMap.h"
#include "LMS7002MCSR_Data.h"
#include "limesuiteng/LMS7002MCSR.h"
#include "limesuiteng/Logger.h"
#include "mcu_programs.h"
#include "MCU_BD.h"
#include "utilities/toString.h"

using namespace lime;
using namespace LMS7002MCSR_Data;
using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

constexpr std::array<std::array<float_type, 2>, 3> LMS7002M::gVCO_frequency_table{
    { { 3800e6, 5222e6 }, { 4961e6, 6754e6 }, { 6306e6, 7714e6 } }
};

constexpr LMS7002M::Channel IntToChannel(int channel)
{
    return channel > 0 ? LMS7002M::Channel::ChB : LMS7002M::Channel::ChA;
}

// Module addresses needs to be sorted in ascending order
const std::vector<LMS7002M::ReadOnlyRegister> LMS7002M::readOnlyRegisters{
    { 0x002F, 0x0000 },
    { 0x008C, 0x0FFF },
    { 0x00A8, 0x007F },
    { 0x00A9, 0x0000 },
    { 0x00AA, 0x0000 },
    { 0x00AB, 0x0000 },
    { 0x00AC, 0x0000 },
    { 0x0123, 0x003F },
    { 0x0209, 0x0000 },
    { 0x020A, 0x0000 },
    { 0x020B, 0x0000 },
    { 0x040E, 0x0000 },
    { 0x040F, 0x0000 },
};

const std::map<LMS7002M::MemorySection, std::array<uint16_t, 2>> LMS7002M::MemorySectionAddresses{
    { LMS7002M::MemorySection::LimeLight, { 0x0020, 0x002F } },
    { LMS7002M::MemorySection::EN_DIR, { 0x0081, 0x0081 } },
    { LMS7002M::MemorySection::AFE, { 0x0082, 0x0082 } },
    { LMS7002M::MemorySection::BIAS, { 0x0084, 0x0084 } },
    { LMS7002M::MemorySection::XBUF, { 0x0085, 0x0085 } },
    { LMS7002M::MemorySection::CGEN, { 0x0086, 0x008C } },
    { LMS7002M::MemorySection::LDO, { 0x0092, 0x00A7 } },
    { LMS7002M::MemorySection::BIST, { 0x00A8, 0x00AC } },
    { LMS7002M::MemorySection::CDS, { 0x00AD, 0x00AE } },
    { LMS7002M::MemorySection::TRF, { 0x0100, 0x0104 } },
    { LMS7002M::MemorySection::TBB, { 0x0105, 0x010B } },
    { LMS7002M::MemorySection::RFE, { 0x010C, 0x0114 } },
    { LMS7002M::MemorySection::RBB, { 0x0115, 0x011A } },
    { LMS7002M::MemorySection::SX, { 0x011C, 0x0124 } },
    { LMS7002M::MemorySection::TRX_GAIN, { 0x0125, 0x0126 } },
    { LMS7002M::MemorySection::TxTSP, { 0x0200, 0x020C } },
    { LMS7002M::MemorySection::TxNCO, { 0x0240, 0x0261 } },
    { LMS7002M::MemorySection::TxGFIR1, { 0x0280, 0x02A7 } },
    { LMS7002M::MemorySection::TxGFIR2, { 0x02C0, 0x02E7 } },
    { LMS7002M::MemorySection::TxGFIR3a, { 0x0300, 0x0327 } },
    { LMS7002M::MemorySection::TxGFIR3b, { 0x0340, 0x0367 } },
    { LMS7002M::MemorySection::TxGFIR3c, { 0x0380, 0x03A7 } },
    { LMS7002M::MemorySection::RxTSP, { 0x0400, 0x040F } },
    { LMS7002M::MemorySection::RxNCO, { 0x0440, 0x0461 } },
    { LMS7002M::MemorySection::RxGFIR1, { 0x0480, 0x04A7 } },
    { LMS7002M::MemorySection::RxGFIR2, { 0x04C0, 0x04E7 } },
    { LMS7002M::MemorySection::RxGFIR3a, { 0x0500, 0x0527 } },
    { LMS7002M::MemorySection::RxGFIR3b, { 0x0540, 0x0567 } },
    { LMS7002M::MemorySection::RxGFIR3c, { 0x0580, 0x05A7 } },
    { LMS7002M::MemorySection::RSSI_DC_CALIBRATION, { 0x05C0, 0x05CC } },
    { LMS7002M::MemorySection::RSSI_PDET_TEMP_CONFIG, { 0x0600, 0x0606 } },
    { LMS7002M::MemorySection::RSSI_DC_CONFIG, { 0x0640, 0x0641 } },
};

static_assert(OpStatus::Success == static_cast<lime::OpStatus>(lime_Result_Success));
static_assert(OpStatus::Error == static_cast<lime::OpStatus>(lime_Result_Error));
static_assert(OpStatus::NotImplemented == static_cast<lime::OpStatus>(lime_Result_NotImplemented));
static_assert(OpStatus::IOFailure == static_cast<lime::OpStatus>(lime_Result_IOFailure));
static_assert(OpStatus::InvalidValue == static_cast<lime::OpStatus>(lime_Result_InvalidValue));
static_assert(OpStatus::FileNotFound == static_cast<lime::OpStatus>(lime_Result_FileNotFound));
static_assert(OpStatus::OutOfRange == static_cast<lime::OpStatus>(lime_Result_OutOfRange));
static_assert(OpStatus::NotSupported == static_cast<lime::OpStatus>(lime_Result_NotSupported));
static_assert(OpStatus::Timeout == static_cast<lime::OpStatus>(lime_Result_Timeout));
static_assert(OpStatus::Busy == static_cast<lime::OpStatus>(lime_Result_Busy));
static_assert(OpStatus::Aborted == static_cast<lime::OpStatus>(lime_Result_Aborted));
static_assert(OpStatus::PermissionDenied == static_cast<lime::OpStatus>(lime_Result_PermissionDenied));
static_assert(OpStatus::NotConnected == static_cast<lime::OpStatus>(lime_Result_NotConnected));

static OpStatus ResultToStatus(lime_Result result)
{
    return static_cast<lime::OpStatus>(result);
}

/** @brief Switches LMS7002M SPI to requested channel and restores previous channel when going out of scope */
class ChannelScope
{
  public:
    /**
   * @brief Saves the current channel and restores it at scope exit.
   * @param chip The chip to use.
   * @param useCache Whether to use caching or not.
   */
    ChannelScope(LMS7002M* chip, bool useCache = false)
        : mChip(chip)
        , mStoredValue(chip->GetActiveChannel(!useCache))
        , mNeedsRestore(true)
    {
    }

    /**
      @brief Convenient constructor when using explicit MAC value.
      @param chip The chip to use.
      @param mac The channel to use.
      @param useCache Whether to use caching or not.
     */
    ChannelScope(LMS7002M* chip, LMS7002M::Channel mac, bool useCache = false)
        : mChip(chip)
        , mStoredValue(chip->GetActiveChannel(!useCache))
        , mNeedsRestore(false)
    {
        if (mStoredValue == mac)
            return;

        mChip->SetActiveChannel(mac);
        mNeedsRestore = true;
    }

    /**
      @brief Convenient constructor when using channel index starting from 0.
      @param chip The chip to use.
      @param index The channel index.
      @param useCache Whether to use caching or not.
     */
    ChannelScope(LMS7002M* chip, uint8_t index, bool useCache = false)
        : mChip(chip)
        , mNeedsRestore(false)
    {
        assert(index < 2);
        mStoredValue = chip->GetActiveChannel(!useCache);
        auto expectedChannel = IntToChannel(index);
        if (mStoredValue == expectedChannel)
            return;

        mChip->SetActiveChannel(expectedChannel);
        mNeedsRestore = true;
    }

    /** @brief Destroy the Channel Scope object and reset the active channel. */
    ~ChannelScope()
    {
        if (mNeedsRestore)
            mChip->SetActiveChannel(mStoredValue);
    }

  private:
    LMS7002M* mChip; ///< The chip to modify
    LMS7002M::Channel mStoredValue; ///< The channel to restore to
    bool mNeedsRestore; ///< Whether the channel needs restoring or not
};

/** @brief Sets connection which is used for data communication with chip
*/
void LMS7002M::SetConnection(std::shared_ptr<ISPI> port)
{
    controlPort = port;

    if (controlPort == nullptr)
    {
        return;
    }

    unsigned byte_array_size = 0;
    unsigned chipRev = this->Get_SPI_Reg_bits(MASK, true);
    if (chipRev >= 1)
        byte_array_size = 1024 * 16;
    else
        byte_array_size = 1024 * 8;
    mcuControl->Initialize(port, byte_array_size);
}

static int spi16_transact(const uint32_t* mosi, uint32_t* miso, uint32_t count, void* userData)
{
    LMS7002M* chip = reinterpret_cast<LMS7002M*>(userData);
    for (uint32_t i = 0; i < count; ++i)
    {
        if (mosi[i] & (1 << 31))
        {
            uint16_t addr = mosi[i] >> 16;
            addr &= 0x7FFF; // clear write bit for now
            uint16_t value = mosi[i] & 0xFFFF;
            chip->SPI_write(addr, value);
        }
        else
        {
            uint16_t addr = mosi[i] >> 16;
            uint16_t value = 0;
            value = chip->SPI_read(addr);
            if (miso)
                miso[i] = value;
        }
    }
    return 0;
}

static void log_hook(int logLevel, const char* message, void* userData)
{
    LogLevel level = static_cast<lime::LogLevel>(logLevel);
    log(level, std::string{ message });
}

/** @brief Creates LMS7002M main control object.
It requires IConnection to be set by SetConnection() to communicate with chip
*/
LMS7002M::LMS7002M(std::shared_ptr<ISPI> port)
    : mCallback_onCGENChange(nullptr)
    , mCallback_onCGENChange_userData(nullptr)
    , useCache(0)
    , mRegistersMap(new LMS7002M_RegistersMap())
    , controlPort(port)
    , mC_impl(nullptr)
{
    struct lms7002m_hooks hooks;
    memset(&hooks, 0, sizeof(hooks));

    hooks.spi16_userData = this;
    hooks.spi16_transact = spi16_transact;
    hooks.log = log_hook;
    hooks.log_userData = this;
    hooks.on_cgen_frequency_changed_userData = this;
    hooks.on_cgen_frequency_changed = [](void* userData) -> void {
        LMS7002M* chip = reinterpret_cast<LMS7002M*>(userData);
        if (chip->mCallback_onCGENChange)
            chip->mCallback_onCGENChange(chip->mCallback_onCGENChange_userData);
    };

    mC_impl = lms7002m_create(&hooks);
    if (mC_impl == nullptr)
        lime::error("Failed to initialize LMS7002M C implementation");
    lms7002m_set_reference_clock(mC_impl, 30.72e6);

    opt_gain_tbb[0] = -1;
    opt_gain_tbb[1] = -1;

    std::vector<const LMS7002MCSR_Data::CSRegister*> parameterList;
    parameterList.reserve(static_cast<int>(LMS7002MCSR::ENUM_COUNT));
    for (int i = 0; i < static_cast<int>(LMS7002MCSR::ENUM_COUNT); ++i)
    {
        const LMS7002MCSR_Data::CSRegister& parameter = GetRegister(static_cast<LMS7002MCSR>(i));
        parameterList.push_back(&parameter);
    }
    mRegistersMap->InitializeDefaultValues(parameterList);
    mcuControl = new MCU_BD();
    mcuControl->Initialize(controlPort);
}

LMS7002M::~LMS7002M()
{
    lms7002m_destroy(mC_impl);
    delete mcuControl;
    delete mRegistersMap;
}

OpStatus LMS7002M::SetActiveChannel(const Channel ch)
{
    lime_Result result = lms7002m_set_active_channel(mC_impl, static_cast<uint8_t>(ch));
    return ResultToStatus(result);
}

LMS7002M::Channel LMS7002M::GetActiveChannel(bool fromChip)
{
    auto result{ GetActiveChannelIndex(fromChip) };
    return static_cast<Channel>(result);
}

size_t LMS7002M::GetActiveChannelIndex(bool fromChip)
{
    uint8_t result = lms7002m_get_active_channel(mC_impl);
    return result;
}

OpStatus LMS7002M::EnableChannel(TRXDir dir, const uint8_t channel, const bool enable)
{
    lime_Result result = lms7002m_enable_channel(mC_impl, dir == TRXDir::Tx, channel, enable);
    return ResultToStatus(result);
}

OpStatus LMS7002M::ResetChip()
{
    OpStatus status;

    const std::vector<uint16_t> usedAddresses = mRegistersMap->GetUsedAddresses(0);

    std::vector<uint16_t> addrs;
    addrs.reserve(usedAddresses.size() + 2);
    std::vector<uint16_t> values;
    values.reserve(usedAddresses.size() + 2);

    addrs.push_back(0x0006); // SPISW_CTRL
    values.push_back(0x0000); // ensure baseband is controlling SPI

    addrs.push_back(0x0020);
    uint16_t x0020default = mRegistersMap->GetDefaultValue(0x0020);
    values.push_back(x0020default | 0x3); // enable simultaneous A&B write

    for (uint16_t addr : usedAddresses)
    {
        if (addr == 0x0020) // skip address containing MAC, to continue writing both channels
            continue;
        addrs.push_back(addr);
        values.push_back(mRegistersMap->GetDefaultValue(addr));
    }
    addrs.push_back(0x0020);
    values.push_back((x0020default & ~0x3) | 0x1); // back to A channel only

    status = SPI_write_batch(addrs.data(), values.data(), addrs.size(), true);
    status = Modify_SPI_Reg_bits(LMS7002MCSR::MIMO_SISO, 0); //enable B channel after reset
    return status;
}

OpStatus LMS7002M::SoftReset()
{
    lime_Result result = lms7002m_soft_reset(mC_impl);
    return ResultToStatus(result);
}

OpStatus LMS7002M::ResetLogicRegisters()
{
    lime_Result result = lms7002m_reset_logic_registers(mC_impl);
    return ResultToStatus(result);
}

OpStatus LMS7002M::LoadConfigLegacyFile(const std::string& filename)
{
    std::ifstream f(filename);
    if (f.good() == false) //file not found
    {
        f.close();
        return ReportError(OpStatus::FileNotFound, "LoadConfigLegacyFile(%s) - file not found", filename.c_str());
    }
    f.close();

    uint16_t addr = 0;
    uint16_t value = 0;
    OpStatus status;
    typedef INI<std::string, std::string, std::string> ini_t;
    ini_t parser(filename, true);
    if (parser.select("FILE INFO"s) == false)
        return ReportError(
            OpStatus::InvalidValue, "LoadConfigLegacyFile(%s) - invalid format, missing FILE INFO section", filename.c_str());

    std::string type{};
    type = parser.get("type"s, "undefined"s);

    if (type.find("LMS7002 configuration"sv) == std::string::npos)
    {
        return ReportError(
            OpStatus::InvalidValue, "LoadConfigLegacyFile(%s) - invalid format, missing LMS7002 configuration", filename.c_str());
    }

    int fileVersion = 0;
    fileVersion = parser.get("version"sv, 0);

    std::vector<uint16_t> addrToWrite;
    std::vector<uint16_t> dataToWrite;
    if (fileVersion != 1)
    {
        return ReportError(OpStatus::InvalidValue, "LoadConfigLegacyFile(%s) - invalid format", filename.c_str());
    }

    ChannelScope scope(this);

    if (parser.select("Reference clocks"s))
    {
        this->SetReferenceClk_SX(TRXDir::Rx, parser.get("SXR reference frequency MHz"sv, 30.72) * 1e6);
        this->SetReferenceClk_SX(TRXDir::Tx, parser.get("SXT reference frequency MHz"sv, 30.72) * 1e6);
    }

    if (parser.select("LMS7002 registers ch.A"s) == true)
    {
        ini_t::sectionsit_t section = parser.sections.find("LMS7002 registers ch.A"s);

        uint16_t x0020_value = 0;
        this->SetActiveChannel(Channel::ChA); //select A channel
        for (ini_t::keysit_t pairs = section->second->begin(); pairs != section->second->end(); pairs++)
        {
            sscanf(pairs->first.c_str(), "%hx", &addr);
            sscanf(pairs->second.c_str(), "%hx", &value);
            if (addr == MAC.address) //skip register containing channel selection
            {
                x0020_value = value;
                continue;
            }
            addrToWrite.push_back(addr);
            dataToWrite.push_back(value);
        }
        status = SPI_write_batch(&addrToWrite[0], &dataToWrite[0], addrToWrite.size(), true);
        if (status != OpStatus::Success && controlPort != nullptr)
            return status;

        //parse FCW or PHO
        if (parser.select("NCO Rx ch.A"s) == true)
        {
            char varname[64];
            int mode = Get_SPI_Reg_bits(LMS7002MCSR::MODE_RX);
            if (mode == 0) //FCW
            {
                for (int i = 0; i < 16; ++i)
                {
                    std::snprintf(varname, sizeof(varname), "FCW%02i", i);
                    SetNCOFrequency(TRXDir::Rx, i, parser.get(varname, 0.0));
                }
            }
            else
            {
                for (int i = 0; i < 16; ++i)
                {
                    std::snprintf(varname, sizeof(varname), "PHO%02i", i);
                    SetNCOPhaseOffset(TRXDir::Rx, i, parser.get(varname, 0.0));
                }
            }
        }
        if (parser.select("NCO Tx ch.A"s) == true)
        {
            char varname[64];
            int mode = Get_SPI_Reg_bits(LMS7002MCSR::MODE_TX);
            if (mode == 0) //FCW
            {
                for (int i = 0; i < 16; ++i)
                {
                    std::snprintf(varname, sizeof(varname), "FCW%02i", i);
                    SetNCOFrequency(TRXDir::Tx, i, parser.get(varname, 0.0));
                }
            }
            else
            {
                for (int i = 0; i < 16; ++i)
                {
                    std::snprintf(varname, sizeof(varname), "PHO%02i", i);
                    SetNCOPhaseOffset(TRXDir::Tx, i, parser.get(varname, 0.0));
                }
            }
        }
        status = SPI_write(0x0020, x0020_value);
        if (status != OpStatus::Success && controlPort != nullptr)
            return status;
    }

    this->SetActiveChannel(Channel::ChB);

    if (parser.select("LMS7002 registers ch.B"s) == true)
    {
        addrToWrite.clear();
        dataToWrite.clear();
        ini_t::sectionsit_t section = parser.sections.find("LMS7002 registers ch.B"s);
        for (ini_t::keysit_t pairs = section->second->begin(); pairs != section->second->end(); pairs++)
        {
            sscanf(pairs->first.c_str(), "%hx", &addr);
            sscanf(pairs->second.c_str(), "%hx", &value);
            addrToWrite.push_back(addr);
            dataToWrite.push_back(value);
        }
        this->SetActiveChannel(Channel::ChB); //select B channel
        status = SPI_write_batch(&addrToWrite[0], &dataToWrite[0], addrToWrite.size(), true);
        if (status != OpStatus::Success && controlPort != nullptr)
            return status;

        //parse FCW or PHO
        if (parser.select("NCO Rx ch.B"s) == true)
        {
            char varname[64];
            int mode = Get_SPI_Reg_bits(LMS7002MCSR::MODE_RX);
            if (mode == 0) //FCW
            {
                for (int i = 0; i < 16; ++i)
                {
                    std::snprintf(varname, sizeof(varname), "FCW%02i", i);
                    SetNCOFrequency(TRXDir::Rx, i, parser.get(varname, 0.0));
                }
            }
            else
            {
                for (int i = 0; i < 16; ++i)
                {
                    std::snprintf(varname, sizeof(varname), "PHO%02i", i);
                    SetNCOPhaseOffset(TRXDir::Rx, i, parser.get(varname, 0.0));
                }
            }
        }
        if (parser.select("NCO Tx ch.A"s) == true)
        {
            char varname[64];
            int mode = Get_SPI_Reg_bits(LMS7002MCSR::MODE_TX);
            if (mode == 0) //FCW
            {
                for (int i = 0; i < 16; ++i)
                {
                    std::snprintf(varname, sizeof(varname), "FCW%02i", i);
                    SetNCOFrequency(TRXDir::Tx, i, parser.get(varname, 0.0));
                }
            }
            else
            {
                for (int i = 0; i < 16; ++i)
                {
                    std::snprintf(varname, sizeof(varname), "PHO%02i", i);
                    SetNCOPhaseOffset(TRXDir::Tx, i, parser.get(varname, 0.0));
                }
            }
        }
    }
    return OpStatus::Success;
}

OpStatus LMS7002M::LoadConfig(const std::string& filename, bool tuneDynamicValues)
{
    std::ifstream f(filename);
    if (f.good() == false) //file not found
    {
        f.close();
        return ReportError(OpStatus::FileNotFound, "LoadConfig(%s) - file not found", filename.c_str());
    }
    f.close();

    uint16_t addr = 0;
    uint16_t value = 0;

    OpStatus status;
    typedef INI<std::string, std::string, std::string> ini_t;
    ini_t parser(filename, true);
    if (parser.select("file_info"s) == false)
    {
        //try loading as legacy format
        status = LoadConfigLegacyFile(filename);
        this->SetActiveChannel(Channel::ChA);
        return status;
    }
    std::string type{};
    type = parser.get("type"s, "undefined"s);

    if (type.find("lms7002m_minimal_config"sv) == std::string::npos)
    {
        return ReportError(
            OpStatus::InvalidValue, "LoadConfig(%s) - invalid format, missing lms7002m_minimal_config", filename.c_str());
    }

    int fileVersion = 0;
    fileVersion = parser.get("version"sv, 0);

    std::vector<uint16_t> addrToWrite;
    std::vector<uint16_t> dataToWrite;

    if (fileVersion == 1)
    {
        ChannelScope scope(this);
        if (parser.select("lms7002_registers_a"s) == true)
        {
            ini_t::sectionsit_t section = parser.sections.find("lms7002_registers_a"s);

            uint16_t x0020_value = 0;
            this->SetActiveChannel(Channel::ChA); //select A channel
            for (ini_t::keysit_t pairs = section->second->begin(); pairs != section->second->end(); pairs++)
            {
                sscanf(pairs->first.c_str(), "%hx", &addr);
                sscanf(pairs->second.c_str(), "%hx", &value);
                if (addr == MAC.address) //skip register containing channel selection
                {
                    x0020_value = value;
                    continue;
                }

                if (addr >= 0x5C3 && addr <= 0x5CA) //enable analog DC correction
                {
                    addrToWrite.push_back(addr);
                    dataToWrite.push_back(value & 0x3FFF);
                    addrToWrite.push_back(addr);
                    dataToWrite.push_back(value | 0x8000);
                }
                else
                {
                    addrToWrite.push_back(addr);
                    dataToWrite.push_back(value);
                }
            }

            status = SPI_write_batch(&addrToWrite[0], &dataToWrite[0], addrToWrite.size(), true);
            if (status != OpStatus::Success && controlPort != nullptr)
                return status;
            status = SPI_write(0x0020, x0020_value);
            if (status != OpStatus::Success && controlPort != nullptr)
                return status;
            this->SetActiveChannel(Channel::ChB);
            if (status != OpStatus::Success && controlPort != nullptr)
                return status;
        }

        if (parser.select("lms7002_registers_b"s) == true)
        {
            addrToWrite.clear();
            dataToWrite.clear();
            ini_t::sectionsit_t section = parser.sections.find("lms7002_registers_b"s);
            for (ini_t::keysit_t pairs = section->second->begin(); pairs != section->second->end(); pairs++)
            {
                sscanf(pairs->first.c_str(), "%hx", &addr);
                sscanf(pairs->second.c_str(), "%hx", &value);
                addrToWrite.push_back(addr);
                dataToWrite.push_back(value);
            }
            this->SetActiveChannel(Channel::ChB); //select B channel
            status = SPI_write_batch(&addrToWrite[0], &dataToWrite[0], addrToWrite.size(), true);
            if (status != OpStatus::Success && controlPort != nullptr)
                return status;
        }

        parser.select("reference_clocks"s);
        this->SetReferenceClk_SX(TRXDir::Rx, parser.get("sxr_ref_clk_mhz"sv, 30.72) * 1e6);
        this->SetReferenceClk_SX(TRXDir::Tx, parser.get("sxt_ref_clk_mhz"sv, 30.72) * 1e6);
    }

    ResetLogicRegisters();

    if (tuneDynamicValues)
    {
        Modify_SPI_Reg_bits(LMS7002MCSR::MAC, 2);
        if (!Get_SPI_Reg_bits(LMS7002MCSR::PD_VCO))
            TuneVCO(VCO_Module::VCO_SXT);
        Modify_SPI_Reg_bits(LMS7002MCSR::MAC, 1);
        if (!Get_SPI_Reg_bits(LMS7002MCSR::PD_VCO))
            TuneVCO(VCO_Module::VCO_SXR);
        if (!Get_SPI_Reg_bits(LMS7002MCSR::PD_VCO_CGEN))
        {
            TuneVCO(VCO_Module::VCO_CGEN);
            if (mCallback_onCGENChange)
                return mCallback_onCGENChange(mCallback_onCGENChange_userData);
        }
    }
    this->SetActiveChannel(Channel::ChA);
    return OpStatus::Success;
}

OpStatus LMS7002M::SaveConfig(const std::string& filename)
{
    std::ofstream fout;
    fout.open(filename);
    fout << "[file_info]"sv << std::endl;
    fout << "type=lms7002m_minimal_config"sv << std::endl;
    fout << "version=1"sv << std::endl;

    char addr[80];
    char value[80];

    ChannelScope scope(this);

    std::vector<uint16_t> addrToRead;
    for (const auto& memorySectionPair : MemorySectionAddresses)
        for (uint16_t addr = memorySectionPair.second[0]; addr <= memorySectionPair.second[1]; ++addr)
            addrToRead.push_back(addr);

    std::vector<uint16_t> dataReceived;
    dataReceived.resize(addrToRead.size(), 0);

    fout << "[lms7002_registers_a]"sv << std::endl;
    this->SetActiveChannel(Channel::ChA);
    for (uint16_t i = 0; i < addrToRead.size(); ++i)
    {
        if (addrToRead[i] >= 0x5C3 && addrToRead[i] <= 0x5CA)
            SPI_write(addrToRead[i], 0x4000); //perform read-back from DAC
        dataReceived[i] = Get_SPI_Reg_bits(addrToRead[i], 15, 0, false);

        //registers 0x5C3 - 0x53A return inverted value field when DAC value read-back is performed
        if (addrToRead[i] >= 0x5C3 && addrToRead[i] <= 0x5C6 && (dataReceived[i] & 0x400)) //sign bit 10
            dataReceived[i] = 0x400 | (~dataReceived[i] & 0x3FF); //magnitude bits  9:0
        else if (addrToRead[i] >= 0x5C7 && addrToRead[i] <= 0x5CA && (dataReceived[i] & 0x40)) //sign bit 6
            dataReceived[i] = 0x40 | (~dataReceived[i] & 0x3F); //magnitude bits  5:0
        else if (addrToRead[i] == 0x5C2)
            dataReceived[i] &= 0xFF00; //do not save calibration start triggers
        std::snprintf(addr, sizeof(addr), "0x%04X", addrToRead[i]);
        std::snprintf(value, sizeof(value), "0x%04X", dataReceived[i]);
        fout << addr << "="sv << value << std::endl;
        // add parameter name/value as comments
        for (int p = 0; p < static_cast<int>(LMS7002MCSR::ENUM_COUNT); ++p)
        {
            const LMS7002MCSR_Data::CSRegister& parameter = GetRegister(static_cast<LMS7002MCSR>(p));
            if (parameter.address == addrToRead[i])
                fout << "//"sv << parameter.name << " : "sv << Get_SPI_Reg_bits(static_cast<LMS7002MCSR>(p)) << std::endl;
        }
    }

    fout << "[lms7002_registers_b]"sv << std::endl;
    addrToRead.clear(); //add only B channel addresses
    for (const auto& memorySectionPair : MemorySectionAddresses)
        if (memorySectionPair.first != MemorySection::RSSI_DC_CALIBRATION)
            for (uint16_t addr = memorySectionPair.second[0]; addr <= memorySectionPair.second[1]; ++addr)
                if (addr >= 0x0100)
                    addrToRead.push_back(addr);

    this->SetActiveChannel(Channel::ChB);
    for (uint16_t i = 0; i < addrToRead.size(); ++i)
    {
        dataReceived[i] = Get_SPI_Reg_bits(addrToRead[i], 15, 0, false);
        std::snprintf(addr, sizeof(addr), "0x%04X", addrToRead[i]);
        std::snprintf(value, sizeof(value), "0x%04X", dataReceived[i]);
        fout << addr << "="sv << value << std::endl;
    }

    fout << "[reference_clocks]"sv << std::endl;
    fout << "sxt_ref_clk_mhz="sv << this->GetReferenceClk_SX(TRXDir::Tx) / 1e6 << std::endl;
    fout << "sxr_ref_clk_mhz="sv << this->GetReferenceClk_SX(TRXDir::Rx) / 1e6 << std::endl;
    fout.close();
    return OpStatus::Success;
}

OpStatus LMS7002M::SetRBBPGA_dB(const float_type value, const Channel channel)
{
    lime_Result result = lms7002m_set_rbbpga_db(mC_impl, value, static_cast<uint8_t>(channel));
    return ResultToStatus(result);
}

float_type LMS7002M::GetRBBPGA_dB(const Channel channel)
{
    return lms7002m_get_rbbpga_db(mC_impl, static_cast<uint8_t>(channel));
}

OpStatus LMS7002M::SetRFELNA_dB(const float_type value, const Channel channel)
{
    lime_Result result = lms7002m_set_rfelna_db(mC_impl, value, static_cast<uint8_t>(channel));
    return ResultToStatus(result);
}

float_type LMS7002M::GetRFELNA_dB(const Channel channel)
{
    return lms7002m_get_rfelna_db(mC_impl, static_cast<uint8_t>(channel));
}

OpStatus LMS7002M::SetRFELoopbackLNA_dB(const float_type gain, const Channel channel)
{
    lime_Result result = lms7002m_set_rfe_loopback_lna_db(mC_impl, gain, static_cast<uint8_t>(channel));
    return ResultToStatus(result);
}

float_type LMS7002M::GetRFELoopbackLNA_dB(const Channel channel)
{
    return lms7002m_get_rfe_loopback_lna_db(mC_impl, static_cast<uint8_t>(channel));
}

OpStatus LMS7002M::SetRFETIA_dB(const float_type value, const Channel channel)
{
    lime_Result result = lms7002m_set_rfetia_db(mC_impl, value, static_cast<uint8_t>(channel));
    return ResultToStatus(result);
}

float_type LMS7002M::GetRFETIA_dB(const Channel channel)
{
    return lms7002m_get_rfetia_db(mC_impl, static_cast<uint8_t>(channel));
}

OpStatus LMS7002M::SetTRFPAD_dB(const float_type value, const Channel channel)
{
    lime_Result result = lms7002m_set_trfpad_db(mC_impl, value, static_cast<uint8_t>(channel));
    return ResultToStatus(result);
}

float_type LMS7002M::GetTRFPAD_dB(const Channel channel)
{
    return lms7002m_get_trfpad_db(mC_impl, static_cast<uint8_t>(channel));
}

OpStatus LMS7002M::SetTRFLoopbackPAD_dB(const float_type gain, const Channel channel)
{
    lime_Result result = lms7002m_set_trf_loopback_pad_db(mC_impl, gain, static_cast<uint8_t>(channel));
    return ResultToStatus(result);
}

float_type LMS7002M::GetTRFLoopbackPAD_dB(const Channel channel)
{
    return lms7002m_get_trf_loopback_pad_db(mC_impl, static_cast<uint8_t>(channel));
}

// opt_gain_tbb
OpStatus LMS7002M::SetTBBIAMP_dB(const float_type gain, const Channel channel)
{
    OpStatus status;
    ChannelScope scope(this, channel);

    int ind = this->GetActiveChannelIndex() % 2;
    if (opt_gain_tbb[ind] <= 0)
    {
        status = CalibrateTxGain();
        if (status != OpStatus::Success) //set optimal BB gain
            return status;
        if (std::fabs(gain) < 0.2) // optimal gain = ~0dB
            return OpStatus::Success;
    }

    int g_iamp = static_cast<float_type>(opt_gain_tbb[ind]) * pow(10.0, gain / 20.0) + 0.4;
    status = Modify_SPI_Reg_bits(LMS7002MCSR::CG_IAMP_TBB, std::clamp(g_iamp, 1, 63), true);

    return status;
}

// opt_gain_tbb
float_type LMS7002M::GetTBBIAMP_dB(const Channel channel)
{
    ChannelScope scope(this, channel);

    int g_current = Get_SPI_Reg_bits(LMS7002MCSR::CG_IAMP_TBB, true);
    int ind = this->GetActiveChannelIndex() % 2;

    if (opt_gain_tbb[ind] <= 0)
    {
        if (CalibrateTxGain() != OpStatus::Success)
            return 0.0;
        Modify_SPI_Reg_bits(LMS7002MCSR::CG_IAMP_TBB, g_current, true); //restore
    }
    return 20.0 * log10(static_cast<float_type>(g_current) / static_cast<float_type>(opt_gain_tbb[ind]));
}

OpStatus LMS7002M::SetPathRFE(PathRFE path)
{
    lime_Result result = lms7002m_set_path_rfe(mC_impl, static_cast<uint8_t>(path));
    return ResultToStatus(result);
}

LMS7002M::PathRFE LMS7002M::GetPathRFE(void)
{
    return static_cast<PathRFE>(lms7002m_get_path_rfe(mC_impl));
}

OpStatus LMS7002M::SetBandTRF(const int band)
{
    lime_Result result = lms7002m_set_band_trf(mC_impl, static_cast<uint8_t>(band));
    return ResultToStatus(result);
}

int LMS7002M::GetBandTRF(void)
{
    return lms7002m_get_band_trf(mC_impl);
}

OpStatus LMS7002M::SetPath(TRXDir direction, uint8_t channel, uint8_t path)
{
    ChannelScope scope(this, channel, false);

    if (direction == TRXDir::Tx)
    {
        return SetBandTRF(path);
    }

    return SetPathRFE(lime::LMS7002M::PathRFE(path));
}

OpStatus LMS7002M::SetReferenceClk_SX(TRXDir dir, float_type freq_Hz)
{
    lime_Result result = lms7002m_set_reference_clock(mC_impl, freq_Hz);
    return ResultToStatus(result);
}

float_type LMS7002M::GetReferenceClk_SX(TRXDir dir)
{
    return lms7002m_get_reference_clock(mC_impl);
}

OpStatus LMS7002M::SetNCOFrequencies(TRXDir dir, const float_type* freq_Hz, uint8_t count, float_type phaseOffset)
{
    for (uint8_t i = 0; i < 16 && i < count; i++)
    {
        OpStatus status = SetNCOFrequency(dir, i, freq_Hz[i]);
        if (status != OpStatus::Success)
            return status;
    }
    return SetNCOPhaseOffsetForMode0(dir, phaseOffset);
}

std::vector<float_type> LMS7002M::GetNCOFrequencies(TRXDir dir, float_type* phaseOffset)
{
    std::vector<float_type> ncos;
    for (int i = 0; i < 16; ++i)
        ncos.push_back(GetNCOFrequency(dir, i, !useCache));
    if (phaseOffset != nullptr)
    {
        uint16_t value = SPI_read(dir == TRXDir::Tx ? 0x0241 : 0x0441);
        *phaseOffset = 360.0 * value / 65536.0;
    }
    return ncos;
}

float_type LMS7002M::GetFrequencyCGEN()
{
    float_type dMul =
        (GetReferenceClk_SX(TRXDir::Rx) / 2.0) / (Get_SPI_Reg_bits(LMS7002MCSR::DIV_OUTCH_CGEN, true) + 1); //DIV_OUTCH_CGEN
    uint16_t gINT = Get_SPI_Reg_bits(0x0088, 13, 0, true); //read whole register to reduce SPI transfers
    uint32_t gFRAC = ((gINT & 0xF) * 65536) | Get_SPI_Reg_bits(0x0087, 15, 0, true);
    return dMul * (((gINT >> 4) + 1 + gFRAC / 1048576.0));
}

float_type LMS7002M::GetReferenceClk_TSP(TRXDir dir)
{
    float_type cgenFreq = GetFrequencyCGEN();
    float_type clklfreq = cgenFreq / pow(2.0, Get_SPI_Reg_bits(LMS7002MCSR::CLKH_OV_CLKL_CGEN, true));
    if (Get_SPI_Reg_bits(LMS7002MCSR::EN_ADCCLKH_CLKGN, true) == 0)
        return dir == TRXDir::Tx ? clklfreq : cgenFreq / 4.0;
    else
        return dir == TRXDir::Tx ? cgenFreq : clklfreq / 4.0;
}

OpStatus LMS7002M::SetFrequencyCGEN(const float_type freq_Hz, const bool retainNCOfrequencies, CGEN_details* output)
{
    lime_Result result = lms7002m_set_frequency_cgen(mC_impl, freq_Hz);
    return ResultToStatus(result);
}

bool LMS7002M::GetCGENLocked(void)
{
    return (Get_SPI_Reg_bits(VCO_CMPHO_CGEN.address, 13, 12, true) & 0x3) == 2;
}

bool LMS7002M::GetSXLocked(TRXDir dir)
{
    SetActiveChannel(dir == TRXDir::Tx ? Channel::ChSXT : Channel::ChSXR);
    return (Get_SPI_Reg_bits(VCO_CMPHO.address, 13, 12, true) & 0x3) == 2;
}

OpStatus LMS7002M::TuneCGENVCO()
{
    lime_Result result = lms7002m_tune_cgen_vco(mC_impl);
    return ResultToStatus(result);
}

OpStatus LMS7002M::TuneVCO(VCO_Module module) // 0-cgen, 1-SXR, 2-SXT
{
    if (module == VCO_Module::VCO_CGEN)
        return TuneCGENVCO();
    auto settlingTime = std::chrono::microseconds(50); //can be lower
    struct CSWInteval {
        int16_t high;
        int16_t low;
    };
    CSWInteval cswSearch[2];
    const char* moduleName = (module == VCO_Module::VCO_CGEN) ? "CGEN" : ((module == VCO_Module::VCO_SXR) ? "SXR" : "SXT");
    uint8_t cmphl; //comparators
    uint16_t addrVCOpd; // VCO power down address
    uint16_t addrCSW_VCO;
    uint16_t addrCMP; //comparator address
    uint8_t lsb; //SWC lsb index
    uint8_t msb; //SWC msb index

    ChannelScope scope(this);

    if (module != VCO_Module::VCO_CGEN) //set addresses to SX module
    {
        this->SetActiveChannel(Channel(module));
        addrVCOpd = PD_VCO.address;
        addrCSW_VCO = CSW_VCO.address;
        lsb = CSW_VCO.lsb;
        msb = CSW_VCO.msb;
        addrCMP = VCO_CMPHO.address;
        lime::debug("TuneVCO(%s) ICT_VCO: %d", moduleName, Get_SPI_Reg_bits(LMS7002MCSR::ICT_VCO));
    }
    else //set addresses to CGEN module
    {
        addrVCOpd = PD_VCO_CGEN.address;
        addrCSW_VCO = CSW_VCO_CGEN.address;
        lsb = CSW_VCO_CGEN.lsb;
        msb = CSW_VCO_CGEN.msb;
        addrCMP = VCO_CMPHO_CGEN.address;
        lime::debug("TuneVCO(%s) ICT_VCO_CGEN: %d", moduleName, Get_SPI_Reg_bits(LMS7002MCSR::ICT_VCO_CGEN));
    }
    // Initialization activate VCO and comparator
    OpStatus status = Modify_SPI_Reg_bits(addrVCOpd, 2, 1, 0);
    if (status != OpStatus::Success)
        return status;
    if (Get_SPI_Reg_bits(addrVCOpd, 2, 1) != 0)
        return ReportError(OpStatus::Error, "TuneVCO(%s) - VCO is powered down", moduleName);

    //check if lock is within VCO range
    {
        Modify_SPI_Reg_bits(addrCSW_VCO, msb, lsb, 0);
        std::this_thread::sleep_for(settlingTime);
        cmphl = static_cast<uint8_t>(Get_SPI_Reg_bits(addrCMP, 13, 12, true));
        if (cmphl == 3) //VCO too high
        {
            lime::debug("TuneVCO(%s) - attempted VCO too high", moduleName);
            return OpStatus::Error;
        }
        Modify_SPI_Reg_bits(addrCSW_VCO, msb, lsb, 255);
        std::this_thread::sleep_for(settlingTime);
        cmphl = static_cast<uint8_t>(Get_SPI_Reg_bits(addrCMP, 13, 12, true));
        if (cmphl == 0) //VCO too low
        {
            lime::debug("TuneVCO(%s) - attempted VCO too low", moduleName);
            return OpStatus::Error;
        }
    }

    //search intervals [0-127][128-255]
    for (int t = 0; t < 2; ++t)
    {
        bool hadLock = false;
        // initialize search range with invalid values
        cswSearch[t].low = 128 * (t + 1); // set low to highest possible value
        cswSearch[t].high = 128 * t; // set high to lowest possible value
        lime::debug("TuneVCO(%s) - searching interval [%i:%i]", moduleName, cswSearch[t].high, cswSearch[t].low);
        Modify_SPI_Reg_bits(addrCSW_VCO, msb, lsb, cswSearch[t].high);
        //binary search for and high value, and on the way store approximate low value
        lime::debug("binary search:"s);
        for (int i = 6; i >= 0; --i)
        {
            cswSearch[t].high |= 1 << i; //CSW_VCO<i>=1
            Modify_SPI_Reg_bits(addrCSW_VCO, msb, lsb, cswSearch[t].high);
            std::this_thread::sleep_for(settlingTime);
            cmphl = static_cast<uint8_t>(Get_SPI_Reg_bits(addrCMP, 13, 12, true));
            lime::debug("csw=%d\tcmphl=%d", cswSearch[t].high, cmphl);
            if (cmphl & 0x01) // reduce CSW
                cswSearch[t].high &= ~(1 << i); //CSW_VCO<i>=0
            if (cmphl == 2 && cswSearch[t].high < cswSearch[t].low)
            {
                cswSearch[t].low = cswSearch[t].high;
                hadLock = true;
            }
        }
        //linear search to make sure there are no gaps, and move away from edge case
        lime::debug("adjust with linear search:"s);
        while (cswSearch[t].low <= cswSearch[t].high && cswSearch[t].low > t * 128)
        {
            --cswSearch[t].low;
            Modify_SPI_Reg_bits(addrCSW_VCO, msb, lsb, cswSearch[t].low);
            std::this_thread::sleep_for(settlingTime);
            const uint8_t tempCMPvalue = Get_SPI_Reg_bits(addrCMP, 13, 12, true);
            lime::debug("csw=%d\tcmphl=%d", cswSearch[t].low, tempCMPvalue);
            if (tempCMPvalue != 2)
            {
                ++cswSearch[t].low;
                break;
            }
        }
        if (hadLock)
        {
            lime::debug("CSW: lowest=%d, highest=%d, will use=%d",
                cswSearch[t].low,
                cswSearch[t].high,
                cswSearch[t].low + (cswSearch[t].high - cswSearch[t].low) / 2);
        }
        else
            lime::debug("CSW interval failed to lock"s);
    }

    //check if the intervals are joined
    int16_t cswHigh, cswLow;
    if (cswSearch[0].high == cswSearch[1].low - 1)
    {
        cswHigh = cswSearch[1].high;
        cswLow = cswSearch[0].low;
        lime::debug("CSW is locking in one continous range: low=%d, high=%d", cswLow, cswHigh);
    }
    //compare which interval is wider
    else
    {
        uint8_t intervalIndex = (cswSearch[1].high - cswSearch[1].low > cswSearch[0].high - cswSearch[0].low);
        cswHigh = cswSearch[intervalIndex].high;
        cswLow = cswSearch[intervalIndex].low;
        lime::debug("choosing wider CSW locking range: low=%d, high=%d", cswLow, cswHigh);
    }

    uint8_t finalCSW = 0;
    if (cswHigh - cswLow == 1)
    {
        lime::debug("TuneVCO(%s) - narrow locking values range detected [%i:%i]. VCO lock status might change with temperature.",
            moduleName,
            cswLow,
            cswHigh);
        //check which of two values really locks
        finalCSW = cswLow;
        Modify_SPI_Reg_bits(addrCSW_VCO, msb, lsb, cswLow);
        std::this_thread::sleep_for(settlingTime);
        cmphl = static_cast<uint8_t>(Get_SPI_Reg_bits(addrCMP, 13, 12, true));
        if (cmphl != 2)
        {
            finalCSW = cswHigh;
            Modify_SPI_Reg_bits(addrCSW_VCO, msb, lsb, cswHigh);
        }
    }
    else
    {
        finalCSW = cswLow + (cswHigh - cswLow) / 2;
        Modify_SPI_Reg_bits(addrCSW_VCO, msb, lsb, finalCSW);
    }
    std::this_thread::sleep_for(settlingTime);
    cmphl = static_cast<uint8_t>(Get_SPI_Reg_bits(addrCMP, 13, 12, true));
    if (cmphl == 2)
    {
        lime::debug("TuneVCO(%s) - confirmed lock with final csw=%i, cmphl=%i", moduleName, finalCSW, cmphl);
        return OpStatus::Success;
    }
    lime::debug("TuneVCO(%s) - failed lock with final csw=%i, cmphl=%i", moduleName, finalCSW, cmphl);
    return OpStatus::Error;
}

uint16_t LMS7002M::Get_SPI_Reg_bits(const CSRegister& param, bool fromChip)
{
    return Get_SPI_Reg_bits(param.address, param.msb, param.lsb, fromChip);
}

uint16_t LMS7002M::Get_SPI_Reg_bits(uint16_t address, uint8_t msb, uint8_t lsb, bool fromChip)
{
    return (SPI_read(address, fromChip) & (~(~0u << (msb + 1)))) >> lsb; //shift bits to LSB
}

uint16_t LMS7002M::Get_SPI_Reg_bits(const LMS7002MCSR param, bool fromChip)
{
    const CSRegister& reg = GetRegister(param);
    return Get_SPI_Reg_bits(reg.address, reg.msb, reg.lsb, fromChip);
}

OpStatus LMS7002M::Modify_SPI_Reg_bits(
    const uint16_t address, const uint8_t msb, const uint8_t lsb, const uint16_t value, bool fromChip)
{
    uint16_t spiDataReg = SPI_read(address, fromChip); //read current SPI reg data
    uint16_t spiMask = (~(~0u << (msb - lsb + 1))) << (lsb); // creates bit mask
    spiDataReg = (spiDataReg & (~spiMask)) | ((value << lsb) & spiMask); //clear bits
    return SPI_write(address, spiDataReg); //write modified data back to SPI reg
}

OpStatus LMS7002M::Modify_SPI_Reg_bits(const LMS7002MCSR_Data::CSRegister& param, const uint16_t value, bool fromChip)
{
    return Modify_SPI_Reg_bits(param.address, param.msb, param.lsb, value, fromChip);
}

OpStatus LMS7002M::Modify_SPI_Reg_bits(const LMS7002MCSR param, const uint16_t value, bool fromChip)
{
    const CSRegister& reg = GetRegister(param);
    return Modify_SPI_Reg_bits(reg, value, fromChip);
}

const CSRegister& LMS7002M::GetParam(const std::string& name)
{
    for (int i = 0; i < static_cast<int>(LMS7002MCSR::ENUM_COUNT); ++i)
    {
        const LMS7002MCSR_Data::CSRegister& parameter = GetRegister(static_cast<LMS7002MCSR>(i));
        if (std::string_view{ parameter.name } == name)
            return parameter;
    }

    throw std::logic_error("Parameter "s + name + " not found"s);
}

OpStatus LMS7002M::SetFrequencySX(TRXDir dir, float_type freq_Hz, SX_details* output)
{
    static std::map<float_type, int8_t> tuning_cache_sel_vco;
    static std::map<float_type, int16_t> tuning_cache_csw_value;

    assert(freq_Hz > 0);

    const char* vcoNames[] = { "VCOL", "VCOM", "VCOH" };
    const uint8_t sxVCO_N = 2; //number of entries in VCO frequencies
    const float_type m_dThrF = 5500e6; //threshold to enable additional divider
    float_type VCOfreq;
    int8_t div_loch;
    int8_t sel_vco;
    bool canDeliverFrequency = false;
    uint16_t integerPart;
    uint32_t fractionalPart;
    int16_t csw_value;

    //find required VCO frequency
    for (div_loch = 6; div_loch >= 0; --div_loch)
    {
        VCOfreq = (1 << (div_loch + 1)) * freq_Hz;
        if ((VCOfreq >= gVCO_frequency_table[0][0]) && (VCOfreq <= gVCO_frequency_table[2][sxVCO_N - 1]))
        {
            canDeliverFrequency = true;
            break;
        }
    }
    if (canDeliverFrequency == false)
        return ReportError(OpStatus::OutOfRange,
            "SetFrequencySX%s(%g MHz) - required VCO frequencies are out of range [%g-%g] MHz",
            dir == TRXDir::Tx ? "T" : "R",
            freq_Hz / 1e6,
            gVCO_frequency_table[0][0] / 1e6,
            gVCO_frequency_table[2][sxVCO_N - 1] / 1e6);

    const float_type refClk_Hz = GetReferenceClk_SX(dir);
    assert(refClk_Hz > 0);
    double divider = refClk_Hz * (1 + (VCOfreq > m_dThrF));
    integerPart = static_cast<uint16_t>(VCOfreq / divider - 4);
    fractionalPart = static_cast<uint32_t>((VCOfreq / divider - static_cast<uint32_t>(VCOfreq / divider)) * 1048576);

    ChannelScope scope(this);
    this->SetActiveChannel(dir == TRXDir::Tx ? Channel::ChSXT : Channel::ChSXR);

    Modify_SPI_Reg_bits(LMS7002MCSR::EN_INTONLY_SDM, 0);
    Modify_SPI_Reg_bits(LMS7002MCSR::INT_SDM, integerPart); //INT_SDM
    Modify_SPI_Reg_bits(0x011D, 15, 0, fractionalPart & 0xFFFF); //FRAC_SDM[15:0]
    Modify_SPI_Reg_bits(0x011E, 3, 0, (fractionalPart >> 16)); //FRAC_SDM[19:16]
    Modify_SPI_Reg_bits(LMS7002MCSR::DIV_LOCH, div_loch); //DIV_LOCH
    Modify_SPI_Reg_bits(LMS7002MCSR::EN_DIV2_DIVPROG, (VCOfreq > m_dThrF)); //EN_DIV2_DIVPROG

    lime::info("SetFrequencySX%s, (%.3f MHz)INT %d, FRAC %d, DIV_LOCH %d, EN_DIV2_DIVPROG %d",
        dir == TRXDir::Tx ? "T" : "R",
        freq_Hz / 1e6,
        integerPart,
        fractionalPart,
        div_loch,
        (VCOfreq > m_dThrF));
    lime::debug("Expected VCO %.2f MHz, RefClk %.2f MHz", VCOfreq / 1e6, refClk_Hz / 1e6);

    if (output)
    {
        output->frequency = freq_Hz;
        output->frequencyVCO = VCOfreq;
        output->referenceClock = GetReferenceClk_SX(dir);
        output->INT = integerPart;
        output->FRAC = fractionalPart;
        output->en_div2_divprog = (VCOfreq > m_dThrF);
        output->div_loch = div_loch;
    }

    // turn on VCO and comparator
    Modify_SPI_Reg_bits(LMS7002MCSR::PD_VCO, 0); //
    Modify_SPI_Reg_bits(LMS7002MCSR::PD_VCO_COMP, 0);

    // try setting tuning values from the cache, if it fails perform full tuning
    if (useCache && tuning_cache_sel_vco.count(freq_Hz) > 0)
    {
        sel_vco = tuning_cache_sel_vco[freq_Hz];
        csw_value = tuning_cache_csw_value[freq_Hz];
        Modify_SPI_Reg_bits(LMS7002MCSR::SEL_VCO, sel_vco);
        Modify_SPI_Reg_bits(CSW_VCO.address, CSW_VCO.msb, CSW_VCO.lsb, csw_value);
        // probably no need for this as the interface is already very slow..
        std::this_thread::sleep_for(std::chrono::microseconds(50));
        auto cmphl = static_cast<uint8_t>(Get_SPI_Reg_bits(VCO_CMPHO.address, 13, 12, true));
        if (cmphl == 2)
        {
            lime::info("Fast Tune success; vco=%d value=%d", tuning_cache_sel_vco[freq_Hz], tuning_cache_csw_value[freq_Hz]);
            if (output)
            {
                output->success = true;
                output->sel_vco = sel_vco;
                output->csw = csw_value;
            }
            return OpStatus::Success;
        }
    }

    canDeliverFrequency = false;
    int tuneScore[] = { -128, -128, -128 }; //best is closest to 0
    for (int i = 0; i < 5; i++) //attempt tune multiple times
    {
        for (sel_vco = 0; sel_vco < 3; ++sel_vco)
        {
            lime::debug("Tuning %s :", vcoNames[sel_vco]);
            Modify_SPI_Reg_bits(LMS7002MCSR::SEL_VCO, sel_vco);
            OpStatus status = TuneVCO(dir == TRXDir::Tx ? VCO_Module::VCO_SXT : VCO_Module::VCO_SXR);
            if (status == OpStatus::Success)
            {
                tuneScore[sel_vco] = -128 + Get_SPI_Reg_bits(LMS7002MCSR::CSW_VCO, true);
                canDeliverFrequency = true;
                lime::debug("%s : csw=%d %s",
                    vcoNames[sel_vco],
                    tuneScore[sel_vco] + 128,
                    (status == OpStatus::Success ? "tune ok" : "tune fail"));
            }
            else
            {
                lime::debug("%s : failed to lock", vcoNames[sel_vco]);
            }
        }
        if (canDeliverFrequency) //tune OK
            break;
        auto bias = Get_SPI_Reg_bits(LMS7002MCSR::ICT_VCO);
        if (bias == 255)
            break;
        bias = bias + 32 > 255 ? 255 : bias + 32; //retry with higher bias current
        Modify_SPI_Reg_bits(LMS7002MCSR::ICT_VCO, bias);
    }

    if (abs(tuneScore[0]) < abs(tuneScore[1]))
    {
        if (abs(tuneScore[0]) < abs(tuneScore[2]))
            sel_vco = 0;
        else
            sel_vco = 2;
    }
    else
    {
        if (abs(tuneScore[1]) < abs(tuneScore[2]))
            sel_vco = 1;
        else
            sel_vco = 2;
    }
    csw_value = tuneScore[sel_vco] + 128;
    lime::debug("Selected: %s, CSW_VCO: %i", vcoNames[sel_vco], csw_value);

    if (output)
    {
        if (canDeliverFrequency)
            output->success = true;
        output->sel_vco = sel_vco;
        output->csw = csw_value;
    }
    Modify_SPI_Reg_bits(LMS7002MCSR::SEL_VCO, sel_vco);
    Modify_SPI_Reg_bits(LMS7002MCSR::CSW_VCO, csw_value);

    // save successful tuning results in cache
    if (useCache && canDeliverFrequency)
    {
        tuning_cache_sel_vco[freq_Hz] = sel_vco;
        tuning_cache_csw_value[freq_Hz] = csw_value;
    }

    if (canDeliverFrequency == false)
        return ReportError(
            OpStatus::Error, "SetFrequencySX%s(%g MHz) - cannot deliver frequency", dir == TRXDir::Tx ? "T" : "R", freq_Hz / 1e6);
    return OpStatus::Success;
}

OpStatus LMS7002M::SetFrequencySXWithSpurCancelation(TRXDir dir, float_type freq_Hz, float_type BW)
{
    const float BWOffset = 2e6;
    BW += BWOffset; //offset to avoid ref clock on BW edge
    bool needCancelation = false;
    float_type refClk = GetReferenceClk_SX(TRXDir::Rx);
    int low = (freq_Hz - BW / 2) / refClk;
    int high = (freq_Hz + BW / 2) / refClk;
    if (low != high)
        needCancelation = true;

    OpStatus status;
    float newFreq(0);
    if (needCancelation)
    {
        newFreq = static_cast<int>(freq_Hz / refClk + 0.5) * refClk;
        TuneRxFilter(BW - BWOffset + 2 * abs(freq_Hz - newFreq));
        status = SetFrequencySX(dir, newFreq);
    }
    else
        status = SetFrequencySX(dir, freq_Hz);
    if (status != OpStatus::Success)
        return status;
    const int ch = Get_SPI_Reg_bits(LMS7002MCSR::MAC);
    for (int i = 0; i < 2; ++i)
    {
        Modify_SPI_Reg_bits(LMS7002MCSR::MAC, i + 1);
        SetNCOFrequency(TRXDir::Rx, 15, 0);
    }
    if (needCancelation)
    {
        Modify_SPI_Reg_bits(LMS7002MCSR::MAC, ch);
        Modify_SPI_Reg_bits(LMS7002MCSR::EN_INTONLY_SDM, 1);

        /*uint16_t gINT = Get_SPI_Reg_bits(0x011E, 13, 0);	// read whole register to reduce SPI transfers
        uint32_t gFRAC = ((gINT&0xF) * 65536) | Get_SPI_Reg_bits(0x011D, 15, 0);
        bool upconvert = gFRAC > (1 << 19);
        gINT = gINT >> 4;
        if(upconvert)
        {
            gINT+=;
            Modify_SPI_Reg_bits(LMS7002MCSR::INT_SDM, gINT);
        }
        Modify_SPI_Reg_bits(0x011D, 15, 0, 0);
        Modify_SPI_Reg_bits(0x011E, 3, 0, 0);*/
        //const float_type refClk_Hz = GetReferenceClk_SX(dir);
        //float actualFreq = (float_type)refClk_Hz / (1 << (Get_SPI_Reg_bits(LMS7002MCSR::DIV_LOCH) + 1));
        //actualFreq *= (gINT + 4) * (Get_SPI_Reg_bits(LMS7002MCSR::EN_DIV2_DIVPROG) + 1);
        float actualFreq = newFreq;
        float userFreq = freq_Hz;
        bool upconvert = actualFreq > userFreq;
        for (int i = 0; i < 2; ++i)
        {
            Modify_SPI_Reg_bits(LMS7002MCSR::MAC, i + 1);
            Modify_SPI_Reg_bits(LMS7002MCSR::CMIX_SC_RXTSP, !upconvert);
            Modify_SPI_Reg_bits(LMS7002MCSR::CMIX_BYP_RXTSP, 0);
            Modify_SPI_Reg_bits(LMS7002MCSR::SEL_RX, 15);
            Modify_SPI_Reg_bits(LMS7002MCSR::CMIX_GAIN_RXTSP, 1);
            SetNCOFrequency(TRXDir::Rx, 14, 0);
            SetNCOFrequency(TRXDir::Rx, 15, abs(actualFreq - userFreq));
        }
    }

    Modify_SPI_Reg_bits(LMS7002MCSR::MAC, ch);
    return OpStatus::Success;
}

float_type LMS7002M::GetFrequencySX(TRXDir dir)
{
    ChannelScope(this, dir == TRXDir::Tx ? Channel::ChSXT : Channel::ChSXR);

    float_type dMul;
    uint16_t gINT = Get_SPI_Reg_bits(0x011E, 13, 0); // read whole register to reduce SPI transfers
    uint32_t gFRAC = ((gINT & 0xF) * 65536) | Get_SPI_Reg_bits(0x011D, 15, 0);

    const float_type refClk_Hz = GetReferenceClk_SX(dir);
    dMul = refClk_Hz / (1 << (Get_SPI_Reg_bits(LMS7002MCSR::DIV_LOCH) + 1));
    //Calculate real frequency according to the calculated parameters
    dMul = dMul * ((gINT >> 4) + 4 + gFRAC / 1048576.0) * (Get_SPI_Reg_bits(LMS7002MCSR::EN_DIV2_DIVPROG) + 1);
    return dMul;
}

OpStatus LMS7002M::SetNCOFrequency(TRXDir dir, uint8_t index, float_type freq_Hz)
{
    if (index > 15)
        return ReportError(OpStatus::InvalidValue, "SetNCOFrequency(index = %d) - index out of range [0, 15]", index);
    float_type refClk_Hz = GetReferenceClk_TSP(dir);
    if (freq_Hz < 0 || freq_Hz / refClk_Hz > 0.5)
        return ReportError(OpStatus::OutOfRange,
            "SetNCOFrequency(index = %d) - Frequency(%g MHz) out of range [0-%g) MHz",
            index,
            freq_Hz / 1e6,
            refClk_Hz / 2e6);
    uint16_t addr = dir == TRXDir::Tx ? 0x0240 : 0x0440;
    uint32_t fcw = static_cast<uint32_t>((freq_Hz / refClk_Hz) * 4294967296);
    SPI_write(addr + 2 + index * 2, (fcw >> 16)); //NCO frequency control word register MSB part.
    SPI_write(addr + 3 + index * 2, fcw); //NCO frequency control word register LSB part.
    return OpStatus::Success;
}

float_type LMS7002M::GetNCOFrequency(TRXDir dir, uint8_t index, bool fromChip)
{
    if (index > 15)
    {
        ReportError(OpStatus::InvalidValue, "GetNCOFrequency_MHz(index = %d) - index out of range [0, 15]", index);
        return 0;
    }
    float_type refClk_Hz = GetReferenceClk_TSP(dir);
    uint16_t addr = dir == TRXDir::Tx ? 0x0240 : 0x0440;
    uint32_t fcw = 0;
    fcw |= SPI_read(addr + 2 + index * 2, fromChip) << 16; //NCO frequency control word register MSB part.
    fcw |= SPI_read(addr + 3 + index * 2, fromChip); //NCO frequency control word register LSB part.
    return refClk_Hz * (fcw / 4294967296.0);
}

OpStatus LMS7002M::SetNCOPhaseOffsetForMode0(TRXDir dir, float_type angle_deg)
{
    uint16_t addr = dir == TRXDir::Tx ? 0x0241 : 0x0441;
    uint16_t pho = static_cast<uint16_t>(65536 * (angle_deg / 360));
    SPI_write(addr, pho);
    return OpStatus::Success;
}

OpStatus LMS7002M::SetNCOPhaseOffset(TRXDir dir, uint8_t index, float_type angle_deg)
{
    if (index > 15)
        return ReportError(OpStatus::InvalidValue, "SetNCOPhaseOffset(index = %d) - index out of range [0, 15]", index);
    uint16_t addr = dir == TRXDir::Tx ? 0x0244 : 0x0444;
    uint16_t pho = static_cast<uint16_t>(65536 * (angle_deg / 360));
    SPI_write(addr + index, pho);
    return OpStatus::Success;
}

OpStatus LMS7002M::SetNCOPhases(TRXDir dir, const float_type* angles_deg, uint8_t count, float_type frequencyOffset)
{
    OpStatus status = SetNCOFrequency(dir, 0, frequencyOffset);
    if (status != OpStatus::Success)
        return status;

    if (angles_deg == nullptr)
    {
        return OpStatus::Success;
    }

    for (uint8_t i = 0; i < 16 && i < count; i++)
    {
        status = SetNCOPhaseOffset(dir, i, angles_deg[i]);
        if (status != OpStatus::Success)
            return status;
    }
    return Modify_SPI_Reg_bits(dir == TRXDir::Tx ? SEL_TX : SEL_RX, 0);
}

std::vector<float_type> LMS7002M::GetNCOPhases(TRXDir dir, float_type* frequencyOffset)
{
    std::vector<float_type> angles_deg;
    return angles_deg;
}

float_type LMS7002M::GetNCOPhaseOffset_Deg(TRXDir dir, uint8_t index)
{
    if (index > 15)
    {
        ReportError(OpStatus::InvalidValue, "GetNCOPhaseOffset_Deg(index = %d) - index out of range [0, 15]", index);
        return 0;
    }
    uint16_t addr = dir == TRXDir::Tx ? 0x0244 : 0x0444;
    uint16_t pho = SPI_read(addr + index);
    float_type angle = 360 * pho / 65536.0;
    return angle;
}

OpStatus LMS7002M::SetGFIRCoefficients(TRXDir dir, uint8_t gfirIndex, const float_type* coef, uint8_t coefCount)
{
    if (gfirIndex > 2)
    {
        lime::warning("SetGFIRCoefficients: Invalid GFIR index(%i). Will configure GFIR[2].", gfirIndex);
        gfirIndex = 2;
    }

    const uint16_t startAddr = 0x0280 + (gfirIndex * 0x40) + (dir == TRXDir::Tx ? 0 : 0x0200);
    const uint8_t maxCoefCount = gfirIndex < 2 ? 40 : 120;
    const uint8_t bankCount = gfirIndex < 2 ? 5 : 15;

    if (coefCount > maxCoefCount)
    {
        return ReportError(OpStatus::OutOfRange,
            "SetGFIRCoefficients: too many coefficients(%i), GFIR[%i] can have only %i",
            coefCount,
            gfirIndex,
            maxCoefCount);
    }

    uint16_t addrs[120];
    int16_t words[120];
    // actual used coefficients count is multiple of 'bankCount'
    // if coefCount is not multiple, extra '0' coefficients will be written
    const uint8_t bankLength = std::ceil(static_cast<float>(coefCount) / bankCount);
    const int16_t actualCoefCount = bankLength * bankCount;
    assert(actualCoefCount <= maxCoefCount);

    for (int i = 0; i < actualCoefCount; ++i)
    {
        uint8_t bank = i / bankLength;
        uint8_t bankRow = i % bankLength;
        addrs[i] = startAddr + (bank * 8) + bankRow;
        addrs[i] += 24 * (bank / 5);

        if (i < coefCount)
        {
            words[i] = coef[i] * 32767;

            if (coef[i] < -1 || coef[i] > 1)
            {
                lime::warning("Coefficient %f is outside of range [-1:1], incorrect value will be written.", coef[i]);
            }
        }
        else
        {
            words[i] = 0;
        }
    }
    CSRegister gfirL_param = GFIR1_L_TXTSP;
    gfirL_param.address += gfirIndex + (dir == TRXDir::Tx ? 0 : 0x0200);
    Modify_SPI_Reg_bits(gfirL_param, bankLength - 1);

    return SPI_write_batch(addrs, reinterpret_cast<const uint16_t*>(words), actualCoefCount, true);
}

OpStatus LMS7002M::GetGFIRCoefficients(TRXDir dir, uint8_t gfirIndex, float_type* coef, uint8_t coefCount)
{
    OpStatus status = OpStatus::Error;

    if (gfirIndex > 2)
    {
        lime::warning("GetGFIRCoefficients: Invalid GFIR index(%i). Will read GFIR[2].", gfirIndex);
        gfirIndex = 2;
    }

    const uint16_t startAddr = 0x0280 + (gfirIndex * 0x40) + (dir == TRXDir::Tx ? 0 : 0x0200);
    const uint8_t coefLimit = gfirIndex < 2 ? 40 : 120;

    if (coefCount > coefLimit)
    {
        return ReportError(OpStatus::OutOfRange, "GetGFIRCoefficients(coefCount=%d) - exceeds coefLimit=%d", coefCount, coefLimit);
    }

    std::vector<uint16_t> addresses;
    for (uint8_t index = 0; index < coefCount; ++index)
    {
        addresses.push_back(startAddr + index + 24 * (index / 40));
    }

    int16_t spiData[120];
    std::memset(spiData, 0, 120 * sizeof(int16_t));
    if (controlPort)
    {
        status = SPI_read_batch(&addresses[0], reinterpret_cast<uint16_t*>(spiData), coefCount);
        for (uint8_t index = 0; index < coefCount; ++index)
        {
            coef[index] = spiData[index] / 32768.0;
        }
    }
    else
    {
        const int channel = Get_SPI_Reg_bits(LMS7002MCSR::MAC, false) > 1 ? 1 : 0;
        for (uint8_t index = 0; index < coefCount; ++index)
        {
            uint16_t value = mRegistersMap->GetValue(channel, addresses[index]);
            coef[index] = *reinterpret_cast<int16_t*>(&value) / 32768.0;
        }
        status = OpStatus::Success;
    }

    return status;
}

OpStatus LMS7002M::SPI_write(uint16_t address, uint16_t data, bool toChip)
{
    if (address != 0x0640 && address != 0x0641)
    {
        return SPI_write_batch(&address, &data, 1, toChip);
    }

    MCU_BD* mcu = GetMCUControls();
    mcu->RunProcedure(MCU_FUNCTION_GET_PROGRAM_ID);
    if (mcu->WaitForMCU(100) != MCU_ID_CALIBRATIONS_SINGLE_IMAGE)
        mcu->Program_MCU(mcu_program_lms7_dc_iq_calibration_bin, MCU_BD::MCU_PROG_MODE::SRAM);
    SPI_write(0x002D, address);
    SPI_write(0x020C, data);
    mcu->RunProcedure(7);
    mcu->WaitForMCU(50);
    return SPI_read(0x040B) == data ? OpStatus::Success : OpStatus::Error;
}

uint16_t LMS7002M::SPI_read(uint16_t address, bool fromChip, OpStatus* status)
{
    fromChip |= !useCache;
    //registers containing read only registers, which values can change
    static const std::unordered_set<uint16_t> volatileRegs = {
        0x0000,
        0x0001,
        0x0002,
        0x0003,
        0x0004,
        0x0005,
        0x0006,
        0x002F,
        0x008C,
        0x00A8,
        0x00A9,
        0x00AA,
        0x00AB,
        0x00AC,
        0x0123,
        0x0209,
        0x020A,
        0x020B,
        0x040E,
        0x040F,
        0x05C3,
        0x05C4,
        0x05C5,
        0x05C6,
        0x05C7,
        0x05C8,
        0x05C9,
        0x05CA,
    };
    if (volatileRegs.find(address) != volatileRegs.end())
        fromChip = true;

    if (!controlPort || fromChip == false)
    {
        if (status && !controlPort)
            *status = ReportError(OpStatus::IOFailure, "chip not connected");
        uint8_t mac = mRegistersMap->GetValue(0, MAC.address) & 0x0003;
        uint8_t channel = (mac == 2) ? 1 : 0; //only when MAC is B -> use register space B
        if (address < 0x0100)
            channel = 0; //force A when below MAC mapped register space
        return mRegistersMap->GetValue(channel, address);
    }
    if (!controlPort)
    {
        return 0;
    }

    uint16_t data = 0;
    OpStatus st;
    if (address == 0x0640 || address == 0x0641)
    {
        MCU_BD* mcu = GetMCUControls();
        mcu->RunProcedure(MCU_FUNCTION_GET_PROGRAM_ID);
        if (mcu->WaitForMCU(100) != MCU_ID_CALIBRATIONS_SINGLE_IMAGE)
            mcu->Program_MCU(mcu_program_lms7_dc_iq_calibration_bin, MCU_BD::MCU_PROG_MODE::SRAM);
        SPI_write(0x002D, address);
        mcu->RunProcedure(8);
        mcu->WaitForMCU(50);
        uint16_t rdVal = SPI_read(0x040B, true, status);
        return rdVal;
    }
    else
        st = this->SPI_read_batch(&address, &data, 1);
    if (status != nullptr)
        *status = st;
    return data;
}

OpStatus LMS7002M::SPI_write_batch(const uint16_t* spiAddr, const uint16_t* spiData, uint16_t cnt, bool toChip)
{
    toChip |= !useCache;
    int mac = mRegistersMap->GetValue(0, MAC.address) & 0x0003;
    std::vector<uint32_t> data;
    for (size_t i = 0; i < cnt; ++i)
    {
        //write which register cache based on MAC bits
        //or always when below the MAC mapped register space
        bool wr0 = ((mac & 0x1) != 0) || (spiAddr[i] < 0x0100);
        bool wr1 = ((mac & 0x2) != 0) && (spiAddr[i] >= 0x0100);

        if (!toChip)
        {
            if (wr0 && (mRegistersMap->GetValue(0, spiAddr[i]) == spiData[i]))
                wr0 = false;
            if (wr1 && (mRegistersMap->GetValue(1, spiAddr[i]) == spiData[i]))
                wr1 = false;
            if (!(wr0 || wr1))
                continue;
        }

        data.push_back((1 << 31) | (static_cast<uint32_t>(spiAddr[i]) << 16) | spiData[i]); //msbit 1=SPI write
        if (wr0)
            mRegistersMap->SetValue(0, spiAddr[i], spiData[i]);
        if (wr1)
            mRegistersMap->SetValue(1, spiAddr[i], spiData[i]);

        //refresh mac, because batch might also change active channel
        if (spiAddr[i] == MAC.address)
            mac = mRegistersMap->GetValue(0, MAC.address) & 0x0003;
    }

    if (data.size() == 0)
        return OpStatus::Success;
    if (!controlPort)
    {
        if (useCache)
            return OpStatus::Success;
        return ReportError(OpStatus::IOFailure, "No device connected"s);
    }
    controlPort->SPI(data.data(), nullptr, data.size());
    return OpStatus::Success;
}

OpStatus LMS7002M::SPI_read_batch(const uint16_t* spiAddr, uint16_t* spiData, uint16_t cnt)
{
    if (!controlPort)
    {
        return ReportError(OpStatus::IOFailure, "No device connected"s);
    }

    std::vector<uint32_t> dataWr(cnt);
    std::vector<uint32_t> dataRd(cnt);
    for (size_t i = 0; i < cnt; ++i)
    {
        dataWr[i] = spiAddr[i];
    }

    controlPort->SPI(dataWr.data(), dataRd.data(), cnt);

    int mac = mRegistersMap->GetValue(0, MAC.address) & 0x0003;

    for (size_t i = 0; i < cnt; ++i)
    {
        spiData[i] = dataRd[i] & 0xffff;

        //write which register cache based on MAC bits
        //or always when below the MAC mapped register space
        bool wr0 = ((mac & 0x1) != 0) or (spiAddr[i] < 0x0100);
        bool wr1 = ((mac & 0x2) != 0) and (spiAddr[i] >= 0x0100);

        if (wr0)
            mRegistersMap->SetValue(0, spiAddr[i], spiData[i]);
        if (wr1)
            mRegistersMap->SetValue(1, spiAddr[i], spiData[i]);
    }
    return OpStatus::Success;
}

OpStatus LMS7002M::RegistersTest(const std::string& fileName)
{
    char chex[16];
    if (!controlPort)
        return ReportError(OpStatus::IOFailure, "No device connected"s);

    OpStatus status;
    ChannelScope scope(this);

    //backup both channel data for restoration after test
    std::vector<uint16_t> ch1Addresses;
    for (const auto& memorySectionPair : MemorySectionAddresses)
        for (uint16_t addr = memorySectionPair.second[0]; addr <= memorySectionPair.second[1]; ++addr)
            ch1Addresses.push_back(addr);
    std::vector<uint16_t> ch1Data;
    ch1Data.resize(ch1Addresses.size(), 0);

    //backup A channel
    this->SetActiveChannel(Channel::ChA);
    status = SPI_read_batch(&ch1Addresses[0], &ch1Data[0], ch1Addresses.size());
    if (status != OpStatus::Success)
        return status;

    std::vector<uint16_t> ch2Addresses;
    for (const auto& memorySectionPair : MemorySectionAddresses)
        for (uint16_t addr = memorySectionPair.second[0]; addr <= memorySectionPair.second[1]; ++addr)
            if (addr >= 0x0100)
                ch2Addresses.push_back(addr);
    std::vector<uint16_t> ch2Data;
    ch2Data.resize(ch2Addresses.size(), 0);

    this->SetActiveChannel(Channel::ChB);
    status = SPI_read_batch(&ch2Addresses[0], &ch2Data[0], ch2Addresses.size());
    if (status != OpStatus::Success)
        return status;

    //test registers
    ResetChip();
    Modify_SPI_Reg_bits(LMS7002MCSR::MIMO_SISO, 0);
    Modify_SPI_Reg_bits(LMS7002MCSR::PD_RX_AFE2, 0);
    Modify_SPI_Reg_bits(LMS7002MCSR::PD_TX_AFE2, 0);
    this->SetActiveChannel(Channel::ChA);

    std::stringstream ss;

    //check single channel memory sections
    std::vector<MemorySection> modulesToCheck = {
        MemorySection::AFE,
        MemorySection::BIAS,
        MemorySection::XBUF,
        MemorySection::CGEN,
        MemorySection::BIST,
        MemorySection::CDS,
        MemorySection::TRF,
        MemorySection::TBB,
        MemorySection::RFE,
        MemorySection::RBB,
        MemorySection::SX,
        MemorySection::TxTSP,
        MemorySection::TxNCO,
        MemorySection::TxGFIR1,
        MemorySection::TxGFIR2,
        MemorySection::TxGFIR3a,
        MemorySection::TxGFIR3b,
        MemorySection::TxGFIR3c,
        MemorySection::RxTSP,
        MemorySection::RxNCO,
        MemorySection::RxGFIR1,
        MemorySection::RxGFIR2,
        MemorySection::RxGFIR3a,
        MemorySection::RxGFIR3b,
        MemorySection::RxGFIR3c,
        MemorySection::LimeLight,
        MemorySection::LDO,
    };

    const std::array<std::string_view, 27> moduleNames = {
        "AFE"sv,
        "BIAS"sv,
        "XBUF"sv,
        "CGEN"sv,
        "BIST"sv,
        "CDS"sv,
        "TRF"sv,
        "TBB"sv,
        "RFE"sv,
        "RBB"sv,
        "SX"sv,
        "TxTSP"sv,
        "TxNCO"sv,
        "TxGFIR1"sv,
        "TxGFIR2"sv,
        "TxGFIR3a"sv,
        "TxGFIR3b"sv,
        "TxGFIR3c"sv,
        "RxTSP"sv,
        "RxNCO"sv,
        "RxGFIR1"sv,
        "RxGFIR2"sv,
        "RxGFIR3a"sv,
        "RxGFIR3b"sv,
        "RxGFIR3c"sv,
        "LimeLight"sv,
        "LDO"sv,
    };

    const uint16_t patterns[] = { 0xAAAA, 0x5555 };
    const uint8_t patternsCount = 2;

    bool allTestSuccess = true;

    for (unsigned i = 0; i < modulesToCheck.size(); ++i)
    {
        bool moduleTestsSuccess = true;
        uint16_t startAddr = MemorySectionAddresses.at(modulesToCheck[i]).at(0);
        uint16_t endAddr = MemorySectionAddresses.at(modulesToCheck[i]).at(1);
        uint8_t channelCount = startAddr >= 0x0100 ? 2 : 1;
        for (int cc = 1; cc <= channelCount; ++cc)
        {
            Modify_SPI_Reg_bits(LMS7002MCSR::MAC, cc);
            std::snprintf(chex, sizeof(chex), "0x%04X", startAddr);
            ss << moduleNames[i] << "  ["sv << chex << ":"sv;
            std::snprintf(chex, sizeof(chex), "0x%04X", endAddr);
            ss << chex << "]"sv;
            if (startAddr >= 0x0100)
            {
                ss << " Ch."sv << (cc == 1 ? "A"sv : "B"sv);
            }
            ss << std::endl;
            for (uint8_t p = 0; p < patternsCount; ++p)
                moduleTestsSuccess &= RegistersTestInterval(startAddr, endAddr, patterns[p], ss) == OpStatus::Success;
        }
        allTestSuccess &= moduleTestsSuccess;
    }

    //restore register values
    this->SetActiveChannel(Channel::ChA);
    SPI_write_batch(&ch1Addresses[0], &ch1Data[0], ch1Addresses.size(), true);
    this->SetActiveChannel(Channel::ChB);
    SPI_write_batch(&ch2Addresses[0], &ch2Data[0], ch2Addresses.size(), true);

    if (!fileName.empty())
    {
        std::fstream fout;
        fout.open(fileName, std::ios::out);
        fout << ss.str() << std::endl;
        fout.close();
    }

    if (allTestSuccess)
        return OpStatus::Success;
    return ReportError(OpStatus::Error, "RegistersTest() failed"s);
}

/** @brief Performs registers test for given address interval by writing given pattern data
    @param startAddr first register address
    @param endAddr last reigster address
    @param pattern data to be written into registers
    @param ss stringstream to use
    @return 0-register test passed, other-failure
*/
OpStatus LMS7002M::RegistersTestInterval(uint16_t startAddr, uint16_t endAddr, uint16_t pattern, std::stringstream& ss)
{
    std::vector<uint16_t> addrToWrite;
    std::vector<uint16_t> dataToWrite;
    std::vector<uint16_t> dataReceived;
    std::vector<uint16_t> dataMasks;

    for (uint16_t addr = startAddr; addr <= endAddr; ++addr)
    {
        addrToWrite.push_back(addr);
    }
    dataMasks.resize(addrToWrite.size(), 0xFFFF);
    for (std::size_t j = 0; j < readOnlyRegisters.size(); ++j)
    {
        for (std::size_t k = 0; k < addrToWrite.size(); ++k)
        {
            if (readOnlyRegisters[j].address == addrToWrite[k])
            {
                dataMasks[k] = readOnlyRegisters[j].mask;
                break;
            }
        }
    }

    dataToWrite.clear();
    dataReceived.clear();
    for (uint16_t j = 0; j < addrToWrite.size(); ++j)
    {
        if (addrToWrite[j] == 0x00A6)
            dataToWrite.push_back(0x1 | (pattern & ~0x2));
        else if (addrToWrite[j] == 0x0084)
            dataToWrite.push_back(pattern & ~0x19);
        else
            dataToWrite.push_back(pattern & dataMasks[j]);
    }
    dataReceived.resize(addrToWrite.size(), 0);
    OpStatus status;
    status = SPI_write_batch(&addrToWrite[0], &dataToWrite[0], addrToWrite.size(), true);
    if (status != OpStatus::Success)
        return status;
    status = SPI_read_batch(&addrToWrite[0], &dataReceived[0], addrToWrite.size());
    if (status != OpStatus::Success)
        return status;
    bool registersMatch = true;
    char ctemp[16];
    for (uint16_t j = 0; j < dataToWrite.size(); ++j)
    {
        if (dataToWrite[j] != (dataReceived[j] & dataMasks[j]))
        {
            registersMatch = false;
            std::snprintf(ctemp, sizeof(ctemp), "0x%04X", addrToWrite[j]);
            ss << "\t"sv << ctemp << "(wr/rd): "sv;
            std::snprintf(ctemp, sizeof(ctemp), "0x%04X", dataToWrite[j]);
            ss << ctemp << "/"sv;
            std::snprintf(ctemp, sizeof(ctemp), "0x%04X", dataReceived[j]);
            ss << ctemp << std::endl;
        }
    }
    if (registersMatch)
    {
        std::snprintf(ctemp, sizeof(ctemp), "0x%04X", pattern);
        ss << "\tRegisters OK ("sv << ctemp << ")\n"sv;
    }
    if (registersMatch)
        return OpStatus::Success;
    return ReportError(OpStatus::Error, "RegistersTestInterval(startAddr=0x%x, endAddr=0x%x) - failed", startAddr, endAddr);
}

/** @brief Sets Rx Dc offsets by converting two's complementary numbers to sign and magnitude
*/
void LMS7002M::SetRxDCOFF(int8_t offsetI, int8_t offsetQ)
{
    uint16_t valToSend = 0;
    if (offsetI < 0)
        valToSend |= 0x40;
    valToSend |= labs(offsetI);
    valToSend = valToSend << 7;
    if (offsetQ < 0)
        valToSend |= 0x40;
    valToSend |= labs(offsetQ);
    SPI_write(0x010E, valToSend);
}

OpStatus LMS7002M::SetDefaults(MemorySection module)
{
    OpStatus status;
    std::vector<uint16_t> addrs;
    std::vector<uint16_t> values;
    for (uint16_t address = MemorySectionAddresses.at(module).at(0); address <= MemorySectionAddresses.at(module).at(1); ++address)
    {
        addrs.push_back(address);
        values.push_back(mRegistersMap->GetDefaultValue(address));
    }
    status = SPI_write_batch(&addrs[0], &values[0], addrs.size());
    return status;
}

void LMS7002M::ModifyRegistersDefaults(const std::vector<std::pair<uint16_t, uint16_t>>& registerValues)
{
    for (const auto& addrValuePair : registerValues)
        mRegistersMap->SetDefaultValue(addrValuePair.first, addrValuePair.second);
}

bool LMS7002M::IsSynced()
{
    if (!controlPort)
        return false;

    ChannelScope scope(this);

    std::vector<uint16_t> addrToRead = mRegistersMap->GetUsedAddresses(0);
    std::vector<uint16_t> dataReceived;
    dataReceived.resize(addrToRead.size(), 0);

    this->SetActiveChannel(Channel::ChA);
    std::vector<uint32_t> dataWr(addrToRead.size());
    std::vector<uint32_t> dataRd(addrToRead.size());
    for (size_t i = 0; i < addrToRead.size(); ++i)
        dataWr[i] = (static_cast<uint32_t>(addrToRead[i]) << 16);
    controlPort->SPI(dataWr.data(), dataRd.data(), dataWr.size());

    for (size_t i = 0; i < addrToRead.size(); ++i)
        dataReceived[i] = dataRd[i] & 0xFFFF;

    //check if local copy matches chip
    for (uint16_t i = 0; i < addrToRead.size(); ++i)
    {
        uint16_t regValue = mRegistersMap->GetValue(0, addrToRead[i]);
        if (addrToRead[i] <= readOnlyRegisters[readOnlyRegisters.size() - 1].address &&
            addrToRead[i] >= readOnlyRegisters[0].address)
        {
            //mask out readonly bits
            for (std::size_t j = 0; j < readOnlyRegisters.size(); ++j)
            {
                if (readOnlyRegisters[j].address == addrToRead[i])
                {
                    dataReceived[i] &= readOnlyRegisters[j].mask;
                    regValue &= readOnlyRegisters[j].mask;
                    break;
                }
            }
        }
        if (dataReceived[i] != regValue)
        {
            lime::debug("Addr: 0x%04X  gui: 0x%04X  chip: 0x%04X", addrToRead[i], regValue, dataReceived[i]);
            return false;
        }
    }

    addrToRead.clear(); //add only B channel addresses
    addrToRead = mRegistersMap->GetUsedAddresses(1);
    dataWr.resize(addrToRead.size());
    dataRd.resize(addrToRead.size());
    for (size_t i = 0; i < addrToRead.size(); ++i)
        dataWr[i] = (static_cast<uint32_t>(addrToRead[i]) << 16);
    controlPort->SPI(dataWr.data(), dataRd.data(), dataWr.size());
    for (size_t i = 0; i < addrToRead.size(); ++i)
        dataReceived[i] = dataRd[i] & 0xFFFF;
    this->SetActiveChannel(Channel::ChB);

    //check if local copy matches chip
    for (uint16_t i = 0; i < addrToRead.size(); ++i)
    {
        uint16_t regValue = mRegistersMap->GetValue(1, addrToRead[i]);
        if (addrToRead[i] <= readOnlyRegisters[readOnlyRegisters.size() - 1].address &&
            addrToRead[i] >= readOnlyRegisters[0].address)
        {
            //mask out readonly bits
            for (std::size_t j = 0; j < readOnlyRegisters.size(); ++j)
            {
                if (readOnlyRegisters[j].address == addrToRead[i])
                {
                    dataReceived[i] &= readOnlyRegisters[j].mask;
                    regValue &= readOnlyRegisters[j].mask;
                    break;
                }
            }
        }
        if (dataReceived[i] != regValue)
        {
            lime::debug("Addr: 0x%04X  gui: 0x%04X  chip: 0x%04X", addrToRead[i], regValue, dataReceived[i]);
            return false;
        }
    }

    return true;
}

OpStatus LMS7002M::UploadAll()
{
    if (!controlPort)
        return ReportError(OpStatus::IOFailure, "No device connected"s);

    ChannelScope scope(this);

    OpStatus status;

    std::vector<uint16_t> addrToWrite;
    std::vector<uint16_t> dataToWrite;

    uint16_t x0020_value = mRegistersMap->GetValue(0, 0x0020);
    this->SetActiveChannel(Channel::ChA); //select A channel

    addrToWrite = mRegistersMap->GetUsedAddresses(0);
    //remove 0x0020 register from list, to not change MAC
    addrToWrite.erase(std::find(addrToWrite.begin(), addrToWrite.end(), 0x0020));
    for (auto address : addrToWrite)
        dataToWrite.push_back(mRegistersMap->GetValue(0, address));

    status = SPI_write_batch(&addrToWrite[0], &dataToWrite[0], addrToWrite.size(), true);
    if (status != OpStatus::Success)
        return status;
    //after all channel A registers have been written, update 0x0020 register value
    status = SPI_write(0x0020, x0020_value);
    if (status != OpStatus::Success)
        return status;
    this->SetActiveChannel(Channel::ChB);
    if (status != OpStatus::Success)
        return status;

    addrToWrite = mRegistersMap->GetUsedAddresses(1);
    dataToWrite.clear();
    for (auto address : addrToWrite)
    {
        dataToWrite.push_back(mRegistersMap->GetValue(1, address));
    }
    this->SetActiveChannel(Channel::ChB); //select B channel
    status = SPI_write_batch(&addrToWrite[0], &dataToWrite[0], addrToWrite.size(), true);
    if (status != OpStatus::Success)
        return status;

    return OpStatus::Success;
}

OpStatus LMS7002M::DownloadAll()
{
    if (!controlPort)
        return ReportError(OpStatus::IOFailure, "No device connected"s);

    OpStatus status;
    ChannelScope scope(this, true);

    std::vector<uint16_t> addrToRead = mRegistersMap->GetUsedAddresses(0);
    std::vector<uint16_t> dataReceived;
    dataReceived.resize(addrToRead.size(), 0);
    this->SetActiveChannel(Channel::ChA);
    status = SPI_read_batch(&addrToRead[0], &dataReceived[0], addrToRead.size());
    if (status != OpStatus::Success)
        return status;

    for (uint16_t i = 0; i < addrToRead.size(); ++i)
    {
        mRegistersMap->SetValue(0, addrToRead[i], dataReceived[i]);
    }

    addrToRead.clear(); //add only B channel addresses
    addrToRead = mRegistersMap->GetUsedAddresses(1);
    dataReceived.resize(addrToRead.size(), 0);

    this->SetActiveChannel(Channel::ChB);
    status = SPI_read_batch(&addrToRead[0], &dataReceived[0], addrToRead.size());
    if (status != OpStatus::Success)
        return status;
    for (uint16_t i = 0; i < addrToRead.size(); ++i)
        mRegistersMap->SetValue(1, addrToRead[i], dataReceived[i]);

    return OpStatus::Success;
}

OpStatus LMS7002M::SetInterfaceFrequency(float_type cgen_freq_Hz, const uint8_t hbi, const uint8_t hbd)
{
    OpStatus status;
    status = Modify_SPI_Reg_bits(LMS7002MCSR::HBD_OVR_RXTSP, hbd);
    if (status != OpStatus::Success)
        return status;
    Modify_SPI_Reg_bits(LMS7002MCSR::HBI_OVR_TXTSP, hbi);

    auto siso = Get_SPI_Reg_bits(LML2_SISODDR);
    int mclk2src = Get_SPI_Reg_bits(LMS7002MCSR::MCLK2SRC);
    if (hbd == 7 || (hbd == 0 && siso == 0)) //bypass
    {
        Modify_SPI_Reg_bits(LMS7002MCSR::RXTSPCLKA_DIV, 0);
        Modify_SPI_Reg_bits(LMS7002MCSR::RXDIVEN, false);
        Modify_SPI_Reg_bits(LMS7002MCSR::MCLK2SRC, (mclk2src & 1) | 0x2);
    }
    else
    {
        uint8_t divider = static_cast<uint8_t>(std::pow(2.0, hbd + siso));
        if (divider > 1)
            Modify_SPI_Reg_bits(LMS7002MCSR::RXTSPCLKA_DIV, (divider / 2) - 1);
        else
            Modify_SPI_Reg_bits(LMS7002MCSR::RXTSPCLKA_DIV, 0);
        Modify_SPI_Reg_bits(LMS7002MCSR::RXDIVEN, true);
        Modify_SPI_Reg_bits(LMS7002MCSR::MCLK2SRC, mclk2src & 1);
    }

    if (Get_SPI_Reg_bits(LMS7002MCSR::RX_MUX) == 0)
    {
        bool mimoBypass = (hbd == 7) && (siso == 0);
        Modify_SPI_Reg_bits(LMS7002MCSR::RXRDCLK_MUX, mimoBypass ? 3 : 1);
        Modify_SPI_Reg_bits(LMS7002MCSR::RXWRCLK_MUX, mimoBypass ? 1 : 2);
    }

    siso = Get_SPI_Reg_bits(LML1_SISODDR);
    int mclk1src = Get_SPI_Reg_bits(LMS7002MCSR::MCLK1SRC);
    if (hbi == 7 || (hbi == 0 && siso == 0)) //bypass
    {
        Modify_SPI_Reg_bits(LMS7002MCSR::TXTSPCLKA_DIV, 0);
        Modify_SPI_Reg_bits(LMS7002MCSR::TXDIVEN, false);
        status = Modify_SPI_Reg_bits(LMS7002MCSR::MCLK1SRC, (mclk1src & 1) | 0x2);
    }
    else
    {
        uint8_t divider = static_cast<uint8_t>(std::pow(2.0, hbi + siso));
        if (divider > 1)
            Modify_SPI_Reg_bits(LMS7002MCSR::TXTSPCLKA_DIV, (divider / 2) - 1);
        else
            Modify_SPI_Reg_bits(LMS7002MCSR::TXTSPCLKA_DIV, 0);
        Modify_SPI_Reg_bits(LMS7002MCSR::TXDIVEN, true);
        status = Modify_SPI_Reg_bits(LMS7002MCSR::MCLK1SRC, mclk1src & 1);
    }

    if (Get_SPI_Reg_bits(LMS7002MCSR::TX_MUX) == 0)
    {
        bool mimoBypass = (hbi == 7) && (siso == 0);
        Modify_SPI_Reg_bits(LMS7002MCSR::TXRDCLK_MUX, mimoBypass ? 0 : 2);
        Modify_SPI_Reg_bits(LMS7002MCSR::TXWRCLK_MUX, 0);
    }

    //clock rate already set because the readback frequency is pretty-close,
    //dont set the cgen frequency again to save time due to VCO selection
    // const auto freqDiff = std::abs(this->GetFrequencyCGEN() - cgen_freq_Hz);
    // if (not this->GetCGENLocked() or freqDiff > 10.0)
    {
        status = SetFrequencyCGEN(cgen_freq_Hz);
        if (status != OpStatus::Success)
            return status;
    }
    return status;
}

void LMS7002M::ConfigureLML_RF2BB(
    const LMLSampleSource s0, const LMLSampleSource s1, const LMLSampleSource s2, const LMLSampleSource s3)
{
    //map a sample source to a position
    const std::map<LMLSampleSource, uint16_t> m{
        { LMLSampleSource::AI, 1 },
        { LMLSampleSource::AQ, 0 },
        { LMLSampleSource::BI, 3 },
        { LMLSampleSource::BQ, 2 },
    };

    //load the same config on both LMLs
    //only one will get used based on direction
    this->Modify_SPI_Reg_bits(LMS7002MCSR::LML1_S3S, m.at(s3));
    this->Modify_SPI_Reg_bits(LMS7002MCSR::LML1_S2S, m.at(s2));
    this->Modify_SPI_Reg_bits(LMS7002MCSR::LML1_S1S, m.at(s1));
    this->Modify_SPI_Reg_bits(LMS7002MCSR::LML1_S0S, m.at(s0));

    this->Modify_SPI_Reg_bits(LMS7002MCSR::LML2_S3S, m.at(s3));
    this->Modify_SPI_Reg_bits(LMS7002MCSR::LML2_S2S, m.at(s2));
    this->Modify_SPI_Reg_bits(LMS7002MCSR::LML2_S1S, m.at(s1));
    this->Modify_SPI_Reg_bits(LMS7002MCSR::LML2_S0S, m.at(s0));
}

void LMS7002M::ConfigureLML_BB2RF(
    const LMLSampleSource s0, const LMLSampleSource s1, const LMLSampleSource s2, const LMLSampleSource s3)
{
    //map a sample source to a position
    const std::map<LMLSampleSource, uint16_t> m{
        { s3, 2 },
        { s2, 3 },
        { s0, 1 },
        { s1, 0 },
    };

    //load the same config on both LMLs
    //only one will get used based on direction
    this->Modify_SPI_Reg_bits(LMS7002MCSR::LML1_BQP, m.at(LMLSampleSource::BQ));
    this->Modify_SPI_Reg_bits(LMS7002MCSR::LML1_BIP, m.at(LMLSampleSource::BI));
    this->Modify_SPI_Reg_bits(LMS7002MCSR::LML1_AQP, m.at(LMLSampleSource::AQ));
    this->Modify_SPI_Reg_bits(LMS7002MCSR::LML1_AIP, m.at(LMLSampleSource::AI));

    this->Modify_SPI_Reg_bits(LMS7002MCSR::LML2_BQP, m.at(LMLSampleSource::BQ));
    this->Modify_SPI_Reg_bits(LMS7002MCSR::LML2_BIP, m.at(LMLSampleSource::BI));
    this->Modify_SPI_Reg_bits(LMS7002MCSR::LML2_AQP, m.at(LMLSampleSource::AQ));
    this->Modify_SPI_Reg_bits(LMS7002MCSR::LML2_AIP, m.at(LMLSampleSource::AI));
}

OpStatus LMS7002M::SetRxDCRemoval(const bool enable)
{
    this->Modify_SPI_Reg_bits(LMS7002MCSR::DC_BYP_RXTSP, enable ? 0 : 1);
    this->Modify_SPI_Reg_bits(LMS7002MCSR::DCCORR_AVG_RXTSP, 0x7);
    return OpStatus::Success;
}

OpStatus LMS7002M::EnableSXTDD(bool tdd)
{
    ChannelScope scope(this, LMS7002M::Channel::ChSXT);
    Modify_SPI_Reg_bits(PD_LOCH_T2RBUF, tdd ? 0 : 1);
    Modify_SPI_Reg_bits(MAC, 1); // switch to SXR
    return Modify_SPI_Reg_bits(PD_VCO, tdd ? 1 : 0);
}

bool LMS7002M::GetRxDCRemoval(void)
{
    return this->Get_SPI_Reg_bits(LMS7002MCSR::DC_BYP_RXTSP) == 0;
}

OpStatus LMS7002M::SetDCOffset(TRXDir dir, const float_type I, const float_type Q)
{
    const bool bypass = I == 0.0 and Q == 0.0;
    if (dir == TRXDir::Tx)
    {
        this->Modify_SPI_Reg_bits(LMS7002MCSR::DC_BYP_TXTSP, bypass ? 1 : 0);
        this->Modify_SPI_Reg_bits(LMS7002MCSR::DCCORRI_TXTSP, std::lrint(I * 127));
        this->Modify_SPI_Reg_bits(LMS7002MCSR::DCCORRQ_TXTSP, std::lrint(Q * 127));
    }
    else
    {
        Modify_SPI_Reg_bits(LMS7002MCSR::EN_DCOFF_RXFE_RFE, bypass ? 0 : 1);
        unsigned val = std::lrint(std::abs(I * 63)) + (I < 0 ? 64 : 0);
        Modify_SPI_Reg_bits(LMS7002MCSR::DCOFFI_RFE, val);
        val = std::lrint(std::abs(Q * 63)) + (Q < 0 ? 64 : 0);
        Modify_SPI_Reg_bits(LMS7002MCSR::DCOFFQ_RFE, val);
    }
    return OpStatus::Success;
}

void LMS7002M::GetDCOffset(TRXDir dir, float_type& I, float_type& Q)
{
    if (dir == TRXDir::Tx)
    {
        I = static_cast<int8_t>(this->Get_SPI_Reg_bits(LMS7002MCSR::DCCORRI_TXTSP)) / 127.0; //signed 8-bit
        Q = static_cast<int8_t>(this->Get_SPI_Reg_bits(LMS7002MCSR::DCCORRQ_TXTSP)) / 127.0; //signed 8-bit
    }
    else
    {
        auto i = Get_SPI_Reg_bits(LMS7002MCSR::DCOFFI_RFE);
        I = ((i & 0x40) ? -1.0 : 1.0) * (i & 0x3F) / 63.0;
        auto q = Get_SPI_Reg_bits(LMS7002MCSR::DCOFFQ_RFE);
        Q = ((q & 0x40) ? -1.0 : 1.0) * (q & 0x3F) / 63.0;
    }
}

OpStatus LMS7002M::SetIQBalance(const TRXDir dir, const float_type phase, const float_type gainI, const float_type gainQ)
{
    const bool bypassPhase = (phase == 0.0);
    const bool bypassGain = ((gainI == 1.0) and (gainQ == 1.0)) or ((gainI == 0.0) and (gainQ == 0.0));
    int iqcorr = std::lrint(2047 * (phase / (M_PI / 2)));
    int gcorri = std::lrint(2047 * gainI);
    int gcorrq = std::lrint(2047 * gainQ);

    this->Modify_SPI_Reg_bits(dir == TRXDir::Tx ? LMS7002MCSR::PH_BYP_TXTSP : LMS7002MCSR::PH_BYP_RXTSP, bypassPhase ? 1 : 0);
    this->Modify_SPI_Reg_bits(dir == TRXDir::Tx ? LMS7002MCSR::GC_BYP_TXTSP : LMS7002MCSR::GC_BYP_RXTSP, bypassGain ? 1 : 0);
    this->Modify_SPI_Reg_bits(dir == TRXDir::Tx ? LMS7002MCSR::IQCORR_TXTSP : LMS7002MCSR::IQCORR_RXTSP, iqcorr);
    this->Modify_SPI_Reg_bits(dir == TRXDir::Tx ? LMS7002MCSR::GCORRI_TXTSP : LMS7002MCSR::GCORRI_RXTSP, gcorri);
    this->Modify_SPI_Reg_bits(dir == TRXDir::Tx ? LMS7002MCSR::GCORRQ_TXTSP : LMS7002MCSR::GCORRQ_RXTSP, gcorrq);
    return OpStatus::Success;
}

void LMS7002M::GetIQBalance(const TRXDir dir, float_type& phase, float_type& gainI, float_type& gainQ)
{
    int iqcorr = static_cast<int16_t>(
                     this->Get_SPI_Reg_bits(dir == TRXDir::Tx ? LMS7002MCSR::IQCORR_TXTSP : LMS7002MCSR::IQCORR_RXTSP) << 4) >>
                 4; //sign extend 12-bit
    int gcorri = static_cast<int16_t>(
        this->Get_SPI_Reg_bits(dir == TRXDir::Tx ? LMS7002MCSR::GCORRI_TXTSP : LMS7002MCSR::GCORRI_RXTSP)); //unsigned 11-bit
    int gcorrq = static_cast<int16_t>(
        this->Get_SPI_Reg_bits(dir == TRXDir::Tx ? LMS7002MCSR::GCORRQ_TXTSP : LMS7002MCSR::GCORRQ_RXTSP)); //unsigned 11-bit

    phase = (M_PI / 2) * iqcorr / 2047.0;
    gainI = gcorri / 2047.0;
    gainQ = gcorrq / 2047.0;
}

void LMS7002M::EnableValuesCache(bool enabled)
{
    useCache = enabled;
}

bool LMS7002M::IsValuesCacheEnabled() const
{
    return useCache;
}

MCU_BD* LMS7002M::GetMCUControls() const
{
    return mcuControl;
}

float_type LMS7002M::GetTemperature()
{
    if (CalibrateInternalADC(32) != OpStatus::Success)
        return 0;
    Modify_SPI_Reg_bits(RSSI_PD, 0);
    Modify_SPI_Reg_bits(RSSI_RSSIMODE, 0);
    uint16_t biasMux = Get_SPI_Reg_bits(MUX_BIAS_OUT);
    Modify_SPI_Reg_bits(MUX_BIAS_OUT, 2);

    std::this_thread::sleep_for(std::chrono::microseconds(250));
    const uint16_t reg606 = SPI_read(0x0606, true);
    float Vtemp = (reg606 >> 8) & 0xFF;
    Vtemp *= 1.84;
    float Vptat = reg606 & 0xFF;
    Vptat *= 1.84;
    float Vdiff = Vptat - Vtemp;
    Vdiff /= 1.05;
    float temperature = 45.0 + Vdiff;
    Modify_SPI_Reg_bits(MUX_BIAS_OUT, biasMux);
    lime::debug("Vtemp 0x%04X, Vptat 0x%04X, Vdiff = %.2f, temp= %.3f", (reg606 >> 8) & 0xFF, reg606 & 0xFF, Vdiff, temperature);
    return temperature;
}

OpStatus LMS7002M::CopyChannelRegisters(const Channel src, const Channel dest, const bool copySX)
{
    ChannelScope scope(this);

    std::vector<uint16_t> addrToWrite;
    addrToWrite = mRegistersMap->GetUsedAddresses(1);
    if (!copySX)
    {
        const auto& SXMemoryAddresses = MemorySectionAddresses.at(MemorySection::SX);
        for (uint32_t address = SXMemoryAddresses.at(0); address <= SXMemoryAddresses.at(1); ++address)
            addrToWrite.erase(std::find(addrToWrite.begin(), addrToWrite.end(), address));
    }
    for (auto address : addrToWrite)
    {
        uint16_t data = mRegistersMap->GetValue(src == Channel::ChA ? 0 : 1, address);
        mRegistersMap->SetValue(dest == Channel::ChA ? 0 : 1, address, data);
    }
    if (controlPort)
        UploadAll();

    return OpStatus::Success;
}

OpStatus LMS7002M::CalibrateAnalogRSSI_DC_Offset()
{
    Modify_SPI_Reg_bits(EN_INSHSW_W_RFE, 1);
    CalibrateInternalADC(0);
    Modify_SPI_Reg_bits(LMS7002MCSR::PD_RSSI_RFE, 0);
    Modify_SPI_Reg_bits(LMS7002MCSR::PD_TIA_RFE, 0);

    /*Modify_SPI_Reg_bits(LMS7002MCSR::RSSIDC_RSEL, 22);
    Modify_SPI_Reg_bits(LMS7002MCSR::RSSIDC_HYSCMP, 0);
    Modify_SPI_Reg_bits(LMS7002MCSR::RSSIDC_PD, 0);*/
    SPI_write(0x0640, 22 << 4);

    Modify_SPI_Reg_bits(LMS7002MCSR::RSSIDC_DCO2, 0);

    int value = -63;
    uint8_t wrValue = abs(value);
    if (value < 0)
        wrValue |= 0x40;
    Modify_SPI_Reg_bits(LMS7002MCSR::RSSIDC_DCO1, wrValue, true);
    uint8_t cmp = Get_SPI_Reg_bits(LMS7002MCSR::RSSIDC_CMPSTATUS, true);
    uint8_t cmpPrev = cmp;
    std::vector<int8_t> edges;
    for (value = -63; value < 64; ++value)
    {
        wrValue = abs(value);
        if (value < 0)
            wrValue |= 0x40;
        Modify_SPI_Reg_bits(LMS7002MCSR::RSSIDC_DCO1, wrValue, true);
        std::this_thread::sleep_for(std::chrono::microseconds(5));
        cmp = Get_SPI_Reg_bits(LMS7002MCSR::RSSIDC_CMPSTATUS, true);
        if (cmp != cmpPrev)
        {
            edges.push_back(value);
            cmpPrev = cmp;
        }
        if (edges.size() > 1)
            break;
    }
    if (edges.size() != 2)
    {
        lime::debug("Not found"s);
        return ReportError(OpStatus::InvalidValue, "Failed to find value"s);
    }
    int8_t found = (edges[0] + edges[1]) / 2;
    wrValue = abs(found);
    if (found < 0)
        wrValue |= 0x40;
    Modify_SPI_Reg_bits(LMS7002MCSR::RSSIDC_DCO1, wrValue, true);
    lime::debug("Found %i", found);
    Modify_SPI_Reg_bits(EN_INSHSW_W_RFE, 0);
    return OpStatus::Success;
}

double LMS7002M::GetClockFreq(ClockID clk_id)
{
    switch (clk_id)
    {
    case ClockID::CLK_REFERENCE:
        return GetReferenceClk_SX(TRXDir::Rx);
    case ClockID::CLK_SXR:
        return GetFrequencySX(TRXDir::Rx);
    case ClockID::CLK_SXT:
        return GetFrequencySX(TRXDir::Tx);
    case ClockID::CLK_CGEN:
        return GetFrequencyCGEN();
    case ClockID::CLK_RXTSP:
        return GetReferenceClk_TSP(TRXDir::Rx);
    case ClockID::CLK_TXTSP:
        return GetReferenceClk_TSP(TRXDir::Tx);
    default:
        lime::ReportError(OpStatus::InvalidValue, "Invalid clock ID."s);
        return 0;
    }
}

OpStatus LMS7002M::SetClockFreq(ClockID clk_id, double freq)
{
    switch (clk_id)
    {
    case ClockID::CLK_REFERENCE:
        // TODO: recalculate CGEN,SXR/T
        break;
    case ClockID::CLK_CGEN:
        return SetFrequencyCGEN(freq, true, nullptr);
        break;
    case ClockID::CLK_SXR:
        return SetFrequencySX(TRXDir::Rx, freq, nullptr);
        break;
    case ClockID::CLK_SXT:
        return SetFrequencySX(TRXDir::Rx, freq, nullptr);
        break;
    case ClockID::CLK_RXTSP:
    case ClockID::CLK_TXTSP:
        return ReportError(OpStatus::InvalidValue, "RxTSP/TxTSP Clocks are read only"s);
    default:
        return ReportError(OpStatus::InvalidValue, "LMS7002M::SetClockFreq Unknown clock id"s);
    }
    return OpStatus::Success;
}

float_type LMS7002M::GetSampleRate(TRXDir dir, Channel ch)
{
    ChannelScope scope(this, ch);
    return GetSampleRate(dir);
}

float_type LMS7002M::GetSampleRate(TRXDir dir)
{
    const auto& parameter = dir == TRXDir::Tx ? HBI_OVR_TXTSP : HBD_OVR_RXTSP;

    uint16_t ratio = Get_SPI_Reg_bits(parameter);

    double interface_Hz = GetReferenceClk_TSP(dir);

    // If decimation/interpolation is 0 (2^1) or 7 (bypass), interface clocks should not be divided
    if (ratio != 7)
    {
        interface_Hz /= 2 * pow(2.0, ratio);
    }

    return interface_Hz;
}

OpStatus LMS7002M::SetGFIRFilter(TRXDir dir, Channel ch, bool enabled, double bandwidth)
{
    ChannelScope scope(this, ch);
    const bool bypassFIR = !enabled;
    if (dir == TRXDir::Tx)
    {
        Modify_SPI_Reg_bits(LMS7002MCSR::GFIR1_BYP_TXTSP, bypassFIR);
        Modify_SPI_Reg_bits(LMS7002MCSR::GFIR2_BYP_TXTSP, bypassFIR);
        Modify_SPI_Reg_bits(LMS7002MCSR::GFIR3_BYP_TXTSP, bypassFIR);
    }
    else
    {
        Modify_SPI_Reg_bits(LMS7002MCSR::GFIR1_BYP_RXTSP, bypassFIR);
        Modify_SPI_Reg_bits(LMS7002MCSR::GFIR2_BYP_RXTSP, bypassFIR);
        Modify_SPI_Reg_bits(LMS7002MCSR::GFIR3_BYP_RXTSP, bypassFIR);
        const bool sisoDDR = Get_SPI_Reg_bits(LML1_SISODDR);
        const bool clockIsNotInverted = !(enabled | sisoDDR);
        if (ch == LMS7002M::Channel::ChB)
        {
            Modify_SPI_Reg_bits(LMS7002MCSR::CDSN_RXBLML, clockIsNotInverted);
            Modify_SPI_Reg_bits(LMS7002MCSR::CDS_RXBLML, enabled ? 3 : 0);
        }
        else
        {
            Modify_SPI_Reg_bits(LMS7002MCSR::CDSN_RXALML, clockIsNotInverted);
            Modify_SPI_Reg_bits(LMS7002MCSR::CDS_RXALML, enabled ? 3 : 0);
        }
    }
    if (!enabled)
        return OpStatus::Success;

    if (bandwidth <= 0)
        return OpStatus::InvalidValue;

    double w, w2;
    int L;
    int div = 1;

    bandwidth /= 1e6;
    double interface_MHz;
    int ratio;
    if (dir == TRXDir::Tx)
    {
        ratio = Get_SPI_Reg_bits(LMS7002MCSR::HBI_OVR_TXTSP);
    }
    else
    {
        ratio = Get_SPI_Reg_bits(LMS7002MCSR::HBD_OVR_RXTSP);
    }

    interface_MHz = GetReferenceClk_TSP(dir) / 1e6;
    if (ratio != 7)
        div = (2 << (ratio));

    w = (bandwidth / 2) / (interface_MHz / div);
    L = div > 8 ? 8 : div;
    div -= 1;

    w2 = w * 1.1;
    if (w2 > 0.495)
    {
        w2 = w * 1.05;
        if (w2 > 0.495)
        {
            lime::error("GFIR LPF cannot be set to the requested bandwidth (%f)", bandwidth);
            return OpStatus::Error;
        }
    }

    double coef[120];
    double coef2[40];

    GenerateFilter(L * 15, w, w2, 1.0, 0, coef);
    GenerateFilter(L * 5, w, w2, 1.0, 0, coef2);

    if (dir == TRXDir::Tx)
    {
        Modify_SPI_Reg_bits(LMS7002MCSR::GFIR1_N_TXTSP, div);
        Modify_SPI_Reg_bits(LMS7002MCSR::GFIR2_N_TXTSP, div);
        Modify_SPI_Reg_bits(LMS7002MCSR::GFIR3_N_TXTSP, div);
    }
    else
    {
        Modify_SPI_Reg_bits(LMS7002MCSR::GFIR1_N_RXTSP, div);
        Modify_SPI_Reg_bits(LMS7002MCSR::GFIR2_N_RXTSP, div);
        Modify_SPI_Reg_bits(LMS7002MCSR::GFIR3_N_RXTSP, div);
    }

    OpStatus status;
    if ((status = SetGFIRCoefficients(dir, 0, coef2, L * 5)) != OpStatus::Success)
        return status;
    if ((status = SetGFIRCoefficients(dir, 1, coef2, L * 5)) != OpStatus::Success)
        return status;
    if ((status = SetGFIRCoefficients(dir, 2, coef, L * 15)) != OpStatus::Success)
        return status;

    std::stringstream ss;
    ss << "LMS "sv << ToString(dir) << " GFIR coefficients (BW: "sv << bandwidth << " MHz):\n"sv;
    ss << "GFIR1 = GFIR2:"sv;
    for (int i = 0; i < L * 5; ++i)
        ss << " " << coef2[i];
    ss << std::endl;
    ss << "GFIR3:"sv;
    for (int i = 0; i < L * 15; ++i)
        ss << " "sv << coef[i];
    ss << std::endl;
    lime::info(ss.str());

    return ResetLogicRegisters();
}

void LMS7002M::SetOnCGENChangeCallback(CGENChangeCallbackType callback, void* userData)
{
    mCallback_onCGENChange = callback;
    mCallback_onCGENChange_userData = userData;
}

OpStatus LMS7002M::SetRxLPF(double rfBandwidth_Hz)
{
    const int tiaGain = Get_SPI_Reg_bits(G_TIA_RFE);
    if (tiaGain < 1 || tiaGain > 3)
        return ReportError(OpStatus::InvalidValue, "RxLPF: Invalid G_TIA gain value");

    Modify_SPI_Reg_bits(PD_TIA_RFE, 0);
    Modify_SPI_Reg_bits(EN_G_RFE, 1);

    Modify_SPI_Reg_bits(ICT_TIAMAIN_RFE, 2);
    Modify_SPI_Reg_bits(ICT_TIAOUT_RFE, 2);

    Modify_SPI_Reg_bits(ICT_LPF_IN_RBB, 0x0C);
    Modify_SPI_Reg_bits(ICT_LPF_OUT_RBB, 0x0C);

    Modify_SPI_Reg_bits(ICT_PGA_OUT_RBB, 0x14);
    Modify_SPI_Reg_bits(ICT_PGA_IN_RBB, 0x14);
    const int pgaGain = Get_SPI_Reg_bits(G_PGA_RBB);
    if (pgaGain != 12)
    {
        lime::warning("RxLPF modifying G_PGA_RBB %i -> 12", pgaGain);
        Modify_SPI_Reg_bits(G_PGA_RBB, 12);
    }

    Modify_SPI_Reg_bits(RCC_CTL_PGA_RBB, 0x18);
    Modify_SPI_Reg_bits(C_CTL_PGA_RBB, 1);

    const double rxLpfMin = (tiaGain == 1) ? 4e6 : 1.5e6;
    const double rxLpfMax = 160e6;
    if (rfBandwidth_Hz != 0 && (rfBandwidth_Hz < rxLpfMin || rfBandwidth_Hz > rxLpfMax))
    {
        lime::warning(
            "Requested RxLPF(%g) is out of range [%g - %g]. Clamping to valid range.", rfBandwidth_Hz, rxLpfMin, rxLpfMax);
        rfBandwidth_Hz = std::clamp(rfBandwidth_Hz, rxLpfMin, rxLpfMax);
    }

    const double bandwidth_MHz = rfBandwidth_Hz / 1e6;

    int TIA_C;
    if (tiaGain == 1)
        TIA_C = 120 * 45 / (bandwidth_MHz / 2 / 1.5) - 15;
    else
        TIA_C = 120 * 14 / (bandwidth_MHz / 2 / 1.5) - 10;
    TIA_C = std::clamp(TIA_C, 0, 4095);

    int TIA_RCOMP = std::clamp(15 - TIA_C * 2 / 100, 0, 15);

    int TIA_CCOMP = (TIA_C / 100) + (tiaGain == 1 ? 1 : 0);
    TIA_CCOMP = std::clamp(TIA_CCOMP, 0, 15);

    int RX_L_C = 120 * 18 / (bandwidth_MHz / 2 / 0.75) - 103;
    RX_L_C = std::clamp(RX_L_C, 0, 2047);

    int RX_H_C = 120 * 50 / (bandwidth_MHz / 2 / 0.75) - 50;
    RX_H_C = std::clamp(RX_H_C, 0, 255);

    lime::debug("RxLPF(%g): TIA_C=%i, TIA_RCOMP=%i, TIA_CCOMP=%i, RX_L_C=%i, RX_H_C=%i\n",
        rfBandwidth_Hz,
        TIA_C,
        TIA_RCOMP,
        TIA_CCOMP,
        RX_L_C,
        RX_H_C);

    uint16_t cfb_tia_rfe = TIA_C;
    uint16_t rcomp_tia_rfe = TIA_RCOMP;
    uint16_t ccomp_tia_rfe = TIA_CCOMP;
    uint16_t input_ctl_pga_rbb = 4;
    uint16_t c_ctl_lpfl_rbb = RX_L_C;
    uint16_t c_ctl_lpfh_rbb = RX_H_C;
    uint16_t powerDowns = 0xD; // 0x0115[3:0]

    const double ifbw = bandwidth_MHz / 2 / 0.75;
    const uint16_t rcc_ctl_lpfh_rbb = std::clamp(ifbw / 10 - 2, 0.0, 7.0);
    uint16_t rcc_ctl_lpfl_rbb = 5;
    if (ifbw >= 20)
        rcc_ctl_lpfl_rbb = 5;
    else if (ifbw >= 15)
        rcc_ctl_lpfl_rbb = 4;
    else if (ifbw >= 10)
        rcc_ctl_lpfl_rbb = 3;
    else if (ifbw >= 5)
        rcc_ctl_lpfl_rbb = 2;
    else if (ifbw >= 3)
        rcc_ctl_lpfl_rbb = 1;
    else // (bw>=1.5)
        rcc_ctl_lpfl_rbb = 0;

    if (rfBandwidth_Hz <= 0) // LPF bypass
    {
        lime::info("RxLPF bypassed");
        powerDowns = 0xD;
        input_ctl_pga_rbb = 2;
    }
    else if (rfBandwidth_Hz < rxLpfMin)
    {
        lime::warning("RxLPF(%g) frequency too low. Clamping to %g MHz.", rfBandwidth_Hz, rxLpfMin / 1e6);
        if (tiaGain == 1)
        {
            cfb_tia_rfe = 4035;
            rcc_ctl_lpfl_rbb = 1;
            c_ctl_lpfl_rbb = 707;
        }
        else
        {
            cfb_tia_rfe = 3350;
            rcc_ctl_lpfl_rbb = 0;
            c_ctl_lpfl_rbb = 2047;
        }
        rcomp_tia_rfe = 0;
        ccomp_tia_rfe = 15;
        powerDowns = 0x9;
        input_ctl_pga_rbb = 0;
    }
    else if (rxLpfMin <= rfBandwidth_Hz && rfBandwidth_Hz <= 30e6)
    {
        powerDowns = 0x9;
        input_ctl_pga_rbb = 0;
    }
    else if (30e6 <= rfBandwidth_Hz && rfBandwidth_Hz <= rxLpfMax)
    {
        powerDowns = 0x5;
        input_ctl_pga_rbb = 1;
    }

    Modify_SPI_Reg_bits(CFB_TIA_RFE, cfb_tia_rfe);
    Modify_SPI_Reg_bits(RCOMP_TIA_RFE, rcomp_tia_rfe);
    Modify_SPI_Reg_bits(CCOMP_TIA_RFE, ccomp_tia_rfe);
    Modify_SPI_Reg_bits(0x0115, 3, 0, powerDowns);
    Modify_SPI_Reg_bits(INPUT_CTL_PGA_RBB, input_ctl_pga_rbb);
    Modify_SPI_Reg_bits(C_CTL_LPFL_RBB, c_ctl_lpfl_rbb);
    Modify_SPI_Reg_bits(C_CTL_LPFH_RBB, c_ctl_lpfh_rbb);
    Modify_SPI_Reg_bits(RCC_CTL_LPFL_RBB, rcc_ctl_lpfl_rbb);
    Modify_SPI_Reg_bits(RCC_CTL_LPFH_RBB, rcc_ctl_lpfh_rbb);

    return OpStatus::Success;
}

OpStatus LMS7002M::SetTxLPF(double rfBandwidth_Hz)
{
    const double txLpfLowRange[2] = { 5e6, 33e6 };
    const double txLpfHighRange[2] = { 56e6, 160e6 };

    // common setup
    Modify_SPI_Reg_bits(0x0106, 15, 0, 0x318C);
    Modify_SPI_Reg_bits(0x0107, 15, 0, 0x318C);
    Modify_SPI_Reg_bits(ICT_IAMP_FRP_TBB, 8);
    Modify_SPI_Reg_bits(ICT_IAMP_GG_FRP_TBB, 12);
    Modify_SPI_Reg_bits(CCAL_LPFLAD_TBB, 31);
    Modify_SPI_Reg_bits(RCAL_LPFS5_TBB, 255);
    Modify_SPI_Reg_bits(R5_LPF_BYP_TBB, 1);
    Modify_SPI_Reg_bits(BYPLADDER_TBB, 0);

    uint16_t powerDowns = 0x15; // addr 0x0105[4:0]

    if (rfBandwidth_Hz <= 0) // Bypass LPF
    {
        lime::info("TxLPF bypassed");
        powerDowns = 0x15;
        Modify_SPI_Reg_bits(0x0105, 4, 0, powerDowns);
        Modify_SPI_Reg_bits(BYPLADDER_TBB, 1);
        return Modify_SPI_Reg_bits(RCAL_LPFS5_TBB, 0);
    }
    else if (rfBandwidth_Hz < txLpfLowRange[0] || txLpfHighRange[1] < rfBandwidth_Hz)
    {
        lime::warning("Requested TxLPF(%g) bandwidth is out of range [%g - %g]. Clamping to valid value.",
            rfBandwidth_Hz,
            txLpfLowRange[0],
            txLpfHighRange[1]);
        rfBandwidth_Hz = std::clamp(rfBandwidth_Hz, txLpfLowRange[0], txLpfHighRange[1]);
    }

    const double rfbandwidth_MHz = rfBandwidth_Hz / 1e6;
    int rcal_lpflad = 0;
    int rcal_lpfh = 0;

    if (rfBandwidth_Hz < 5.3e6)
    {
        lime::warning("TxLPF(%g) setting bandwidth to %g.", rfBandwidth_Hz, txLpfLowRange[0]);
        rcal_lpflad = 0;
        powerDowns = 0x11;
    }
    else if (rfBandwidth_Hz <= txLpfLowRange[1]) // 5.3-33 MHz
    {
        const double LADlog = 20.0 * std::log10(rfbandwidth_MHz / (2.6 * 2));
        double LADterm1;
        {
            double t1 = 1.92163e-15;
            double t2 = std::sqrt(5.9304678933309e99 * std::pow(LADlog, 2) - 1.64373265875744e101 * LADlog + 1.17784161390406e102);
            LADterm1 = t1 * std::pow(t2 + 7.70095311849832e49 * LADlog - 1.0672267662616e51, 1.0 / 3.0);
        }

        double LADterm2;
        {
            double t1 = 6.50934553014677e18;
            double t2 = std::sqrt(5.9304678933309e99 * std::pow(LADlog, 2) - 1.64373265875744e101 * LADlog + 1.17784161390406e102);
            double t3 = t2 + 7.70095311849832e49 * LADlog - 1.0672267662616e51;
            LADterm2 = t1 / std::pow(t3, 1.0 / 3.0);
        }
        rcal_lpflad = std::clamp(196.916 + LADterm1 - LADterm2, 0.0, 255.0);
        powerDowns = 0x11;
    }
    else if (txLpfLowRange[1] <= rfBandwidth_Hz && rfBandwidth_Hz <= txLpfHighRange[0]) // 33-56 MHz gap
    {
        lime::warning("Requested TxLPF(%g) is in frequency gap [%g-%g], setting bandwidth to %g.",
            rfBandwidth_Hz,
            txLpfLowRange[1],
            txLpfHighRange[0],
            txLpfHighRange[0]);
        rcal_lpfh = 0;
        powerDowns = 0x07;
    }
    else if (rfBandwidth_Hz <= txLpfHighRange[1]) // <160MHz
    {
        const double Hlog = 20 * std::log10(rfbandwidth_MHz / (28 * 2));
        double Hterm1;
        {
            double t1 = 5.66735e-16;
            double t2 = std::sqrt(1.21443429517649e103 * std::pow(Hlog, 2) - 2.85279160551735e104 * Hlog + 1.72772373636442e105);
            double t3 = std::pow(t2 + 3.48487344845762e51 * Hlog - 4.09310646098208e052, 1.0 / 3.0);
            Hterm1 = t1 * t3;
        }
        double Hterm2;
        {
            double t1 = 2.12037432410767e019;
            double t2 = std::sqrt(1.21443429517649e103 * std::pow(Hlog, 2) - 2.85279160551735e104 * Hlog + 1.72772373636442e105);
            double t3 = std::pow(t2 + 3.48487344845762e51 * Hlog - 4.09310646098208e052, 1.0 / 3.0);
            Hterm2 = t1 / t3;
        }
        rcal_lpfh = std::clamp(197.429 + Hterm1 - Hterm2, 0.0, 255.0);
        powerDowns = 0x07;
    }

    lime::debug("TxLPF(%g): LAD=%i, H=%i\n", rfBandwidth_Hz, rcal_lpflad, rcal_lpfh);

    Modify_SPI_Reg_bits(RCAL_LPFLAD_TBB, rcal_lpflad);
    Modify_SPI_Reg_bits(RCAL_LPFH_TBB, rcal_lpfh);
    return Modify_SPI_Reg_bits(0x0105, 4, 0, powerDowns);
}
