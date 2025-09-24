#include "LMS8001.h"
#include "LMS8001_RegistersMap.h"

#include "INI.h"

#include <chrono>
#include <thread>
#include <vector>

#include "limesuiteng/Logger.h"
#include "limesuiteng/Register.h"

#include "comms/SPI/ISPI.h"
#include "comms/SPI/SPI_utilities.h"

using namespace std;

namespace lime {

extern std::vector<const LMS8Parameter*> LMS8parameterList;

LMS8001::LMS8001(std::shared_ptr<ISPI> port)
    : controlPort(port)
    , mRegistersMap(new LMS8001_RegistersMap())
{
    mRefClk_MHz = REF_CLK_MHZ;
    tempSens_T0 = TEMPSENS_T0;

    //memory intervals for registers tests and calibration algorithms
    MemorySectionAddresses[MS_ChipConfig][0] = 0x0000;
    MemorySectionAddresses[MS_ChipConfig][1] = 0x000F;
    MemorySectionAddresses[MS_BiasLDOConfig][0] = 0x0010;
    MemorySectionAddresses[MS_BiasLDOConfig][1] = 0x001F;
    MemorySectionAddresses[MS_Channel][0] = 0x1000;
    MemorySectionAddresses[MS_Channel][1] = 0x107F;
    MemorySectionAddresses[MS_HLMIX][0] = 0x2000;
    MemorySectionAddresses[MS_HLMIX][1] = 0x203F;
    MemorySectionAddresses[MS_PLLConfig][0] = 0x4000;
    MemorySectionAddresses[MS_PLLConfig][1] = 0x401F;
    MemorySectionAddresses[MS_PLLProfiles][0] = 0x4100;
    MemorySectionAddresses[MS_PLLProfiles][1] = 0x417F;

    mRegistersMap->InitializeDefaultValues(LMS8parameterList);
}

LMS8001::~LMS8001()
{
}

/** @brief Sends reset signal to chip, after reset enables B channel controls
    @return 0-success, other-failure
*/
OpStatus LMS8001::ResetChip()
{
    return OpStatus::NotImplemented;
}

/** @brief Reads configuration file and uploads registers to chip
    @param filename Configuration source file
    @return 0-success, other-failure
*/
OpStatus LMS8001::LoadConfig(const std::string& filename)
{
    ifstream f(filename);
    if (f.good() == false) //file not found
    {
        f.close();
        return OpStatus::FileNotFound;
    }
    f.close();
    uint16_t addr = 0;
    uint16_t value = 0;

    OpStatus status;
    INI parser(filename, true);
    parser.select("file_info");
    string type = "";

    type = parser.get("type", string("undefined"));
    stringstream ss;
    if (type.find("lms8001_minimal_config") == string::npos)
    {
        ss << "File " << filename << " not recognized" << endl;
        return OpStatus::Error;
    }

    const int fileVersion = parser.getAs("version"s, 0);

    vector<uint16_t> addrToWrite;
    vector<uint16_t> dataToWrite;

    if (fileVersion == 1)
    {
        parser.select("reference_clock");
        mRefClk_MHz = parser.getAs("ref_clk_mhz", REF_CLK_MHZ);

        parser.select("temp_sens_coeff");
        tempSens_T0 = parser.getAs("T0", TEMPSENS_T0);

        auto section = parser.sections.find("lms8001_registers");

        for (auto pairs = section->second->begin(); pairs != section->second->end(); pairs++)
        {
            sscanf(pairs->first.c_str(), "%hx", &addr);
            sscanf(pairs->second.c_str(), "%hx", &value);
            addrToWrite.push_back(addr);
            dataToWrite.push_back(value);
        }

        status = SPI_write_batch(&addrToWrite[0], &dataToWrite[0], addrToWrite.size());
        if (status != OpStatus::Success)
            return status;
    }
    return OpStatus::Success;
}

/** @brief Reads all registers from chip and saves to file
    @param filename destination filename
    @return 0-success, other failure
*/
OpStatus LMS8001::SaveConfig(const std::string& filename)
{
    OpStatus status;
    INI parser(filename, true);
    parser.create("file_info");
    parser.set("type", "lms8001_minimal_config");
    parser.set("version", 1);

    char addr[80];
    char value[80];

    vector<uint16_t> addrToRead;
    for (uint8_t i = 0; i < MEMORY_SECTIONS_COUNT; ++i)
        for (uint16_t addr = MemorySectionAddresses[i][0]; addr <= MemorySectionAddresses[i][1]; ++addr)
            addrToRead.push_back(addr);
    vector<uint16_t> dataReceived;
    dataReceived.resize(addrToRead.size(), 0);

    parser.create("lms8001_registers");

    //msavic I commented this out so I can write the data from the GUI instead from the Board
    status = SPI_read_batch(&addrToRead[0], &dataReceived[0], addrToRead.size());

    for (uint16_t i = 0; i < addrToRead.size(); ++i)
    {
        sprintf(addr, "0x%04X", addrToRead[i]);
        sprintf(value, "0x%04X", dataReceived[i]);
        parser.set(addr, value);
    }

    parser.create("reference_clock");
    parser.set("ref_clk_mhz", mRefClk_MHz);

    parser.create("temp_sens_coeff");
    parser.set("T0", tempSens_T0);

    parser.save(filename);

    return status;
}

uint16_t LMS8001::Get_SPI_Reg_bits(const LMS8Parameter& param, bool fromChip, uint8_t channel, uint8_t PLLprofile, OpStatus* status)
{
    //msavic ABCD
    uint16_t address = param.address;
    if ((address >= 0x1000) && (address < 0x1020)) //Channel A
        address = address + channel * 0x0020;
    if ((address >= 0x2000) && (address < 0x2010)) //HLMIX A
        address = address + channel * 0x0010;
    if ((address >= 0x4100) && (address < 0x4110)) //PLL Profile 0
        address = address + PLLprofile * 0x0010;

    return Get_SPI_Reg_bits(address, param.msb, param.lsb, fromChip, status);
}

uint16_t LMS8001::Get_SPI_Reg_bits(uint16_t address, uint8_t msb, uint8_t lsb, bool fromChip, OpStatus* status)
{
    Register reg{ address, msb, lsb };
    return ReadRegister(controlPort.get(), reg, status);
}

OpStatus LMS8001::Modify_SPI_Reg_bits(
    const LMS8Parameter& param, const uint16_t value, bool fromChip, uint8_t channel, uint8_t PLLprofile)
{
    //msavic ABCD
    uint16_t address = param.address;
    if ((address >= 0x1000) && (address < 0x1020)) //Channel A
        address = address + channel * 0x0020;
    if ((address >= 0x2000) && (address < 0x2010)) //HLMIX A
        address = address + channel * 0x0010;
    if ((address >= 0x4100) && (address < 0x4110)) //PLL Profile 0
        address = address + PLLprofile * 0x0010;

    return Modify_SPI_Reg_bits(address, param.msb, param.lsb, value, fromChip);
}

OpStatus LMS8001::Modify_SPI_Reg_bits(
    const uint16_t address, const uint8_t msb, const uint8_t lsb, const uint16_t value, bool fromChip)
{
    Register reg{ address, msb, lsb };
    return ModifyRegister(controlPort.get(), reg, value);
}

/** @brief Sets given module registers to default values
    @return 0-success, other-failure
*/
OpStatus LMS8001::SetDefaults(MemorySection module)
{
    OpStatus status = OpStatus::Success;
    vector<uint16_t> addrs;
    vector<uint16_t> values;
    for (uint32_t address = MemorySectionAddresses[module][0]; address <= MemorySectionAddresses[module][1]; ++address)
    {
        addrs.push_back(address);
        values.push_back(mRegistersMap->GetDefaultValue(address));
    }
    status = SPI_write_batch(&addrs[0], &values[0], addrs.size());
    return status;
}

OpStatus LMS8001::SPI_write_batch(const uint16_t* spiAddr, const uint16_t* spiData, uint16_t cnt)
{
    std::vector<uint32_t> data;
    for (size_t i = 0; i < cnt; ++i)
    {
        data.push_back((1 << 31) | (static_cast<uint32_t>(spiAddr[i]) << 16) | spiData[i]); //msbit 1=SPI write
        mRegistersMap->SetValue(spiAddr[i], spiData[i]);
    }

    if (data.size() == 0)
        return OpStatus::Success;
    return controlPort->Transact(data.data(), nullptr, data.size());
}

OpStatus LMS8001::SPI_read_batch(const uint16_t* spiAddr, uint16_t* spiData, uint16_t cnt)
{
    if (!controlPort)
    {
        //return ReportError(OpStatus::IOFailure, "No device connected"s);
        return OpStatus::Error;
    }

    std::vector<uint32_t> dataWr(cnt);
    std::vector<uint32_t> dataRd(cnt);
    for (size_t i = 0; i < cnt; ++i)
    {
        dataWr[i] = spiAddr[i];
    }

    controlPort->Transact(dataWr.data(), dataRd.data(), cnt);
    for (size_t i = 0; i < cnt; ++i)
    {
        spiData[i] = dataRd[i] & 0xffff;
        mRegistersMap->SetValue(spiAddr[i], spiData[i]);
    }
    return OpStatus::Success;
}

} // namespace lime
