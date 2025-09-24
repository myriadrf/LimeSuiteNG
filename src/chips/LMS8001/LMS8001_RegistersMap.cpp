#include "LMS8001_RegistersMap.h"
#include "LMS8001_parameters.h"

namespace lime {

LMS8001_RegistersMap::LMS8001_RegistersMap()
{
}

LMS8001_RegistersMap::~LMS8001_RegistersMap()
{
}

uint16_t LMS8001_RegistersMap::GetDefaultValue(uint16_t address) const
{
    std::map<uint16_t, Register>::const_iterator iter = mRegMap.find(address);
    if (iter != mRegMap.end())
        return iter->second.defaultValue;
    else
        return 0;
}

void LMS8001_RegistersMap::InitializeDefaultValues(const std::vector<const LMS8Parameter*> parameterList)
{
    for (auto parameter : parameterList)
    {
        uint16_t regValue = mRegMap[parameter->address].defaultValue;
        mRegMap[parameter->address].defaultValue = regValue | (parameter->defaultValue << parameter->lsb);
        mRegMap[parameter->address].value = mRegMap[parameter->address].defaultValue;

        uint16_t currAddress;
        if ((parameter->address >= 0x1000) && (parameter->address < 0x1020)) //Channel A
            for (int ch = 1; ch < 4; ch++) //Channels B, C & D
            {
                currAddress = parameter->address + ch * 0x0020;
                mRegMap[currAddress].value = mRegMap[parameter->address].defaultValue;
            }
        if ((parameter->address >= 0x2000) && (parameter->address < 0x2010)) //HLMIX A
            for (int ch = 1; ch < 4; ch++) //HLMIX B, C & D
            {
                currAddress = parameter->address + ch * 0x0010;
                mRegMap[currAddress].value = mRegMap[parameter->address].defaultValue;
            }
        // PLLprofile - Default values
        if ((parameter->address >= 0x4100) && (parameter->address < 0x4110)) //PLL Profile 0
            for (int profile = 1; profile < 8; profile++) //PLL Profiles from 1 to 7
            {
                currAddress = parameter->address + profile * 0x0010;
                mRegMap[currAddress].value = mRegMap[parameter->address].defaultValue;
            }
    }
}

void LMS8001_RegistersMap::SetValue(const uint16_t address, const uint16_t value)
{
    mRegMap[address].value = value;
}

uint16_t LMS8001_RegistersMap::GetValue(uint16_t address) const
{
    std::map<const uint16_t, Register>::const_iterator iter;
    iter = mRegMap.find(address);
    if (iter != mRegMap.end())
        return iter->second.value;
    else
        return 0;
}

std::vector<uint16_t> LMS8001_RegistersMap::GetUsedAddresses() const
{
    std::vector<uint16_t> addresses;
    for (auto iter : mRegMap)
        addresses.push_back(iter.first);

    return addresses;
}

} // namespace lime