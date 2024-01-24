#include <string>
#include <type_traits>
#include <unordered_map>

#include "limesuite/GainTypes.h"
#include "limesuite/MemoryDevices.h"
#include "limesuite/MemoryRegions.h"

using namespace lime;
using namespace std::literals::string_literals;

template<class OriginalKey, class OriginalValue>
const std::unordered_map<std::remove_cv_t<OriginalValue>, OriginalKey> SwapKeysAndValues(
    const std::unordered_map<OriginalKey, OriginalValue>& source)
{
    std::unordered_map<std::remove_cv_t<OriginalValue>, OriginalKey> map;

    for (const auto& pair : source)
    {
        map[pair.second] = pair.first;
    }

    return map;
}

const std::unordered_map<eGainTypes, const std::string> lime::GAIN_TYPES_TEXT{
    { eGainTypes::LNA, "LNA"s },
    { eGainTypes::LoopbackLNA, "LB_LNA"s },
    { eGainTypes::PGA, "PGA"s },
    { eGainTypes::TIA, "TIA"s },
    { eGainTypes::PAD, "PAD"s },
    { eGainTypes::LoopbackPAD, "LB_PAD"s },
    { eGainTypes::IAMP, "IAMP"s },
    { eGainTypes::PA, "PA"s },
    { eGainTypes::UNKNOWN, ""s },
};
const std::unordered_map<std::string, eGainTypes> lime::STRING_TO_GAIN_TYPES = SwapKeysAndValues(GAIN_TYPES_TEXT);

const std::unordered_map<eMemoryDevice, const std::string> lime::MEMORY_DEVICES_TEXT{
    { eMemoryDevice::FPGA_RAM, "FPGA RAM"s },
    { eMemoryDevice::FPGA_FLASH, "FPGA FLASH"s },
    { eMemoryDevice::EEPROM, "EEPROM"s },
};

const std::unordered_map<std::string, eMemoryDevice> lime::STRING_TO_MEMORY_DEVICES =  SwapKeysAndValues(MEMORY_DEVICES_TEXT);

const std::unordered_map<eMemoryRegion, const std::string> lime::MEMORY_REGIONS_TEXT{
    { eMemoryRegion::VCTCXO_DAC, "VCTCXO DAC (non-volatile)"s },
};
