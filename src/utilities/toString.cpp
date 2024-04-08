#include "toString.h"

#include <string>
#include <unordered_map>

using namespace std::literals::string_literals;

namespace lime {

static const std::string cRx{ "Rx" };
static const std::string cTx{ "Tx" };
const std::string& ToString(TRXDir dir)
{
    switch (dir)
    {
    case TRXDir::Rx:
        return cRx;
    case TRXDir::Tx:
        return cTx;
    }
}

const char* ToCString(TRXDir dir)
{
    return ToString(dir).c_str();
}

const char* ToCString(OpStatus value)
{
    switch (value)
    {
    case OpStatus::SUCCESS:
        return "success";
    case OpStatus::ERROR:
        return "error";
    case OpStatus::NOT_IMPLEMENTED:
        return "not implemented";
    case OpStatus::IO_FAILURE:
        return "input/output failure";
    case OpStatus::INVALID_VALUE:
        return "invalid value";
    case OpStatus::FILE_NOT_FOUND:
        return "file not found";
    case OpStatus::OUT_OF_RANGE:
        return "value out of range";
    case OpStatus::NOT_SUPPORTED:
        return "not supported";
    case OpStatus::TIMEOUT:
        return "timeout";
    case OpStatus::BUSY:
        return "busy";
    case OpStatus::ABORTED:
        return "aborted";
    case OpStatus::PERMISSION_DENIED:
        return "permission denied";
    case OpStatus::NOT_CONNECTED:
        return "not connected";
    }
    return "";
}

static const std::unordered_map<eMemoryRegion, const std::string> MEMORY_REGIONS_TEXT{
    { eMemoryRegion::VCTCXO_DAC, "VCTCXO DAC (non-volatile)"s },
};

const std::string& ToString(eMemoryRegion value)
{
    return MEMORY_REGIONS_TEXT.at(value);
}

static const std::unordered_map<eGainTypes, const std::string> GAIN_TYPES_TEXT{
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

const std::string& ToString(eGainTypes value)
{
    return GAIN_TYPES_TEXT.at(value);
}

static const std::unordered_map<eMemoryDevice, const std::string> MEMORY_DEVICES_TEXT{
    { eMemoryDevice::FPGA_RAM, "FPGA RAM"s },
    { eMemoryDevice::FPGA_FLASH, "FPGA FLASH"s },
    { eMemoryDevice::EEPROM, "EEPROM"s },
};

const std::string& ToString(eMemoryDevice value)
{
    return MEMORY_DEVICES_TEXT.at(value);
}

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

const std::unordered_map<std::string, eMemoryDevice> STRING_TO_MEMORY_DEVICES = SwapKeysAndValues(MEMORY_DEVICES_TEXT);

static const std::unordered_map<std::string, eGainTypes> STRING_TO_GAIN_TYPES = SwapKeysAndValues(GAIN_TYPES_TEXT);
template<> eGainTypes ToEnumClass(const std::string& str)
{
    return STRING_TO_GAIN_TYPES.at(str);
}

} // namespace lime
