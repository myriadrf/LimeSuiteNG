#include "toString.h"

#include <string>
#include <unordered_map>

using namespace std::literals::string_literals;

namespace lime {

static const std::unordered_map<TRXDir, const std::string> TRXDIR_TEXT{
    { TRXDir::Rx, "Rx"s },
    { TRXDir::Tx, "Tx"s },
};

const std::string& ToString(TRXDir dir)
{
    return TRXDIR_TEXT.at(dir);
}

static const std::unordered_map<OpStatus, const std::string> OP_STATUS_TEXT{
    { OpStatus::Success, "Success"s },
    { OpStatus::Error, "Error"s },
    { OpStatus::NotImplemented, "Not implemented"s },
    { OpStatus::IOFailure, "Input/output failure"s },
    { OpStatus::InvalidValue, "Invalid value"s },
    { OpStatus::FileNotFound, "File not found"s },
    { OpStatus::OutOfRange, "Value out of range"s },
    { OpStatus::NotSupported, "Not supported"s },
    { OpStatus::Timeout, "Timeout"s },
    { OpStatus::Busy, "Busy"s },
    { OpStatus::Aborted, "Aborted"s },
    { OpStatus::PermissionDenied, "Permission denied"s },
    { OpStatus::NotConnected, "Not connected"s },
};

const std::string& ToString(OpStatus value)
{
    return OP_STATUS_TEXT.at(value);
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

static const auto STRING_TO_GAIN_TYPES = SwapKeysAndValues(GAIN_TYPES_TEXT);

template<> LIME_API eGainTypes ToEnumClass(const std::string& str)
{
    return STRING_TO_GAIN_TYPES.at(str);
}

} // namespace lime
