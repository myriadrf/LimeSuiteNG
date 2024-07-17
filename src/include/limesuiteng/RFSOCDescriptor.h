#ifndef LIMESUITENG_RFSOCDESCRIPTOR_H
#define LIMESUITENG_RFSOCDESCRIPTOR_H

#include <string>
#include <set>
#include <vector>
#include <unordered_map>

#include "limesuiteng/types.h"

namespace lime {

/// @brief Information about possible gain values.
struct GainValue {
    uint16_t hardwareRegisterValue; ///< The value that is written to the hardware
    float actualGainValue; ///< The actual meaning of the value (in dB)
};

/// @brief General information about the Radio-Frequency System-on-Chip (RFSoC).
struct RFSOCDescriptor {
    std::string name; ///< The name of the system
    uint8_t channelCount; ///< The available channel count of the system
    std::unordered_map<TRXDir, std::vector<std::string>> pathNames; ///< The available antenna names

    Range<double> frequencyRange; ///< Deliverable frequency capabilities of the device
    Range<double> samplingRateRange; ///< Sampling rate capabilities of the device

    std::unordered_map<TRXDir, std::unordered_map<std::string, Range<double>>> antennaRange; ///< Antenna recommended bandwidths
    std::unordered_map<TRXDir, Range<double>> lowPassFilterRange; ///< The ranges of the low pass filter

    std::unordered_map<TRXDir, std::set<eGainTypes>> gains; ///< The types of gains available
    std::unordered_map<TRXDir, std::unordered_map<eGainTypes, Range<double>>> gainRange; ///< The available ranges of each gain
    std::unordered_map<TRXDir, std::unordered_map<eGainTypes, std::vector<GainValue>>> gainValues; ///< The possible gain values
};

} // namespace lime

#endif
