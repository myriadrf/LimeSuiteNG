#ifndef LIME_MEMORYREGIONS_H
#define LIME_MEMORYREGIONS_H

#include "limesuiteng/config.h"

#include <cstdint>
#include <string>
#include <unordered_map>

namespace lime {

enum class eMemoryRegion : uint8_t { VCTCXO_DAC, COUNT };

extern LIME_API const std::unordered_map<eMemoryRegion, const std::string> MEMORY_REGIONS_TEXT;

/// @brief Structure for storing the information of a memory region.
struct Region {
    int32_t address; ///< Starting address of the memory region
    int32_t size; ///< The size of the memory region
};

} // namespace lime

#endif // LIME_MEMORYREGIONS_H
