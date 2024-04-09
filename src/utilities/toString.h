#pragma once

#include "limesuiteng/types.h"
#include "limesuiteng/OpStatus.h"

#include <string>

namespace lime {

/// @brief Converts a given TRXDir value into a human readable C-string.
/// @param value The value to convert.
/// @return The C-string representing the direction.
LIME_API const char* ToCString(TRXDir dir);
LIME_API const std::string& ToString(TRXDir dir);

/// @brief Converts a given OpStatus value into a human readable C-string.
/// @param value The value to convert.
/// @return The C-string representing the status.
LIME_API const char* ToCString(OpStatus value);

LIME_API const std::string& ToString(eMemoryRegion value);
LIME_API const std::string& ToString(eGainTypes value);
LIME_API const std::string& ToString(eMemoryDevice value);

template<class T> T ToEnumClass(const std::string& str);
} // namespace lime
