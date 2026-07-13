#pragma once

/**
* @file limesuiteng/ToString.h
* @author Lime Microsystems
* @brief Defines string manipulation functions used to convert library enumerations into std::string format
*/

#include "limesuiteng/types.hpp"
#include "limesuiteng/OpStatus.h"

#include <string>

namespace lime {

/// @brief Converts a given TRXDir value into a human readable string.
/// @param dir The value to convert.
/// @return Channel direction in a string format.
LIME_API const std::string& ToString(TRXDir dir);

/// @brief Converts a given OpStatus value into a human readable string.
/// @param value The value to convert.
/// @return Operation status in a string format.
LIME_API const std::string& ToString(OpStatus value);

/// @brief Converts a given eGainType enum value into a human readable string.
/// @param value The value to convert.
/// @return Gain type in a string format.
LIME_API const std::string& ToString(eGainTypes value);

/// @brief Converts a given eMemoryDevice enum value into a human readable string.
/// @param value The value to convert.
/// @return Memory device type in a string format.
LIME_API const std::string& ToString(eMemoryDevice value);

template<class T> LIME_API T ToEnumClass(const std::string& str);

} // namespace lime
