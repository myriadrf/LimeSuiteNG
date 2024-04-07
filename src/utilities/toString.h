#pragma once

#include "limesuiteng/types.h"
#include "limesuiteng/OpStatus.h"

namespace lime {

/// @brief Converts a given TRXDir value into a human readable C-string.
/// @param value The value to convert.
/// @return The C-string representing the direction.
const char* ToCString(TRXDir dir);

/// @brief Converts a given OpStatus value into a human readable C-string.
/// @param value The value to convert.
/// @return The C-string representing the status.
const char* ToCString(OpStatus value);

}
