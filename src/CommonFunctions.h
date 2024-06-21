#pragma once

#include <string>
#include <sstream>
#include <iomanip>

#include "limesuiteng/types.h"

namespace lime {

const std::string strFormat(const char* format, ...);

template<typename T> std::string intToHex(T i, bool uppercase = false)
{
    std::stringstream stream;

    if (uppercase)
    {
        stream << std::uppercase;
    }

    stream << std::setfill('0') << std::setw(sizeof(T) * 2) << std::hex << i;
    return stream.str();
}

} // namespace lime
