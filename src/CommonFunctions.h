#pragma once

#include <string>
#include <sstream>
#include <iomanip>

#include "limesuiteng/commonTypes.h"

namespace lime {

const std::string strFormat(const char* format, ...);
std::string ToString(TRXDir dir);

template<typename T> std::string intToHex(T i)
{
    std::stringstream stream;
    stream << std::setfill('0') << std::setw(sizeof(T) * 2) << std::hex << i;
    return stream.str();
}

} // namespace lime
