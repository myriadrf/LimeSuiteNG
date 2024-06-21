#pragma once

#include <iomanip>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

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

template<typename strView> std::vector<strView> SplitString(strView string, strView delimiter)
{
    std::vector<strView> ret;

    auto position{ string.find(delimiter) };

    while (position != strView::npos)
    {
        if (position != 0)
        {
            ret.push_back(string.substr(0, position));
        }

        string = string.substr(position + delimiter.size());
        position = string.find(delimiter);
    }

    if (string.size() > 0)
    {
        ret.push_back(string);
    }
    return ret;
}

} // namespace lime
