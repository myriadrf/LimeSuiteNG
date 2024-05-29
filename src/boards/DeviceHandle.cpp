#include "limesuiteng/DeviceHandle.h"
#include <cctype>
#include <string>
#include <map>
#include <iso646.h> // alternative operators for visual c++: not, and, or...
using namespace lime;
using namespace std::literals::string_literals;

/*******************************************************************
 * String parsing helpers
 ******************************************************************/
static std::string_view trim(const std::string_view s)
{
    std::string_view out = s;
    while (not out.empty() and std::isspace(out[0]))
        out = out.substr(1);
    while (not out.empty() and std::isspace(out[out.size() - 1]))
        out = out.substr(0, out.size() - 1);
    return out;
}

static std::map<std::string, std::string> argsToMap(const std::string_view args)
{
    std::map<std::string, std::string> kwmap;

    bool inKey = true;
    std::string key, val;
    for (size_t i = 0; i < args.size(); i++)
    {
        const char ch = args[i];
        if (inKey)
        {
            if (ch == '=')
                inKey = false;
            else if (ch == ',')
                inKey = true;
            else
                key += ch;
        }
        else
        {
            if (ch == ',')
                inKey = true;
            else
                val += ch;
        }
        if ((inKey and not val.empty()) or ((i + 1) == args.size()))
        {
            key = trim(key);
            val = trim(val);
            if (not key.empty())
                kwmap[key] = val;
            key = ""s;
            val = ""s;
        }
    }

    return kwmap;
}

/*******************************************************************
 * Connection handle implementation
 ******************************************************************/
DeviceHandle::DeviceHandle()
    : media()
    , name()
    , addr()
    , serial()
{
}

DeviceHandle::DeviceHandle(const std::string& args)
    : media()
    , name()
    , addr()
    , serial()
{
    auto kwmap = argsToMap("name="s + args); // Append name= since it was stripped in serialize
    if (kwmap.count("media"s) != 0)
        media = kwmap.at("media"s);
    if (kwmap.count("name"s) != 0)
        name = kwmap.at("name"s);
    if (kwmap.count("addr"s) != 0)
        addr = kwmap.at("addr"s);
    if (kwmap.count("serial"s) != 0)
        serial = kwmap.at("serial"s);
}

std::string DeviceHandle::Serialize(void) const
{
    std::string out = ""s;
    if (not name.empty())
        out += name;
    if (not media.empty())
        out += ", media="s + media;
    if (not addr.empty())
        out += ", addr="s + addr;
    if (not serial.empty())
        out += ", serial="s + serial;

    return out;
}

std::string DeviceHandle::ToString(void) const
{
    //name and media format
    std::string out(name);
    if (not media.empty())
        out += " ["s + media + "]"s;

    // Remove leading zeros for a displayable serial
    std::string_view trimmedSerial{ serial };
    while (not trimmedSerial.empty() and trimmedSerial.at(0) == '0')
        trimmedSerial = trimmedSerial.substr(1);

    if (not trimmedSerial.empty())
        out += " "s + std::string{ trimmedSerial };

    //backup condition if we are empty somehow
    if (out.empty())
        out = this->Serialize();

    return out;
}

bool DeviceHandle::IsEqualIgnoringEmpty(const DeviceHandle& hint) const
{
    if (!hint.media.empty() &&  media.find(hint.media) == std::string::npos)
    {
        return false;
    }

    if (!hint.name.empty() && name.find(hint.name) == std::string::npos)
    {
        return false;
    }

    if (!hint.addr.empty() && addr.find(hint.addr) == std::string::npos)
    {
        return false;
    }

    if (!hint.serial.empty() && serial.find(hint.serial) == std::string::npos)
    {
        return false;
    }

    return true;
}

bool operator==(const DeviceHandle& lhs, const DeviceHandle& rhs)
{
    return lhs.Serialize() == rhs.Serialize();
}
