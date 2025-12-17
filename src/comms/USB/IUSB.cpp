#include "IUSB.h"

#include <string>

using namespace std::literals::string_view_literals;

namespace lime {

std::string_view ToString(USBSpeed speed)
{
    switch (speed)
    {
    case USBSpeed::USB1_0:
        return "USB1.0"sv;
    case USBSpeed::USB1_1:
        return "USB1.1"sv;
    case USBSpeed::USB2_0:
        return "USB2.0"sv;
    case USBSpeed::USB3_0:
        return "USB3.0"sv;
    case USBSpeed::USB3_1:
        return "USB3.1"sv;
    default:
        return "USB_Unknown";
    }
}

} // namespace lime
