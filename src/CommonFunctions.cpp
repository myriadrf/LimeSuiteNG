#include "CommonFunctions.h"
#include <stdarg.h>
#include <string>

using namespace std::literals::string_literals;

namespace lime {

static constexpr int BUFFER_SIZE = 256;

const std::string strFormat(const char* format, ...)
{
    char ctemp[BUFFER_SIZE];

    va_list args;

    va_start(args, format);
    std::vsnprintf(ctemp, BUFFER_SIZE, format, args);
    va_end(args);

    return std::string(ctemp);
}

} // namespace lime
