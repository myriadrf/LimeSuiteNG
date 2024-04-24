#ifndef LIME_LOGGERINTERNAL_H
#define LIME_LOGGERINTERNAL_H

#include "limesuiteng/Logger.h"

#ifdef _MSC_VER
    #define thread_local __declspec(thread)
#endif

namespace lime {

class Logger
{
  public:
    static void defaultLogHandlerCString(const LogLevel level, const char* message);
    static void logHandlerCStringWrapper(const LogLevel level, const char* message);

    static void defaultLogHandler(const LogLevel level, const std::string& message);
    static void logHandlerWrapper(const LogLevel level, const std::string& message);

    static LogHandlerCString logHandlerCString;
    static LogHandler logHandler;

    static thread_local int _reportedErrorCode;
    static thread_local std::string _reportedErrorMessage;
};

} // namespace lime

#endif // LIME_LOGGERINTERNAL_H