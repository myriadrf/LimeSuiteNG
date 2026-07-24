#ifndef LIME_LOGGERINTERNAL_H
#define LIME_LOGGERINTERNAL_H

#include "limesuiteng/Logger.h"

#include <mutex>

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

    // guards the handler pair below, which is swapped as a unit by registerLogHandler
    // while worker threads read it; copy the handler under the lock, call it outside
    static std::mutex handlerMutex;
    static LogHandlerCString logHandlerCString;
    static LogHandler logHandler;

    static thread_local int _reportedErrorCode;
    static thread_local std::string _reportedErrorMessage;
};

} // namespace lime

#endif // LIME_LOGGERINTERNAL_H