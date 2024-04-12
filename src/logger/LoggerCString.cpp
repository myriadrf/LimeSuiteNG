#include "limesuiteng/Logger.h"
#include "LoggerInternal.h"

namespace lime {

LogHandlerCString Logger::logHandlerCString{ &defaultLogHandlerCString };

void Logger::defaultLogHandlerCString(const LogLevel level, const char* message)
{
    if (level > LogLevel::Error)
        return;
    fprintf(stderr, "%s\n", message);
}

void Logger::logHandlerCStringWrapper(const LogLevel level, const char* message)
{
    Logger::logHandler(level, message);
}

void registerLogHandler(const LogHandlerCString handler)
{
    Logger::logHandlerCString = handler ? handler : Logger::defaultLogHandlerCString;
    Logger::logHandler = handler ? Logger::logHandlerWrapper : Logger::defaultLogHandler;
}

const char* GetLastErrorMessageCString(void)
{
    return Logger::_reportedErrorMessage.c_str();
}

void log(const LogLevel level, const char* format, va_list argList)
{
    char buff[4096];
    int ret = vsnprintf(buff, sizeof(buff), format, argList);
    if (ret > 0)
        Logger::logHandlerCString(level, buff);
}

void critical(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    log(LogLevel::Critical, format, args);
    va_end(args);
}

int error(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    log(LogLevel::Error, format, args);
    va_end(args);
    return -1;
}

void warning(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    log(LogLevel::Warning, format, args);
    va_end(args);
}

void info(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    log(LogLevel::Info, format, args);
    va_end(args);
}

void debug(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    log(LogLevel::Debug, format, args);
    va_end(args);
}

void log(const LogLevel level, const char* format, ...)
{
    va_list args;
    va_start(args, format);
    log(level, format, args);
    va_end(args);
}

constexpr std::size_t MAX_MSG_LEN = 1024;
thread_local char _reportedErrorMessageCStringBuffer[MAX_MSG_LEN];
template<class T> inline T ReportErrorTemplate(const T errnum, const char* format, va_list argList)
{
    Logger::_reportedErrorCode = static_cast<int>(errnum);

    int ret = vsnprintf(_reportedErrorMessageCStringBuffer, MAX_MSG_LEN, format, argList);
    if (ret > 0)
    {
        Logger::_reportedErrorMessage = _reportedErrorMessageCStringBuffer;
    }

    log(LogLevel::Error, Logger::_reportedErrorMessage);
    return errnum;
}

int ReportError(const int errnum, const char* format, va_list argList)
{
    return ReportErrorTemplate(errnum, format, argList);
}

OpStatus ReportError(const OpStatus errnum, const char* format, va_list argList)
{
    return ReportErrorTemplate(errnum, format, argList);
}

OpStatus ReportError(const OpStatus errnum, const char* format, ...)
{
    va_list argList;
    va_start(argList, format);
    OpStatus status = ReportError(errnum, format, argList);
    va_end(argList);
    return status;
}

int ReportError(const int errnum, const char* format, ...)
{
    va_list argList;
    va_start(argList, format);
    int status = ReportError(errnum, format, argList);
    va_end(argList);
    return status;
}

} // namespace lime