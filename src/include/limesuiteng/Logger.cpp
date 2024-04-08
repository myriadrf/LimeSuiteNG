/**
@file Logger.cpp
@author Lime Microsystems
@brief API for reporting error codes and error messages.
*/

#include "Logger.h"
#include <cstdio>
#include <cstring> //strerror

#include "utilities/toString.h"

#ifdef _MSC_VER
    #define thread_local __declspec(thread)
#endif

#ifdef __APPLE__
    #define thread_local __thread
#endif

namespace lime {

#define MAX_MSG_LEN 1024
thread_local int _reportedErrorCode;
thread_local char _reportedErrorMessage[MAX_MSG_LEN];

const char* GetLastErrorMessage(void)
{
    return _reportedErrorMessage;
}

OpStatus ReportError(const OpStatus errnum)
{
    return ReportError(errnum, ToCString(errnum));
}

int ReportError(const int errnum, const char* format, va_list argList)
{
    _reportedErrorCode = errnum;
    vsnprintf(_reportedErrorMessage, MAX_MSG_LEN, format, argList);
    log(LogLevel::Error, _reportedErrorMessage);
    return errnum;
}

OpStatus ReportError(const OpStatus errnum, const char* format, va_list argList)
{
    vsnprintf(_reportedErrorMessage, MAX_MSG_LEN, format, argList);
    log(LogLevel::Error, _reportedErrorMessage);
    return errnum;
}

static void defaultLogHandler(const LogLevel level, const char* message)
{
    if (level > LogLevel::Error)
        return;
    fprintf(stderr, "%s\n", message);
}

static LogHandler logHandler(&defaultLogHandler);

void log(const LogLevel level, const char* format, va_list argList)
{
    char buff[4096];
    int ret = vsnprintf(buff, sizeof(buff), format, argList);
    if (ret > 0)
        logHandler(level, buff);
}

void registerLogHandler(const LogHandler handler)
{
    logHandler = handler ? handler : defaultLogHandler;
}

void log(const LogLevel level, const char* format, ...)
{
    va_list args;
    va_start(args, format);
    log(level, format, args);
    va_end(args);
}

void critical(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    log(LogLevel::Critical, format, args);
    va_end(args);
}

void critical(const std::string& text)
{
    log(LogLevel::Critical, text);
}

int error(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    log(LogLevel::Error, format, args);
    va_end(args);
    return -1;
}

int error(const std::string& text)
{
    log(LogLevel::Error, text);
    return -1;
}

void warning(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    log(LogLevel::Warning, format, args);
    va_end(args);
}

void warning(const std::string& text)
{
    log(LogLevel::Warning, text);
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

void debug(const std::string& text)
{
    log(LogLevel::Debug, text);
}

void info(const std::string& text)
{
    log(LogLevel::Info, text);
}

//! Log a message with formatting and specified logging level
void log(const LogLevel level, const std::string& text)
{
    log(level, "%s", text.c_str());
}

OpStatus ReportError(const OpStatus errnum, const char* format, ...)
{
    va_list argList;
    va_start(argList, format);
    OpStatus status = ReportError(errnum, format, argList);
    va_end(argList);
    return status;
}

} // namespace lime
