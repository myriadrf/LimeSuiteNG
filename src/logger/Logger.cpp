/**
@file Logger.cpp
@author Lime Microsystems
@brief API for reporting error codes and error messages.
*/

#include "limesuiteng/Logger.h"
#include "LoggerInternal.h"

#include <iostream>
#include <string>

#include "utilities/toString.h"

using namespace std::literals::string_literals;

namespace lime {

LogHandler Logger::logHandler{ Logger::defaultLogHandler };
thread_local int Logger::_reportedErrorCode{ 0 };
thread_local std::string Logger::_reportedErrorMessage{ ""s };

void Logger::defaultLogHandler(const LogLevel level, const std::string& message)
{
    if (level > LogLevel::Warning)
        return;
    std::cerr << message << std::endl;
}

void Logger::logHandlerWrapper(const LogLevel level, const std::string& message)
{
    logHandlerCString(level, message.c_str());
}

void registerLogHandler(const LogHandler handler)
{
    Logger::logHandler = handler ? handler : Logger::defaultLogHandler;
    Logger::logHandlerCString = handler ? Logger::logHandlerCStringWrapper : Logger::defaultLogHandlerCString;
}

const std::string& GetLastErrorMessage(void)
{
    return Logger::_reportedErrorMessage;
}

void critical(const std::string& text)
{
    log(LogLevel::Critical, text);
}

int error(const std::string& text)
{
    log(LogLevel::Error, text);
    return -1;
}

void warning(const std::string& text)
{
    log(LogLevel::Warning, text);
}

void info(const std::string& text)
{
    log(LogLevel::Info, text);
}

void debug(const std::string& text)
{
    log(LogLevel::Debug, text);
}

void log(const LogLevel level, const std::string& text)
{
    Logger::logHandler(level, text);
}

OpStatus ReportError(const OpStatus errnum)
{
    return ReportError(errnum, ToString(errnum));
}

template<class T> T ReportErrorTemplate(const T errnum, const std::string& text)
{
    Logger::_reportedErrorMessage = text;
    log(LogLevel::Error, text);
    return errnum;
}

int ReportError(const int errnum, const std::string& text)
{
    return ReportErrorTemplate(errnum, text);
}

OpStatus ReportError(const OpStatus errnum, const std::string& text)
{
    return ReportErrorTemplate(errnum, text);
}

} // namespace lime
