#ifndef LIME_LOGLEVEL_H
#define LIME_LOGLEVEL_H

typedef enum lime_LogLevel {
    lime_LogLevel_Critical, //!< A critical error. The application might not be able to continue running successfully.
    lime_LogLevel_Error, //!< An error. An operation did not complete successfully, but the application as a whole is not affected.
    lime_LogLevel_Warning, //!< A warning. An operation completed with an unexpected result.
    lime_LogLevel_Info, //!< An informational message, usually denoting the successful completion of an operation.
    lime_LogLevel_Verbose, //!< An informational message, detailing intermediate step results.
    lime_LogLevel_Debug, //!< A debugging message, only shown in Debug configuration.
} lime_LogLevel;

#endif // LIME_LOGLEVEL_H
