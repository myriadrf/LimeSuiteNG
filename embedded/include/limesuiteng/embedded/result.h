#ifndef LIME_RESULT_H
#define LIME_RESULT_H

typedef enum lime_Result {
    lime_Result_Success = 0,
    lime_Result_Error = -1,
    lime_Result_NotImplemented = -2,
    lime_Result_IOFailure = -3,
    lime_Result_InvalidValue = -4,
    lime_Result_FileNotFound = -5,
    lime_Result_OutOfRange = -6,
    lime_Result_NotSupported = -7,
    lime_Result_Timeout = -8,
    lime_Result_Busy = -9,
    lime_Result_Aborted = -10,
    lime_Result_PermissionDenied = -11,
    lime_Result_NotConnected = -12,
} lime_Result;

#endif // LIME_RESULT_H
