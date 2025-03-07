#ifndef LIME_EXPECT_MACROS_H
#define LIME_EXPECT_MACROS_H

#include "privates.h"

#ifndef NDEBUG // warn about unexpected conditions
    #define EXPECT(context, condition) \
        do \
        { \
            if (!(condition)) \
            { \
                LMS7002M_LOG(self, lime_LogLevel_Error, "%s:%i Unmet expectation: (" #condition ")", __FILE__, __LINE__); \
            } \
        } while (0)
#else
    #define EXPECT(context, condition) // do nothing
#endif

#endif // LIME_EXPECT_MACROS_H
