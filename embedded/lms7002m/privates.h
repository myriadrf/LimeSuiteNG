#ifndef LMS7002M_PRIVATES_H
#define LMS7002M_PRIVATES_H

#include "limesuiteng/embedded/loglevel.h"
#include "limesuiteng/embedded/result.h"
#include "limesuiteng/embedded/types.h"

struct lms7002m_context;
struct lms7002m_csr;

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __unix__
    #define FORMAT_ATTR(type, fmt_str, fmt_param) __attribute__((format(type, fmt_str, fmt_param)))
#else
    #define FORMAT_ATTR(type, fmt_str, fmt_param)
#endif

#define LMS7002M_LOG_ENABLE 0
#if LMS7002M_LOG_ENABLE
void FORMAT_ATTR(printf, 3, 4) lms7002m_log(struct lms7002m_context* context, lime_LogLevel level, const char* format, ...);

    #define LMS7002M_LOG(context, lime_LogLevel, format, ...) \
        do \
        { \
            lms7002m_log(context, lime_LogLevel, format, __VA_ARGS__); \
        } while (0)

#else
    #define LMS7002M_LOG(context, lime_LogLevel, format, ...)
#endif // LMS7002M_LOG_ENABLE

void lms7002m_sleep(long timeInMicroseconds);

uint8_t lms7002m_minimum_tune_score_index(int tuneScore[], int count);

// calibration
void lms7002m_save_chip_state(struct lms7002m_context* self, bool wr);
void lms7002m_trigger_rising_edge(struct lms7002m_context* self, const struct lms7002m_csr* reg);

// clamps
int32_t clamp_int(int32_t value, int32_t min, int32_t max);
uint32_t clamp_uint(uint32_t value, uint32_t min, uint32_t max);

#ifdef __cplusplus
} // extern C
#endif

#endif // LMS7002M_PRIVATES_H
