#include "privates.h"

#include "limesuiteng/embedded/loglevel.h"

#include "csr.h"
#include "spi.h"
#include "lms7002m_context.h"

#ifdef __KERNEL__
    #include <linux/kernel.h>
    #include <linux/printk.h>
    #include <asm/delay.h>
#else
    #include <stdarg.h>
    #include <stdio.h>

    #ifdef _MSC_VER
        #include <windows.h>
    #else
        #include <unistd.h>
    #endif
#endif

#if LMS7002M_LOG_ENABLE
void lms7002m_log(lms7002m_context* context, lime_LogLevel level, const char* format, ...)
{
    if (context->hooks.log == NULL)
        return;

    char buff[4096];

    va_list args;
    va_start(args, format);
    vsnprintf(buff, sizeof(buff), format, args);
    va_end(args);

    context->hooks.log(level, buff, context->hooks.log_userData);
}
#endif

void lms7002m_sleep(long timeInMicroseconds)
{
#ifdef _MSC_VER
    Sleep(timeInMicroseconds / 1000);
#else
    #ifdef __KERNEL__
    usleep_range(timeInMicroseconds, timeInMicroseconds + 50);
    #else
    usleep(timeInMicroseconds);
    #endif // __KERNEL__
#endif
}

int32_t clamp_int(int32_t value, int32_t min, int32_t max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

uint32_t clamp_uint(uint32_t value, uint32_t min, uint32_t max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

void lms7002m_trigger_rising_edge(lms7002m_context* self, const struct lms7002m_csr* reg)
{
    lms7002m_spi_modify_csr(self, *reg, 0);
    lms7002m_spi_modify_csr(self, *reg, 1);
}
