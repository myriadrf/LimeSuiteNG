#include "privates.h"

#include "csr.h"
#include "limesuiteng/embedded/loglevel.h"
#include "lms7002m_context.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef _MSC_VER
    #include <windows.h>
#else
    #include <time.h>
#endif

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

static void lms7002m_log_va(lms7002m_context* context, lime_LogLevel level, const char* format, va_list args)
{
    if (context->hooks.log == NULL)
        return;

    char buff[4096];
    vsnprintf(buff, sizeof(buff), format, args);
    context->hooks.log(level, buff, context->hooks.log_userData);
}

lime_Result lms7002m_report_error(lms7002m_context* context, lime_Result result, const char* format, ...)
{
    if (context->hooks.log == NULL)
        return result;

    va_list args;
    va_start(args, format);
    lms7002m_log_va(context, lime_LogLevel_Error, format, args);
    va_end(args);

    return result;
}

void lms7002m_sleep(long timeInMicroseconds)
{
#ifdef _MSC_VER
    Sleep(timeInMicroseconds / 1000);
#else
    struct timespec time;
    time.tv_sec = 0;
    time.tv_nsec = timeInMicroseconds * 1000;

    // POSIX function, non-standard C
    nanosleep(&time, NULL);
#endif
}

void lms7002m_spi_write(lms7002m_context* self, uint16_t address, uint16_t value)
{
    uint32_t mosi = address << 16 | value;
    mosi |= 1 << 31;
    self->hooks.spi16_transact(&mosi, NULL, 1, self->hooks.spi16_userData);
}

uint16_t lms7002m_spi_read(lms7002m_context* self, uint16_t address)
{
    uint32_t mosi = address << 16;
    uint32_t miso = 0;
    self->hooks.spi16_transact(&mosi, &miso, 1, self->hooks.spi16_userData);
    return miso & 0xFFFF;
}

lime_Result lms7002m_spi_modify(lms7002m_context* self, uint16_t address, uint8_t msb, uint8_t lsb, uint16_t value)
{
    uint16_t spiDataReg = lms7002m_spi_read(self, address);
    uint16_t spiMask = (~(~0u << (msb - lsb + 1))) << (lsb); // creates bit mask
    spiDataReg = (spiDataReg & (~spiMask)) | ((value << lsb) & spiMask); //clear bits
    lms7002m_spi_write(self, address, spiDataReg);
    return lime_Result_Success;
}

lime_Result lms7002m_spi_modify_csr(lms7002m_context* self, const lms7002m_csr csr, uint16_t value)
{
    return lms7002m_spi_modify(self, csr.address, csr.msb, csr.lsb, value);
}

uint16_t lms7002m_spi_read_bits(lms7002m_context* self, uint16_t address, uint8_t msb, uint8_t lsb)
{
    uint16_t regVal = lms7002m_spi_read(self, address);
    return (regVal & (~(~0u << (msb + 1)))) >> lsb; //shift bits to LSB
}

uint16_t lms7002m_spi_read_csr(lms7002m_context* self, const lms7002m_csr csr)
{
    return lms7002m_spi_read_bits(self, csr.address, csr.msb, csr.lsb);
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
