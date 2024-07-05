#ifndef LMS7002M_PRIVATES_H
#define LMS7002M_PRIVATES_H

#include "csr.h"
#include "limesuiteng/embedded/loglevel.h"
#include "limesuiteng/embedded/result.h"

#include <stdbool.h>
#include <stdint.h>

struct lms7002m_context;

#define LMS7002M_LOG_ENABLE 1
#if LMS7002M_LOG_ENABLE
    #define LMS7002M_LOG(context, lime_LogLevel, format, ...) \
        do \
        { \
            lms7002m_log(context, lime_LogLevel, format, __VA_ARGS__); \
        } while (0)
#else
    #define LMS7002M_LOG(context, lime_LogLevel, format, ...)
#endif

#ifdef __unix__
    #define FORMAT_ATTR(type, fmt_str, fmt_param) __attribute__((format(type, fmt_str, fmt_param)))
#else
    #define FORMAT_ATTR(type, fmt_str, fmt_param)
#endif

void FORMAT_ATTR(printf, 3, 4) lms7002m_log(struct lms7002m_context* context, lime_LogLevel level, const char* format, ...);
lime_Result FORMAT_ATTR(printf, 3, 4)
    lms7002m_report_error(struct lms7002m_context* context, lime_Result result, const char* format, ...);
void lms7002m_sleep(long timeInMicroseconds);

void lms7002m_spi_write(struct lms7002m_context* self, uint16_t address, uint16_t value);
uint16_t lms7002m_spi_read(struct lms7002m_context* self, uint16_t address);
lime_Result lms7002m_spi_modify(struct lms7002m_context* self, uint16_t address, uint8_t msb, uint8_t lsb, uint16_t value);
lime_Result lms7002m_spi_modify_csr(struct lms7002m_context* self, const lms7002m_csr csr, uint16_t value);
uint16_t lms7002m_spi_read_bits(struct lms7002m_context* self, uint16_t address, uint8_t msb, uint8_t lsb);
uint16_t lms7002m_spi_read_csr(struct lms7002m_context* self, const lms7002m_csr csr);

uint8_t lms7002m_minimum_tune_score_index(int tuneScore[], int count);

// calibration
void lms7002m_save_chip_state(struct lms7002m_context* self, bool wr);
void lms7002m_flip_rising_edge(struct lms7002m_context* self, const lms7002m_csr* reg);

// clamps
int32_t clamp_int(int32_t value, int32_t min, int32_t max);
uint32_t clamp_uint(uint32_t value, uint32_t min, uint32_t max);

#endif // LMS7002M_PRIVATES_H
