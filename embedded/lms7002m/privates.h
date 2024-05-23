#ifndef LMS7002M_PRIVATES_H
#define LMS7002M_PRIVATES_H

#include "csr.h"
#include "limesuiteng/embedded/loglevel.h"
#include "limesuiteng/embedded/result.h"

#include <stdint.h>

struct lms7002m_context;

#define LOG_D(context, format, ...) \
    do \
    { \
        lms7002m_log(context, lime_LogLevel_Debug, format, __VA_ARGS__); \
    } while (0)

void lms7002m_log(struct lms7002m_context* context, lime_LogLevel level, const char* format, ...);
lime_Result lms7002m_report_error(struct lms7002m_context* context, lime_Result result, const char* format, ...);
void lms7002m_sleep(long timeInMicroseconds);

void lms7002m_spi_write(struct lms7002m_context* self, uint16_t address, uint16_t value);
uint16_t lms7002m_spi_read(struct lms7002m_context* self, uint16_t address);
lime_Result lms7002m_spi_modify(struct lms7002m_context* self, uint16_t address, uint8_t msb, uint8_t lsb, uint16_t value);
lime_Result lms7002m_spi_modify_csr(struct lms7002m_context* self, const lms7002m_csr csr, uint16_t value);
uint16_t lms7002m_spi_read_bits(struct lms7002m_context* self, uint16_t address, uint8_t msb, uint8_t lsb);
uint16_t lms7002m_spi_read_csr(struct lms7002m_context* self, const lms7002m_csr csr);

uint8_t lms7002m_minimum_tune_score_index(int tuneScore[], int count);

uint16_t clamp_uint(uint16_t value, uint16_t min, uint16_t max);
double clamp_double(double value, double min, double max);

#endif // LMS7002M_PRIVATES_H
