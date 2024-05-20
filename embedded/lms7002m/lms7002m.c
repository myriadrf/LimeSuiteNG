#include "limesuiteng/embedded/lms7002m/lms7002m.h"
#include "limesuiteng/embedded/loglevel.h"
#include "lms7002m_context.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "csr.h"

#define CGEN_MAX_FREQ 640e6

static void lms7002m_log(lms7002m_context* context, lime_LogLevel level, const char* format, ...)
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

#define LOG_D(context, format, ...) \
    do \
    { \
        lms7002m_log(context, lime_LogLevel_Debug, format, __VA_ARGS__); \
    } while (0)

static lime_Result lms7002m_report_error(lms7002m_context* context, lime_Result result, const char* format, ...)
{
    if (context->hooks.log == NULL)
        return result;

    va_list args;
    va_start(args, format);
    lms7002m_log(context, lime_LogLevel_Error, format, args);
    va_end(args);

    return result;
}

struct lms7002m_context* lms7002m_create(const lms7002m_hooks* hooks)
{
    lms7002m_context* self = malloc(sizeof(lms7002m_context));
    memcpy(&self->hooks, hooks, sizeof(lms7002m_hooks));
    return self;
}

void lms7002m_destroy(lms7002m_context* context)
{
    if (context)
        free(context);
}

static void lms7002m_spi_write(lms7002m_context* self, uint16_t address, uint16_t value)
{
    uint32_t mosi = address << 16 | value;
    mosi |= 1 << 31;
    self->hooks.spi16_transact(&mosi, NULL, 1, self->hooks.spi16_userData);
}

static uint16_t lms7002m_spi_read(lms7002m_context* self, uint16_t address)
{
    uint32_t mosi = address << 16;
    uint32_t miso = 0;
    self->hooks.spi16_transact(&mosi, &miso, 1, self->hooks.spi16_userData);
    return miso & 0xFFFF;
}

static lime_Result lms7002m_spi_modify(lms7002m_context* self, uint16_t address, uint8_t msb, uint8_t lsb, uint16_t value)
{
    uint16_t spiDataReg = lms7002m_spi_read(self, address);
    uint16_t spiMask = (~(~0u << (msb - lsb + 1))) << (lsb); // creates bit mask
    spiDataReg = (spiDataReg & (~spiMask)) | ((value << lsb) & spiMask); //clear bits
    lms7002m_spi_write(self, address, spiDataReg);
    return lime_Result_Success;
}

static lime_Result lms7002m_spi_modify_csr(lms7002m_context* self, const lms7002m_csr csr, uint16_t value)
{
    return lms7002m_spi_modify(self, csr.address, csr.msb, csr.lsb, value);
}

static uint16_t lms7002m_spi_read_bits(lms7002m_context* self, uint16_t address, uint8_t msb, uint8_t lsb)
{
    uint16_t regVal = lms7002m_spi_read(self, address);
    return (regVal & (~(~0u << (msb + 1)))) >> lsb; //shift bits to LSB
}

static uint16_t lms7002m_spi_read_csr(lms7002m_context* self, const lms7002m_csr csr)
{
    return lms7002m_spi_read_bits(self, csr.address, csr.msb, csr.lsb);
}

static uint16_t clamp(uint16_t value, uint16_t min, uint16_t max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

uint8_t lms7002m_get_active_channel(lms7002m_context* self)
{
    return lms7002m_spi_read_csr(self, LMS7002M_MAC);
}

lime_Result lms7002m_set_active_channel(lms7002m_context* self, const uint8_t channel)
{
    if (channel == lms7002m_get_active_channel(self))
        return lime_Result_Success;
    return lms7002m_spi_modify_csr(self, LMS7002M_MAC, channel);
}

static uint8_t check_cgen_csw(lms7002m_context* self, uint8_t csw)
{
    lms7002m_spi_modify_csr(self, LMS7002M_CSW_VCO_CGEN, csw); //write CSW value
    //std::this_thread::sleep_for(std::chrono::microseconds(50)); //comparator settling time
    return lms7002m_spi_read_bits(self, LMS7002M_VCO_CMPHO_CGEN.address, 13, 12); //read comparators
}

lime_Result lms7002m_tune_cgen_vco(lms7002m_context* self)
{
    // Initialization activate VCO and comparator
    lime_Result result = lms7002m_spi_modify(self, LMS7002M_PD_VCO_CGEN.address, 2, 1, 0);
    if (result != lime_Result_Success)
        return result;

    //find lock
    int csw = 127;
    for (int step = 64; step > 0; step >>= 1)
    {
        uint8_t cmphl = check_cgen_csw(self, csw);
        if (cmphl == 0)
            csw += step;
        else if (cmphl == 3)
            csw -= step;
        else
            break;
    }
    //search around (+/-7) to determine lock interval
    //number of iterations could be reduced in some cases by narrowing down the search interval in find lock phase
    int cswLow = csw, cswHigh = csw;
    for (int step = 4; step > 0; step >>= 1)
        if (check_cgen_csw(self, cswLow - step) != 0)
            cswLow = cswLow - step;
    for (int step = 4; step > 0; step >>= 1)
        if (check_cgen_csw(self, cswHigh + step) == 2)
            cswHigh = cswHigh + step;

    // lime::debug("csw %d; interval [%d, %d]", (cswHigh + cswLow) / 2, cswLow, cswHigh);
    uint8_t cmphl = check_cgen_csw(self, (cswHigh + cswLow) / 2);
    if (cmphl == 2)
        return lime_Result_Success;
    // lime::error("TuneVCO(CGEN) - failed to lock (cmphl!=%d)", cmphl);
    return lime_Result_Error;
}

float lms7002m_get_reference_clock(struct lms7002m_context* context)
{
    return context->reference_clock_hz;
}

lime_Result lms7002m_set_reference_clock(struct lms7002m_context* context, float frequency_Hz)
{
    if (frequency_Hz <= 0)
        return lime_Result_InvalidValue;

    context->reference_clock_hz = frequency_Hz;
    return lime_Result_Success;
}

static const float gCGEN_VCO_frequencies[2] = { 1930e6, 2940e6 };
lime_Result lms7002m_set_frequency_cgen(lms7002m_context* self, float freq_Hz)
{
    if (freq_Hz > CGEN_MAX_FREQ)
        return lms7002m_report_error(self, lime_Result_OutOfRange, "requested CGEN frequency too high");

    float dFvco;
    float dFrac;

    const float refClk = lms7002m_get_reference_clock(self);

    //VCO frequency selection according to F_CLKH
    uint16_t iHdiv_high = (gCGEN_VCO_frequencies[1] / 2 / freq_Hz) - 1;
    uint16_t iHdiv_low = (gCGEN_VCO_frequencies[0] / 2 / freq_Hz);
    uint16_t iHdiv = (iHdiv_low + iHdiv_high) / 2;
    iHdiv = iHdiv > 255 ? 255 : iHdiv;
    dFvco = 2 * (iHdiv + 1) * freq_Hz;
    if (dFvco <= gCGEN_VCO_frequencies[0] || dFvco >= gCGEN_VCO_frequencies[1])
        return lms7002m_report_error(
            self, lime_Result_Error, "SetFrequencyCGEN(%g MHz) - cannot deliver requested frequency", freq_Hz / 1e6);

    //Integer division
    uint16_t gINT = (uint16_t)(dFvco / refClk - 1);

    //Fractional division
    dFrac = dFvco / refClk - (uint32_t)(dFvco / refClk);
    uint32_t gFRAC = (uint32_t)(dFrac * 1048576);

    lms7002m_spi_modify_csr(self, LMS7002M_INT_SDM_CGEN, gINT); //INT_SDM_CGEN
    lms7002m_spi_modify(self, 0x0087, 15, 0, gFRAC & 0xFFFF); //INT_SDM_CGEN[15:0]
    lms7002m_spi_modify(self, 0x0088, 3, 0, gFRAC >> 16); //INT_SDM_CGEN[19:16]
    lms7002m_spi_modify_csr(self, LMS7002M_DIV_OUTCH_CGEN, iHdiv); //DIV_OUTCH_CGEN

    LOG_D(self, "INT %d, FRAC %d, DIV_OUTCH_CGEN %d", gINT, gFRAC, iHdiv);
    LOG_D(self, "VCO %.2f MHz, RefClk %.2f MHz", dFvco / 1e6, refClk / 1e6);

    if (lms7002m_tune_cgen_vco(self) != lime_Result_Success)
    {
        //return lms7002m_report_error(self, lime_Result_Error, "SetFrequencyCGEN(%g MHz) failed", freq_Hz / 1e6);
        return lms7002m_report_error(self, lime_Result_Error, "SetFrequencyCGEN failed");
    }

    if (self->hooks.on_cgen_frequency_changed)
        self->hooks.on_cgen_frequency_changed(self->hooks.on_cgen_frequency_changed_userData);

    return lime_Result_Success;
}

lime_Result lms7002m_set_rbbpga_db(lms7002m_context* self, const float value, const uint8_t channel)
{
    uint8_t savedChannel = lms7002m_get_active_channel(self);
    lms7002m_set_active_channel(self, channel);

    int g_pga_rbb = clamp(lroundf(value) + 12, 0, 31);
    lime_Result ret = lms7002m_spi_modify_csr(self, LMS7002M_G_PGA_RBB, g_pga_rbb);

    int rcc_ctl_pga_rbb = (430.0 * pow(0.65, (g_pga_rbb / 10.0)) - 110.35) / 20.4516 + 16;

    int c_ctl_pga_rbb = 0;
    if (0 <= g_pga_rbb && g_pga_rbb < 8)
        c_ctl_pga_rbb = 3;
    else if (8 <= g_pga_rbb && g_pga_rbb < 13)
        c_ctl_pga_rbb = 2;
    else if (13 <= g_pga_rbb && g_pga_rbb < 21)
        c_ctl_pga_rbb = 1;

    ret = lms7002m_spi_modify_csr(self, LMS7002M_RCC_CTL_PGA_RBB, rcc_ctl_pga_rbb);
    ret = lms7002m_spi_modify_csr(self, LMS7002M_C_CTL_PGA_RBB, c_ctl_pga_rbb);

    lms7002m_set_active_channel(self, savedChannel);
    return ret;
}

float lms7002m_get_rbbpga_db(lms7002m_context* self, const uint8_t channel)
{
    uint8_t savedChannel = lms7002m_get_active_channel(self);
    lms7002m_set_active_channel(self, channel);

    uint16_t g_pga_rbb = lms7002m_spi_read_csr(self, LMS7002M_G_PGA_RBB);

    lms7002m_set_active_channel(self, savedChannel);
    return g_pga_rbb - 12;
}

lime_Result lms7002m_set_rfelna_db(lms7002m_context* self, const float value, const uint8_t channel)
{
    uint8_t savedChannel = lms7002m_get_active_channel(self);
    lms7002m_set_active_channel(self, channel);

    const double gmax = 30;
    double val = value - gmax;

    int g_lna_rfe = 1;
    if (val >= 0)
        g_lna_rfe = 15;
    else if (val >= -1)
        g_lna_rfe = 14;
    else if (val >= -2)
        g_lna_rfe = 13;
    else if (val >= -3)
        g_lna_rfe = 12;
    else if (val >= -4)
        g_lna_rfe = 11;
    else if (val >= -5)
        g_lna_rfe = 10;
    else if (val >= -6)
        g_lna_rfe = 9;
    else if (val >= -9)
        g_lna_rfe = 8;
    else if (val >= -12)
        g_lna_rfe = 7;
    else if (val >= -15)
        g_lna_rfe = 6;
    else if (val >= -18)
        g_lna_rfe = 5;
    else if (val >= -21)
        g_lna_rfe = 4;
    else if (val >= -24)
        g_lna_rfe = 3;
    else if (val >= -27)
        g_lna_rfe = 2;

    lime_Result ret = lms7002m_spi_modify_csr(self, LMS7002M_G_LNA_RFE, g_lna_rfe);
    lms7002m_set_active_channel(self, savedChannel);
    return ret;
}

float lms7002m_get_rfelna_db(lms7002m_context* self, const uint8_t channel)
{
    uint8_t savedChannel = lms7002m_get_active_channel(self);
    lms7002m_set_active_channel(self, channel);

    const double gmax = 30;
    uint16_t g_lna_rfe = lms7002m_spi_read_csr(self, LMS7002M_G_LNA_RFE);

    float retval = 0.0;
    const int value_to_minus[16] = { 0, 30, 27, 24, 21, 18, 15, 12, 9, 6, 5, 4, 3, 2, 1, 0 };

    if (g_lna_rfe > 0 && g_lna_rfe < 16)
    {
        retval = gmax - value_to_minus[g_lna_rfe];
    }

    lms7002m_set_active_channel(self, savedChannel);
    return retval;
}

lime_Result lms7002m_set_rfe_loopback_lna_db(lms7002m_context* self, const float gain, const uint8_t channel)
{
    uint8_t savedChannel = lms7002m_get_active_channel(self);
    lms7002m_set_active_channel(self, channel);

    const double gmax = 40;
    double val = gain - gmax;

    int g_rxloopb_rfe = 0;
    if (val >= 0)
        g_rxloopb_rfe = 15;
    else if (val >= -0.5)
        g_rxloopb_rfe = 14;
    else if (val >= -1)
        g_rxloopb_rfe = 13;
    else if (val >= -1.6)
        g_rxloopb_rfe = 12;
    else if (val >= -2.4)
        g_rxloopb_rfe = 11;
    else if (val >= -3)
        g_rxloopb_rfe = 10;
    else if (val >= -4)
        g_rxloopb_rfe = 9;
    else if (val >= -5)
        g_rxloopb_rfe = 8;
    else if (val >= -6.2)
        g_rxloopb_rfe = 7;
    else if (val >= -7.5)
        g_rxloopb_rfe = 6;
    else if (val >= -9)
        g_rxloopb_rfe = 5;
    else if (val >= -11)
        g_rxloopb_rfe = 4;
    else if (val >= -14)
        g_rxloopb_rfe = 3;
    else if (val >= -17)
        g_rxloopb_rfe = 2;
    else if (val >= -24)
        g_rxloopb_rfe = 1;

    lime_Result ret = lms7002m_spi_modify_csr(self, LMS7002M_G_RXLOOPB_RFE, g_rxloopb_rfe);

    lms7002m_set_active_channel(self, savedChannel);
    return ret;
}

float lms7002m_get_rfe_loopback_lna_db(lms7002m_context* self, const uint8_t channel)
{
    uint8_t savedChannel = lms7002m_get_active_channel(self);
    lms7002m_set_active_channel(self, channel);

    const double gmax = 40;
    uint16_t g_rxloopb_rfe = lms7002m_spi_read_csr(self, LMS7002M_G_RXLOOPB_RFE);

    float retval = 0.0;
    const float value_to_minus[16] = { 0, 24, 17, 14, 11, 9, 7.5, 6.2, 5, 4, 3, 2.4, 1.6, 1, 0.5, 0 };

    if (g_rxloopb_rfe > 0 && g_rxloopb_rfe < 16)
    {
        retval = gmax - value_to_minus[g_rxloopb_rfe];
    }

    lms7002m_set_active_channel(self, savedChannel);
    return retval;
}

lime_Result lms7002m_set_rfetia_db(lms7002m_context* self, const float value, const uint8_t channel)
{
    uint8_t savedChannel = lms7002m_get_active_channel(self);
    lms7002m_set_active_channel(self, channel);

    const double gmax = 12;
    double val = value - gmax;

    int g_tia_rfe = 1;

    if (val >= 0)
        g_tia_rfe = 3;
    else if (val >= -3)
        g_tia_rfe = 2;

    uint16_t ret = lms7002m_spi_modify_csr(self, LMS7002M_G_TIA_RFE, g_tia_rfe);
    lms7002m_set_active_channel(self, savedChannel);
    return ret;
}

float lms7002m_get_rfetia_db(lms7002m_context* self, const uint8_t channel)
{
    uint8_t savedChannel = lms7002m_get_active_channel(self);
    lms7002m_set_active_channel(self, channel);

    const double gmax = 12;
    uint8_t g_tia_rfe = lms7002m_spi_read_csr(self, LMS7002M_G_TIA_RFE);

    float retval = 0.0;
    const float value_to_minus[4] = { 0, 12, 3, 0 };

    if (g_tia_rfe > 0 && g_tia_rfe < 4)
    {
        retval = gmax - value_to_minus[g_tia_rfe];
    }

    lms7002m_set_active_channel(self, savedChannel);
    return retval;
}
