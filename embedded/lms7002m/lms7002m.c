#include "limesuiteng/embedded/lms7002m/lms7002m.h"
#include "limesuiteng/embedded/loglevel.h"
#include "lms7002m_context.h"

#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

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

static void lms7002m_sleep(long timeInMicroseconds)
{
    struct timespec time;
    time.tv_sec = 0;
    time.tv_nsec = timeInMicroseconds * 1000;

    // POSIX function, non-standard C
    nanosleep(&time, NULL);
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

lime_Result lms7002m_enable_channel(lms7002m_context* self, const bool isTx, uint8_t channel, const bool enable)
{
    const uint8_t savedChannel = lms7002m_get_active_channel(self);
    channel = clamp(channel, 0, 1) + 1;
    lms7002m_set_active_channel(self, channel);

    //--- LML ---
    if (channel == LMS7002M_CHANNEL_A)
    {
        if (isTx)
            lms7002m_spi_modify_csr(self, LMS7002M_TXEN_A, enable ? 1 : 0);
        else
            lms7002m_spi_modify_csr(self, LMS7002M_RXEN_A, enable ? 1 : 0);
    }
    else
    {
        if (isTx)
            lms7002m_spi_modify_csr(self, LMS7002M_TXEN_B, enable ? 1 : 0);
        else
            lms7002m_spi_modify_csr(self, LMS7002M_RXEN_B, enable ? 1 : 0);
    }

    //--- ADC/DAC ---
    lms7002m_spi_modify_csr(self, LMS7002M_EN_DIR_AFE, 1);

    if (!enable)
    {
        bool disable;
        if (channel == LMS7002M_CHANNEL_A)
            disable = lms7002m_spi_read_csr(self, isTx ? LMS7002M_TXEN_B : LMS7002M_RXEN_B) == 0;
        else
            disable = lms7002m_spi_read_csr(self, isTx ? LMS7002M_TXEN_A : LMS7002M_RXEN_A) == 0;
        lms7002m_spi_modify_csr(self, isTx ? LMS7002M_PD_TX_AFE1 : LMS7002M_PD_RX_AFE1, disable);
    }
    else
        lms7002m_spi_modify_csr(self, isTx ? LMS7002M_PD_TX_AFE1 : LMS7002M_PD_RX_AFE1, 0);

    if (channel == LMS7002M_CHANNEL_B)
        lms7002m_spi_modify_csr(self, isTx ? LMS7002M_PD_TX_AFE2 : LMS7002M_PD_RX_AFE2, enable ? 0 : 1);

    int disabledChannels = (lms7002m_spi_read_bits(self, LMS7002M_PD_AFE.address, 4, 1) & 0xF); //check if all channels are disabled
    lms7002m_spi_modify_csr(self, LMS7002M_EN_G_AFE, disabledChannels == 0xF ? 0 : 1);
    lms7002m_spi_modify_csr(self, LMS7002M_PD_AFE, disabledChannels == 0xF ? 1 : 0);

    //--- digital ---
    if (isTx)
    {
        lms7002m_spi_modify_csr(self, LMS7002M_EN_TXTSP, enable ? 1 : 0);
        lms7002m_spi_modify_csr(self, LMS7002M_ISINC_BYP_TXTSP, enable ? 0 : 1);
        lms7002m_spi_modify_csr(self, LMS7002M_GFIR3_BYP_TXTSP, 1);
        lms7002m_spi_modify_csr(self, LMS7002M_GFIR2_BYP_TXTSP, 1);
        lms7002m_spi_modify_csr(self, LMS7002M_GFIR1_BYP_TXTSP, 1);

        if (!enable)
        {
            lms7002m_spi_modify_csr(self, LMS7002M_CMIX_BYP_TXTSP, 1);
            lms7002m_spi_modify_csr(self, LMS7002M_DC_BYP_TXTSP, 1);
            lms7002m_spi_modify_csr(self, LMS7002M_GC_BYP_TXTSP, 1);
            lms7002m_spi_modify_csr(self, LMS7002M_PH_BYP_TXTSP, 1);
        }
    }
    else
    {
        lms7002m_spi_modify_csr(self, LMS7002M_EN_RXTSP, enable ? 1 : 0);
        lms7002m_spi_modify_csr(self, LMS7002M_DC_BYP_RXTSP, enable ? 0 : 1);
        lms7002m_spi_modify_csr(self, LMS7002M_DCLOOP_STOP, enable ? 0 : 1);
        lms7002m_spi_modify_csr(self, LMS7002M_AGC_MODE_RXTSP, 2); //bypass
        lms7002m_spi_modify_csr(self, LMS7002M_AGC_BYP_RXTSP, 1);
        lms7002m_spi_modify_csr(self, LMS7002M_GFIR3_BYP_RXTSP, 1);
        lms7002m_spi_modify_csr(self, LMS7002M_GFIR2_BYP_RXTSP, 1);
        lms7002m_spi_modify_csr(self, LMS7002M_GFIR1_BYP_RXTSP, 1);
        if (!enable)
        {
            lms7002m_spi_modify_csr(self, LMS7002M_CMIX_BYP_RXTSP, 1);
            lms7002m_spi_modify_csr(self, LMS7002M_GC_BYP_RXTSP, 1);
            lms7002m_spi_modify_csr(self, LMS7002M_PH_BYP_RXTSP, 1);
        }
    }

    //--- baseband ---
    if (isTx)
    {
        lms7002m_spi_modify_csr(self, LMS7002M_EN_DIR_TBB, 1);
        lms7002m_spi_modify_csr(self, LMS7002M_EN_G_TBB, enable ? 1 : 0);
        lms7002m_spi_modify_csr(self, LMS7002M_PD_LPFIAMP_TBB, enable ? 0 : 1);
    }
    else
    {
        lms7002m_spi_modify_csr(self, LMS7002M_EN_DIR_RBB, 1);
        lms7002m_spi_modify_csr(self, LMS7002M_EN_G_RBB, enable ? 1 : 0);
        lms7002m_spi_modify_csr(self, LMS7002M_PD_PGA_RBB, enable ? 0 : 1);
    }

    //--- frontend ---
    if (isTx)
    {
        lms7002m_spi_modify_csr(self, LMS7002M_EN_DIR_TRF, 1);
        lms7002m_spi_modify_csr(self, LMS7002M_EN_G_TRF, enable ? 1 : 0);
        lms7002m_spi_modify_csr(self, LMS7002M_PD_TLOBUF_TRF, enable ? 0 : 1);
        lms7002m_spi_modify_csr(self, LMS7002M_PD_TXPAD_TRF, enable ? 0 : 1);
    }
    else
    {
        lms7002m_spi_modify_csr(self, LMS7002M_EN_DIR_RFE, 1);
        lms7002m_spi_modify_csr(self, LMS7002M_EN_G_RFE, enable ? 1 : 0);
        lms7002m_spi_modify_csr(self, LMS7002M_PD_MXLOBUF_RFE, enable ? 0 : 1);
        lms7002m_spi_modify_csr(self, LMS7002M_PD_QGEN_RFE, enable ? 0 : 1);
        lms7002m_spi_modify_csr(self, LMS7002M_PD_TIA_RFE, enable ? 0 : 1);
        lms7002m_spi_modify_csr(self, LMS7002M_PD_LNA_RFE, enable ? 0 : 1);
    }

    //--- synthesizers ---
    if (isTx)
    {
        lms7002m_set_active_channel(self, LMS7002M_CHANNEL_SXT);
        lms7002m_spi_modify_csr(self, LMS7002M_EN_DIR_SXRSXT, 1);
        lms7002m_spi_modify_csr(self, LMS7002M_EN_G, (disabledChannels & 3) == 3 ? 0 : 1);
        if (channel == LMS7002M_CHANNEL_B) //enable LO to channel B
        {
            lms7002m_set_active_channel(self, LMS7002M_CHANNEL_A);
            lms7002m_spi_modify_csr(self, LMS7002M_EN_NEXTTX_TRF, enable ? 1 : 0);
        }
    }
    else
    {
        lms7002m_set_active_channel(self, LMS7002M_CHANNEL_SXR);
        lms7002m_spi_modify_csr(self, LMS7002M_EN_DIR_SXRSXT, 1);
        lms7002m_spi_modify_csr(self, LMS7002M_EN_G, (disabledChannels & 0xC) == 0xC ? 0 : 1);
        if (channel == LMS7002M_CHANNEL_B) //enable LO to channel B
        {
            lms7002m_set_active_channel(self, LMS7002M_CHANNEL_A);
            lms7002m_spi_modify_csr(self, LMS7002M_EN_NEXTRX_RFE, enable ? 1 : 0);
        }
    }

    lms7002m_set_active_channel(self, savedChannel);
    return lime_Result_Success;
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

lime_Result lms7002m_soft_reset(lms7002m_context* self)
{
    uint16_t reg_0x0020 = lms7002m_spi_read(self, 0x0020);
    uint16_t reg_0x002E = lms7002m_spi_read(self, 0x002E);
    lms7002m_spi_write(self, 0x0020, 0x0);
    lms7002m_spi_write(self, 0x0020, reg_0x0020);
    lms7002m_spi_write(self, 0x002E, reg_0x002E); //must write, enables/disabled MIMO channel B
    return lime_Result_Success;
}

lime_Result lms7002m_reset_logic_registers(lms7002m_context* self)
{
    const uint16_t x0020_value = lms7002m_spi_read(self, 0x0020); //reset logic registers

    lms7002m_spi_write(self, 0x0020, x0020_value & 0x553F);
    lms7002m_spi_write(self, 0x0020, x0020_value | 0xFFC0);
    return lime_Result_Success;
}

float lms7002m_get_reference_clock(lms7002m_context* context)
{
    return context->reference_clock_hz;
}

lime_Result lms7002m_set_reference_clock(lms7002m_context* context, float frequency_Hz)
{
    if (frequency_Hz <= 0)
        return lime_Result_InvalidValue;

    context->reference_clock_hz = frequency_Hz;
    return lime_Result_Success;
}

static uint8_t check_cgen_csw(lms7002m_context* self, uint8_t csw)
{
    lms7002m_spi_modify_csr(self, LMS7002M_CSW_VCO_CGEN, csw); //write CSW value
    lms7002m_sleep(50);
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

    // LOG_D(self, "csw %d; interval [%d, %d]", (cswHigh + cswLow) / 2, cswLow, cswHigh);
    uint8_t cmphl = check_cgen_csw(self, (cswHigh + cswLow) / 2);
    if (cmphl == 2)
        return lime_Result_Success;
    // lime::error("TuneVCO(CGEN) - failed to lock (cmphl!=%d)", cmphl);
    return lime_Result_Error;
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

float lms7002m_get_frequency_cgen(lms7002m_context* self)
{
    const float ref_clock = lms7002m_get_reference_clock(self);
    const uint16_t div_outch_cgen = lms7002m_spi_read_csr(self, LMS7002M_DIV_OUTCH_CGEN);
    const float dMul = (ref_clock / 2.0) / (div_outch_cgen + 1);

    const uint16_t gINT = lms7002m_spi_read_bits(self, 0x0088, 13, 0); //read whole register to reduce SPI transfers
    const uint16_t lowerRegister = lms7002m_spi_read_bits(self, 0x0087, 15, 0);

    const uint32_t gFRAC = ((gINT & 0xF) << 16) | lowerRegister;
    return dMul * ((gINT >> 4) + 1 + gFRAC / 1048576.0);
}

lime_Result lms7002m_set_rbbpga_db(lms7002m_context* self, const float value, const uint8_t channel)
{
    const uint8_t savedChannel = lms7002m_get_active_channel(self);
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
    const uint8_t savedChannel = lms7002m_get_active_channel(self);
    lms7002m_set_active_channel(self, channel);

    uint16_t g_pga_rbb = lms7002m_spi_read_csr(self, LMS7002M_G_PGA_RBB);

    lms7002m_set_active_channel(self, savedChannel);
    return g_pga_rbb - 12;
}

lime_Result lms7002m_set_rfelna_db(lms7002m_context* self, const float value, const uint8_t channel)
{
    const uint8_t savedChannel = lms7002m_get_active_channel(self);
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
    const uint8_t savedChannel = lms7002m_get_active_channel(self);
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
    const uint8_t savedChannel = lms7002m_get_active_channel(self);
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
    const uint8_t savedChannel = lms7002m_get_active_channel(self);
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
    const uint8_t savedChannel = lms7002m_get_active_channel(self);
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
    const uint8_t savedChannel = lms7002m_get_active_channel(self);
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

lime_Result lms7002m_set_trfpad_db(lms7002m_context* self, const float value, const uint8_t channel)
{
    const uint8_t savedChannel = lms7002m_get_active_channel(self);
    lms7002m_set_active_channel(self, channel);

    const double pmax = 52;
    uint16_t loss_int = lroundl(pmax - value);

    //different scaling realm
    if (loss_int > 10)
    {
        loss_int = (loss_int + 10) / 2;
    }

    loss_int = clamp(loss_int, 0, 31);

    lime_Result ret;
    lms7002m_spi_modify_csr(self, LMS7002M_LOSS_LIN_TXPAD_TRF, loss_int);
    ret = lms7002m_spi_modify_csr(self, LMS7002M_LOSS_MAIN_TXPAD_TRF, loss_int);

    lms7002m_set_active_channel(self, savedChannel);
    return ret;
}

float lms7002m_get_trfpad_db(lms7002m_context* self, const uint8_t channel)
{
    const uint8_t savedChannel = lms7002m_get_active_channel(self);
    lms7002m_set_active_channel(self, channel);

    const double pmax = 52;
    uint16_t loss_int = lms7002m_spi_read_csr(self, LMS7002M_LOSS_LIN_TXPAD_TRF);
    if (loss_int > 10)
        return pmax - 10 - 2 * (loss_int - 10);

    lms7002m_set_active_channel(self, savedChannel);
    return pmax - loss_int;
}

lime_Result lms7002m_set_trf_loopback_pad_db(lms7002m_context* self, const float gain, const uint8_t channel)
{
    const uint8_t savedChannel = lms7002m_get_active_channel(self);
    lms7002m_set_active_channel(self, channel);

    //there are 4 discrete gain values, use the midpoints
    int val = 3;
    if (gain >= (-1.4 - 0) / 2)
        val = 0;
    else if (gain >= (-1.4 - 3.3) / 2)
        val = 1;
    else if (gain >= (-3.3 - 4.3) / 2)
        val = 2;

    lime_Result ret = lms7002m_spi_modify_csr(self, LMS7002M_L_LOOPB_TXPAD_TRF, val);
    lms7002m_set_active_channel(self, savedChannel);
    return ret;
}

float lms7002m_get_trf_loopback_pad_db(lms7002m_context* self, const uint8_t channel)
{
    const uint8_t savedChannel = lms7002m_get_active_channel(self);
    lms7002m_set_active_channel(self, channel);

    uint16_t value = lms7002m_spi_read_csr(self, LMS7002M_L_LOOPB_TXPAD_TRF);

    float retval = 0.0;
    float value_table[] = { 0.0, -1.4, -3.3, -4.3 };

    if (value < 4)
    {
        retval = value_table[value];
    }

    lms7002m_set_active_channel(self, savedChannel);
    return retval;
}

lime_Result lms7002m_set_path_rfe(lms7002m_context* self, const uint8_t path)
{
    int sel_path_rfe;
    int pd_lb1 = 1;
    int pd_lb2 = 1;

    switch (path)
    {
    case LMS7002M_PATH_RFE_LNAH:
        sel_path_rfe = 1;
        break;
    case LMS7002M_PATH_RFE_LB2:
        pd_lb2 = 0;
    case LMS7002M_PATH_RFE_LNAL:
        sel_path_rfe = 2;
        break;
    case LMS7002M_PATH_RFE_LB1:
        pd_lb1 = 0;
    case LMS7002M_PATH_RFE_LNAW:
        sel_path_rfe = 3;
        break;
    default:
        sel_path_rfe = 0;
        break;
    }

    lms7002m_spi_modify_csr(self, LMS7002M_SEL_PATH_RFE, sel_path_rfe);

    int pd_lna_rfe = (path == 5 || path == 4 || sel_path_rfe == 0) ? 1 : 0;
    lms7002m_spi_modify_csr(self, LMS7002M_PD_LNA_RFE, pd_lna_rfe);

    lms7002m_spi_modify_csr(self, LMS7002M_PD_RLOOPB_1_RFE, pd_lb1);
    lms7002m_spi_modify_csr(self, LMS7002M_PD_RLOOPB_2_RFE, pd_lb2);
    lms7002m_spi_modify_csr(self, LMS7002M_EN_INSHSW_LB1_RFE, pd_lb1);
    lms7002m_spi_modify_csr(self, LMS7002M_EN_INSHSW_LB2_RFE, pd_lb2);
    lms7002m_spi_modify_csr(self, LMS7002M_EN_INSHSW_L_RFE, (path == LMS7002M_PATH_RFE_LNAL) ? 0 : 1);
    lms7002m_spi_modify_csr(self, LMS7002M_EN_INSHSW_W_RFE, (path == LMS7002M_PATH_RFE_LNAW) ? 0 : 1);

    //enable/disable the loopback path
    const bool loopback = (path == LMS7002M_PATH_RFE_LB1) || (path == LMS7002M_PATH_RFE_LB2);
    lms7002m_spi_modify_csr(self, LMS7002M_EN_LOOPB_TXPAD_TRF, loopback ? 1 : 0);

    return lime_Result_Success;
}

uint8_t lms7002m_get_path_rfe(lms7002m_context* self)
{
    const int sel_path_rfe = lms7002m_spi_read_csr(self, LMS7002M_SEL_PATH_RFE);
    if (lms7002m_spi_read_csr(self, LMS7002M_EN_INSHSW_LB1_RFE) == 0 && sel_path_rfe == 3)
        return LMS7002M_PATH_RFE_LB1;
    if (lms7002m_spi_read_csr(self, LMS7002M_EN_INSHSW_LB2_RFE) == 0 && sel_path_rfe == 2)
        return LMS7002M_PATH_RFE_LB2;
    if (lms7002m_spi_read_csr(self, LMS7002M_EN_INSHSW_L_RFE) == 0 && sel_path_rfe == 2)
        return LMS7002M_PATH_RFE_LNAL;
    if (lms7002m_spi_read_csr(self, LMS7002M_EN_INSHSW_W_RFE) == 0 && sel_path_rfe == 3)
        return LMS7002M_PATH_RFE_LNAW;
    if (sel_path_rfe == 1)
        return LMS7002M_PATH_RFE_LNAH;
    return LMS7002M_PATH_RFE_NONE;
}

lime_Result lms7002m_set_band_trf(lms7002m_context* self, const uint8_t band)
{
    lms7002m_spi_modify_csr(self, LMS7002M_SEL_BAND1_TRF, (band == 1) ? 1 : 0);
    lms7002m_spi_modify_csr(self, LMS7002M_SEL_BAND2_TRF, (band == 2) ? 1 : 0);

    return lime_Result_Success;
}

uint8_t lms7002m_get_band_trf(lms7002m_context* self)
{
    if (lms7002m_spi_read_csr(self, LMS7002M_SEL_BAND1_TRF) == 1)
        return 1;
    if (lms7002m_spi_read_csr(self, LMS7002M_SEL_BAND2_TRF) == 1)
        return 2;
    return 0;
}

lime_Result lms7002m_set_path(lms7002m_context* self, bool isTx, uint8_t channel, uint8_t path)
{
    const uint8_t savedChannel = lms7002m_get_active_channel(self);
    lms7002m_set_active_channel(self, channel);

    lime_Result ret = lime_Result_Success;

    if (isTx)
    {
        ret = lms7002m_set_band_trf(self, path);
    }
    else
    {
        ret = lms7002m_set_path_rfe(self, path);
    }

    lms7002m_set_active_channel(self, savedChannel);
    return ret;
}

float lms7002m_get_reference_clock_tsp(lms7002m_context* self, bool isTx)
{
    const float cgenFreq = lms7002m_get_frequency_cgen(self);
    const float clklfreq = cgenFreq / pow(2.0, lms7002m_spi_read_csr(self, LMS7002M_CLKH_OV_CLKL_CGEN));
    if (lms7002m_spi_read_csr(self, LMS7002M_EN_ADCCLKH_CLKGN) == 0)
        return isTx ? clklfreq : cgenFreq / 4.0;

    return isTx ? cgenFreq : clklfreq / 4.0;
}

bool lms7002m_get_cgen_locked(lms7002m_context* self)
{
    return (lms7002m_spi_read_bits(self, LMS7002M_VCO_CMPHO_CGEN.address, 13, 12) & 0x3) == 0x2;
}

bool lms7002m_get_sx_locked(lms7002m_context* self, bool isTx)
{
    lms7002m_set_active_channel(self, isTx ? LMS7002M_CHANNEL_SXT : LMS7002M_CHANNEL_SXR);
    return (lms7002m_spi_read_bits(self, LMS7002M_VCO_CMPHO.address, 13, 12) & 0x3) == 0x2;
}

lime_Result lms7002m_tune_vco(lms7002m_context* self, enum lms7002m_vco_type module)
{
    if (module == LMS7002M_VCO_CGEN)
        return lms7002m_tune_cgen_vco(self);

    const uint8_t savedChannel = lms7002m_get_active_channel(self);
    lms7002m_set_active_channel(self, module);

    const char* const moduleName = (module == LMS7002M_VCO_SXR) ? "SXR" : "SXT";
    LOG_D(self, "TuneVCO(%s) ICT_VCO: %d", moduleName, lms7002m_spi_read_csr(self, LMS7002M_ICT_VCO));

    // Initialization activate VCO and comparator
    const uint16_t addrVCOpd = LMS7002M_PD_VCO.address; // VCO power down address
    const lime_Result status = lms7002m_spi_modify(self, addrVCOpd, 2, 1, 0);
    if (status != lime_Result_Success)
    {
        lms7002m_set_active_channel(self, savedChannel);
        return status;
    }

    if (lms7002m_spi_read_bits(self, addrVCOpd, 2, 1) != 0)
    {
        lms7002m_log(self, lime_LogLevel_Error, "TuneVCO(%s) - VCO is powered down", moduleName);

        lms7002m_set_active_channel(self, savedChannel);
        return lime_Result_Error;
    }

    //check if lock is within VCO range
    const uint16_t addrCSW_VCO = LMS7002M_CSW_VCO.address;
    const uint8_t lsb = LMS7002M_CSW_VCO.lsb; //SWC lsb index
    const uint8_t msb = LMS7002M_CSW_VCO.msb; //SWC msb index
    lms7002m_spi_modify(self, addrCSW_VCO, msb, lsb, 0);

    const uint16_t settlingTimeMicroseconds = 50; //can be lower
    lms7002m_sleep(settlingTimeMicroseconds);

    const uint16_t addrCMP = LMS7002M_VCO_CMPHO.address; //comparator address

    uint8_t cmphl = lms7002m_spi_read_bits(self, addrCMP, 13, 12); //comparators
    if (cmphl == 3) //VCO too high
    {
        LOG_D(self, "TuneVCO(%s) - attempted VCO too high", moduleName);

        lms7002m_set_active_channel(self, savedChannel);
        return lime_Result_Error;
    }

    lms7002m_spi_modify(self, addrCSW_VCO, msb, lsb, 255);
    lms7002m_sleep(settlingTimeMicroseconds);
    cmphl = lms7002m_spi_read_bits(self, addrCMP, 13, 12);
    if (cmphl == 0) //VCO too low
    {
        LOG_D(self, "TuneVCO(%s) - attempted VCO too low", moduleName);

        lms7002m_set_active_channel(self, savedChannel);
        return lime_Result_Error;
    }

    typedef struct {
        int16_t high;
        int16_t low;
    } CSWInteval;

    CSWInteval cswSearch[2];

    //search intervals [0-127][128-255]
    for (int t = 0; t < 2; ++t)
    {
        bool hadLock = false;
        // initialize search range with invalid values
        cswSearch[t].low = 128 * (t + 1); // set low to highest possible value
        cswSearch[t].high = 128 * t; // set high to lowest possible value
        LOG_D(self, "TuneVCO(%s) - searching interval [%i:%i]", moduleName, cswSearch[t].high, cswSearch[t].low);
        lms7002m_spi_modify(self, addrCSW_VCO, msb, lsb, cswSearch[t].high);
        //binary search for and high value, and on the way store approximate low value
        LOG_D(self, "%s", "binary search:");
        for (int i = 6; i >= 0; --i)
        {
            cswSearch[t].high |= 1 << i; //CSW_VCO<i>=1
            lms7002m_spi_modify(self, addrCSW_VCO, msb, lsb, cswSearch[t].high);
            lms7002m_sleep(settlingTimeMicroseconds);
            cmphl = lms7002m_spi_read_bits(self, addrCMP, 13, 12);
            LOG_D(self, "csw=%d\tcmphl=%d", cswSearch[t].high, cmphl);
            if (cmphl & 0x01) // reduce CSW
                cswSearch[t].high &= ~(1 << i); //CSW_VCO<i>=0
            if (cmphl == 2 && cswSearch[t].high < cswSearch[t].low)
            {
                cswSearch[t].low = cswSearch[t].high;
                hadLock = true;
            }
        }
        //linear search to make sure there are no gaps, and move away from edge case
        LOG_D(self, "%s", "adjust with linear search:");
        while (cswSearch[t].low <= cswSearch[t].high && cswSearch[t].low > t * 128)
        {
            --cswSearch[t].low;
            lms7002m_spi_modify(self, addrCSW_VCO, msb, lsb, cswSearch[t].low);
            lms7002m_sleep(settlingTimeMicroseconds);
            const uint8_t tempCMPvalue = lms7002m_spi_read_bits(self, addrCMP, 13, 12);
            LOG_D(self, "csw=%d\tcmphl=%d", cswSearch[t].low, tempCMPvalue);
            if (tempCMPvalue != 2)
            {
                ++cswSearch[t].low;
                break;
            }
        }
        if (hadLock)
        {
            LOG_D(self,
                "CSW: lowest=%d, highest=%d, will use=%d",
                cswSearch[t].low,
                cswSearch[t].high,
                cswSearch[t].low + (cswSearch[t].high - cswSearch[t].low) / 2);
        }
        else
            LOG_D(self, "%s", "CSW interval failed to lock");
    }

    //check if the intervals are joined
    int16_t cswHigh = 0, cswLow = 0;
    if (cswSearch[0].high == cswSearch[1].low - 1)
    {
        cswHigh = cswSearch[1].high;
        cswLow = cswSearch[0].low;
        LOG_D(self, "CSW is locking in one continous range: low=%d, high=%d", cswLow, cswHigh);
    }
    //compare which interval is wider
    else
    {
        uint8_t intervalIndex = (cswSearch[1].high - cswSearch[1].low > cswSearch[0].high - cswSearch[0].low);
        cswHigh = cswSearch[intervalIndex].high;
        cswLow = cswSearch[intervalIndex].low;
        LOG_D(self, "choosing wider CSW locking range: low=%d, high=%d", cswLow, cswHigh);
    }

    uint8_t finalCSW = 0;
    if (cswHigh - cswLow == 1)
    {
        LOG_D(self,
            "TuneVCO(%s) - narrow locking values range detected [%i:%i]. VCO lock status might change with temperature.",
            moduleName,
            cswLow,
            cswHigh);
        //check which of two values really locks
        finalCSW = cswLow;
        lms7002m_spi_modify(self, addrCSW_VCO, msb, lsb, cswLow);
        lms7002m_sleep(settlingTimeMicroseconds);
        cmphl = lms7002m_spi_read_bits(self, addrCMP, 13, 12);
        if (cmphl != 2)
        {
            finalCSW = cswHigh;
            lms7002m_spi_modify(self, addrCSW_VCO, msb, lsb, cswHigh);
        }
    }
    else
    {
        finalCSW = cswLow + (cswHigh - cswLow) / 2;
        lms7002m_spi_modify(self, addrCSW_VCO, msb, lsb, finalCSW);
    }
    lms7002m_sleep(settlingTimeMicroseconds);
    cmphl = lms7002m_spi_read_bits(self, addrCMP, 13, 12);
    lms7002m_set_active_channel(self, savedChannel);

    if (cmphl != 2)
    {
        LOG_D(self, "TuneVCO(%s) - failed lock with final csw=%i, cmphl=%i", moduleName, finalCSW, cmphl);
        return lime_Result_Error;
    }

    LOG_D(self, "TuneVCO(%s) - confirmed lock with final csw=%i, cmphl=%i", moduleName, finalCSW, cmphl);
    return lime_Result_Success;
}

float lms7002m_get_frequency_sx(lms7002m_context* self, bool isTx)
{
    const uint8_t savedChannel = lms7002m_get_active_channel(self);
    lms7002m_set_active_channel(self, isTx ? LMS7002M_CHANNEL_SXT : LMS7002M_CHANNEL_SXR);

    const uint16_t gINT = lms7002m_spi_read_bits(self, 0x011E, 13, 0); // read whole register to reduce SPI transfers
    const uint16_t lowerRegister = lms7002m_spi_read_bits(self, 0x011D, 15, 0);
    const uint32_t gFRAC = ((gINT & 0xF) << 16) | lowerRegister;

    const float refClk_Hz = lms7002m_get_reference_clock(self);
    const uint16_t div_loch = lms7002m_spi_read_csr(self, LMS7002M_DIV_LOCH);
    const uint16_t en_div2_divprog = lms7002m_spi_read_csr(self, LMS7002M_EN_DIV2_DIVPROG);

    // Calculate real frequency according to the calculated parameters
    float dMul = (refClk_Hz / (1 << (div_loch + 1))) * ((gINT >> 4) + 4 + gFRAC / 1048576.0) * (en_div2_divprog + 1);

    lms7002m_set_active_channel(self, savedChannel);
    return dMul;
}

lime_Result lms7002m_set_nco_frequency(lms7002m_context* self, bool isTx, const uint8_t index, float freq_Hz)
{
    if (index > 15)
    {
        return lms7002m_report_error(
            self, lime_Result_InvalidValue, "SetNCOFrequency(index = %d) - index out of range [0, 15]", index);
    }

    float refClk_Hz = lms7002m_get_reference_clock_tsp(self, isTx);
    if (freq_Hz < 0 || freq_Hz / refClk_Hz > 0.5)
    {
        return lms7002m_report_error(self,
            lime_Result_OutOfRange,
            "SetNCOFrequency(index = %d) - Frequency(%g MHz) out of range [0-%g) MHz",
            index,
            freq_Hz / 1e6,
            refClk_Hz / 2e6);
    }

    const uint16_t addr = isTx ? 0x0240 : 0x0440;
    const uint32_t fcw = (freq_Hz / refClk_Hz) * 4294967296;
    lms7002m_spi_write(self, addr + 2 + index * 2, (fcw >> 16)); //NCO frequency control word register MSB part.
    lms7002m_spi_write(self, addr + 3 + index * 2, fcw); //NCO frequency control word register LSB part.
    return lime_Result_Success;
}

float lms7002m_get_nco_frequency(lms7002m_context* self, bool isTx, const uint8_t index)
{
    if (index > 15)
    {
        lms7002m_report_error(
            self, lime_Result_InvalidValue, "GetNCOFrequency_MHz(index = %d) - index out of range [0, 15]", index);
        return 0.0f;
    }

    const float refClk_Hz = lms7002m_get_reference_clock_tsp(self, isTx);
    const uint16_t addr = isTx ? 0x0240 : 0x0440;
    uint32_t fcw = 0;
    fcw |= lms7002m_spi_read(self, addr + 2 + index * 2) << 16; //NCO frequency control word register MSB part.
    fcw |= lms7002m_spi_read(self, addr + 3 + index * 2); //NCO frequency control word register LSB part.
    return refClk_Hz * (fcw / 4294967296.0);
}

lime_Result lms7002m_set_nco_phase_offset_for_mode_0(lms7002m_context* self, bool isTx, float angle_deg)
{
    const uint16_t addr = isTx ? 0x0241 : 0x0441;
    const uint16_t pho = 65536 * (angle_deg / 360);
    lms7002m_spi_write(self, addr, pho);
    return lime_Result_Success;
}
