#include "limesuiteng/embedded/lms7002m/lms7002m.h"

#include "csr.h"
#include "limesuiteng/embedded/loglevel.h"
#include "lms7002m_context.h"
#include "lms_gfir.h"
#include "privates.h"

#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CGEN_MAX_FREQ 640e6

#ifndef M_PI
    #define M_PI 3.14159265358979323846 /* pi */
#endif

struct lms7002m_context* lms7002m_create(const lms7002m_hooks* hooks)
{
    lms7002m_context* self = malloc(sizeof(lms7002m_context));
    if (self == NULL)
    {
        return self;
    }

    memcpy(&self->hooks, hooks, sizeof(lms7002m_hooks));
    return self;
}

void lms7002m_destroy(lms7002m_context* context)
{
    if (context)
        free(context);
}

static enum lms7002m_channel lms7002m_set_active_channel_readback(lms7002m_context* self, const enum lms7002m_channel channel)
{
    enum lms7002m_channel prev_ch = lms7002m_get_active_channel(self);
    if (channel != prev_ch)
        lms7002m_spi_modify_csr(self, LMS7002M_MAC, channel);
    return prev_ch;
}

lime_Result lms7002m_enable_channel(lms7002m_context* self, const bool isTx, enum lms7002m_channel channel, const bool enable)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, channel);

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

enum lms7002m_channel lms7002m_get_active_channel(lms7002m_context* self)
{
    return lms7002m_spi_read_csr(self, LMS7002M_MAC);
}

lime_Result lms7002m_set_active_channel(lms7002m_context* self, const enum lms7002m_channel channel)
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

    //VCO frequency selection according to F_CLKH
    const uint16_t iHdiv_high = (gCGEN_VCO_frequencies[1] / 2 / freq_Hz) - 1;
    const uint16_t iHdiv_low = (gCGEN_VCO_frequencies[0] / 2 / freq_Hz);
    const uint16_t iHdiv = clamp_uint((iHdiv_low + iHdiv_high) / 2, 0, 255);
    const float dFvco = 2 * (iHdiv + 1) * freq_Hz;
    if (dFvco <= gCGEN_VCO_frequencies[0] || dFvco >= gCGEN_VCO_frequencies[1])
    {
        return lms7002m_report_error(
            self, lime_Result_Error, "SetFrequencyCGEN(%g MHz) - cannot deliver requested frequency", freq_Hz / 1e6);
    }

    const float refClk = lms7002m_get_reference_clock(self);
    //Integer division
    const uint16_t gINT = (uint16_t)(dFvco / refClk - 1);

    //Fractional division
    const float dFrac = dFvco / refClk - (uint32_t)(dFvco / refClk);
    const uint32_t gFRAC = (uint32_t)(dFrac * 1048576);

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

lime_Result lms7002m_set_rbbpga_db(lms7002m_context* self, const float value, const enum lms7002m_channel channel)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, channel);

    int g_pga_rbb = clamp_uint(lroundf(value) + 12, 0, 31);
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

float lms7002m_get_rbbpga_db(lms7002m_context* self, const enum lms7002m_channel channel)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, channel);

    uint16_t g_pga_rbb = lms7002m_spi_read_csr(self, LMS7002M_G_PGA_RBB);

    lms7002m_set_active_channel(self, savedChannel);
    return g_pga_rbb - 12;
}

lime_Result lms7002m_set_rfelna_db(lms7002m_context* self, const float value, const enum lms7002m_channel channel)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, channel);

    const float gmax = 30;
    float val = value - gmax;

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

float lms7002m_get_rfelna_db(lms7002m_context* self, const enum lms7002m_channel channel)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, channel);

    const float gmax = 30;
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

lime_Result lms7002m_set_rfe_loopback_lna_db(lms7002m_context* self, const float gain, const enum lms7002m_channel channel)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, channel);

    const float gmax = 40;
    float val = gain - gmax;

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

float lms7002m_get_rfe_loopback_lna_db(lms7002m_context* self, const enum lms7002m_channel channel)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, channel);

    const float gmax = 40;
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

lime_Result lms7002m_set_rfetia_db(lms7002m_context* self, const float value, const enum lms7002m_channel channel)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, channel);

    const float gmax = 12;
    float val = value - gmax;

    int g_tia_rfe = 1;

    if (val >= 0)
        g_tia_rfe = 3;
    else if (val >= -3)
        g_tia_rfe = 2;

    uint16_t ret = lms7002m_spi_modify_csr(self, LMS7002M_G_TIA_RFE, g_tia_rfe);
    lms7002m_set_active_channel(self, savedChannel);
    return ret;
}

float lms7002m_get_rfetia_db(lms7002m_context* self, const enum lms7002m_channel channel)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, channel);

    const float gmax = 12;
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

lime_Result lms7002m_set_trfpad_db(lms7002m_context* self, const float value, const enum lms7002m_channel channel)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, channel);

    const float pmax = 52;
    uint16_t loss_int = lroundl(pmax - value);

    //different scaling realm
    if (loss_int > 10)
    {
        loss_int = (loss_int + 10) / 2;
    }

    loss_int = clamp_uint(loss_int, 0, 31);

    lime_Result ret;
    lms7002m_spi_modify_csr(self, LMS7002M_LOSS_LIN_TXPAD_TRF, loss_int);
    ret = lms7002m_spi_modify_csr(self, LMS7002M_LOSS_MAIN_TXPAD_TRF, loss_int);

    lms7002m_set_active_channel(self, savedChannel);
    return ret;
}

float lms7002m_get_trfpad_db(lms7002m_context* self, const enum lms7002m_channel channel)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, channel);

    const float pmax = 52;
    uint16_t loss_int = lms7002m_spi_read_csr(self, LMS7002M_LOSS_LIN_TXPAD_TRF);
    if (loss_int > 10)
        return pmax - 10 - 2 * (loss_int - 10);

    lms7002m_set_active_channel(self, savedChannel);
    return pmax - loss_int;
}

lime_Result lms7002m_set_trf_loopback_pad_db(lms7002m_context* self, const float gain, const enum lms7002m_channel channel)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, channel);

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

float lms7002m_get_trf_loopback_pad_db(lms7002m_context* self, const enum lms7002m_channel channel)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, channel);

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

lime_Result lms7002m_set_path_rfe(lms7002m_context* self, const enum lms7002m_path_rfe path)
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

    int pd_lna_rfe = (path == LMS7002M_PATH_RFE_LB2 || path == LMS7002M_PATH_RFE_LB1 || sel_path_rfe == 0) ? 1 : 0;
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

lime_Result lms7002m_set_path(lms7002m_context* self, bool isTx, enum lms7002m_channel channel, uint8_t path)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, channel);

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
    enum lms7002m_channel savedChannel =
        lms7002m_set_active_channel_readback(self, isTx ? LMS7002M_CHANNEL_SXT : LMS7002M_CHANNEL_SXR);

    bool isLocked = (lms7002m_spi_read_bits(self, LMS7002M_VCO_CMPHO.address, 13, 12) & 0x3) == 0x2;

    lms7002m_set_active_channel(self, savedChannel);
    return isLocked;
}

lime_Result lms7002m_tune_vco(lms7002m_context* self, enum lms7002m_vco_type module)
{
    if (module == LMS7002M_VCO_CGEN)
        return lms7002m_tune_cgen_vco(self);

    enum lms7002m_channel savedChannel =
        lms7002m_set_active_channel_readback(self, module == LMS7002M_VCO_SXR ? LMS7002M_CHANNEL_SXR : LMS7002M_CHANNEL_SXT);

    const char* const moduleName = (module == LMS7002M_VCO_SXR) ? "SXR" : "SXT";

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
    } CSWInterval;

    CSWInterval cswSearch[2];

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
        LOG_D(self, "CSW is locking in one continuous range: low=%d, high=%d", cswLow, cswHigh);
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
    if (cswHigh - cswLow <= 1)
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

static lime_Result lms7002m_write_sx_registers(
    lms7002m_context* self, uint64_t VCOfreq_hz, uint64_t reference_clock_hz, uint8_t div_loch)
{
    const float m_dThrF = 5500e6; // VCO frequency threshold to enable additional divider
    const float divider = reference_clock_hz * (1 + (VCOfreq_hz > m_dThrF));

    const uint16_t integerPart = (uint16_t)(VCOfreq_hz / divider - 4);
    const uint32_t fractionalPart = (uint32_t)((VCOfreq_hz / divider - (uint32_t)(VCOfreq_hz / divider)) * 1048576);

    lms7002m_spi_modify_csr(self, LMS7002M_EN_INTONLY_SDM, 0);
    lms7002m_spi_modify_csr(self, LMS7002M_INT_SDM, integerPart); //INT_SDM
    lms7002m_spi_modify(self, 0x011D, 15, 0, fractionalPart & 0xFFFF); //FRAC_SDM[15:0]
    lms7002m_spi_modify(self, 0x011E, 3, 0, (fractionalPart >> 16)); //FRAC_SDM[19:16]
    lms7002m_spi_modify_csr(self, LMS7002M_DIV_LOCH, div_loch); //DIV_LOCH
    lms7002m_spi_modify_csr(self, LMS7002M_EN_DIV2_DIVPROG, (VCOfreq_hz > m_dThrF)); //EN_DIV2_DIVPROG

    LOG_D(self,
        "SX VCO:%.3f MHz, RefClk:%.3f MHz, INT:%d, FRAC:%d, DIV_LOCH:%d, EN_DIV2_DIVPROG:%d",
        VCOfreq_hz / 1e6,
        reference_clock_hz / 1e6,
        integerPart,
        fractionalPart,
        div_loch,
        (VCOfreq_hz > m_dThrF));

    return lime_Result_Success;
}

lime_Result lms7002m_set_frequency_sx(lms7002m_context* self, bool isTx, float LO_freq_hz)
{
    LOG_D(self, "Set %s LO frequency (%g MHz)", isTx ? "Tx" : "Rx", LO_freq_hz / 1e6);

    if (LO_freq_hz < 0)
        return lime_Result_InvalidValue;

    const char* const vcoNames[] = { "VCOL", "VCOM", "VCOH" };
    const uint64_t VCO_min_frequency[3] = { 3800000000, 4961000000, 6306000000 };
    const uint64_t VCO_max_frequency[3] = { 5222000000, 6754000000, 7714000000 };

    struct VCOData {
        uint64_t frequency;
        uint8_t div_loch;
        uint8_t csw;
        bool canDeliverFrequency;
    } vco[3];

    bool canDeliverFrequency = false;

    //find required VCO frequency
    // div_loch value 7 is not allowed
    for (int8_t div_loch = 6; div_loch >= 0; --div_loch)
    {
        float VCOfreq = (1 << (div_loch + 1)) * LO_freq_hz;
        for (int i = 0; i < 3; ++i)
        {
            if (!vco[i].canDeliverFrequency && (VCOfreq >= VCO_min_frequency[i]) && (VCOfreq <= VCO_max_frequency[i]))
            {
                vco[i].canDeliverFrequency = true;
                vco[i].div_loch = div_loch;
                vco[i].frequency = VCOfreq;
                canDeliverFrequency = true;
            }
        }
    }

    if (!canDeliverFrequency)
    {
        return lms7002m_report_error(self,
            lime_Result_OutOfRange,
            "SetFrequencySX%s(%g MHz) - required VCO frequencies are out of range [%g-%g] MHz",
            isTx ? "T" : "R",
            LO_freq_hz / 1e6,
            VCO_min_frequency[0] / 1e6,
            VCO_max_frequency[2] / 1e6);
    }

    const float refClk_Hz = lms7002m_get_reference_clock(self);
    assert(refClk_Hz > 0);

    enum lms7002m_channel savedChannel =
        lms7002m_set_active_channel_readback(self, isTx ? LMS7002M_CHANNEL_SXT : LMS7002M_CHANNEL_SXR);

    // turn on VCO and comparator
    lms7002m_spi_modify_csr(self, LMS7002M_PD_VCO, 0); //
    lms7002m_spi_modify_csr(self, LMS7002M_PD_VCO_COMP, 0);

    const uint8_t preferred_vco_order[3] = { 2, 0, 1 };
    uint8_t sel_vco;
    canDeliverFrequency = false;
    uint8_t ict_vco = lms7002m_spi_read_csr(self, LMS7002M_ICT_VCO);
    do // if initial tune fails, attempt again with modified bias current
    {
        for (int i = 0; i < 3; ++i)
        {
            sel_vco = preferred_vco_order[i];
            if (!vco[sel_vco].canDeliverFrequency)
            {
                LOG_D(self, "%s skipped", vcoNames[sel_vco]);
                continue;
            }

            lms7002m_write_sx_registers(self, vco[sel_vco].frequency, refClk_Hz, vco[sel_vco].div_loch);
            lms7002m_log(self, lime_LogLevel_Debug, "Tuning %s %s (ICT_VCO:%d):", (isTx ? "Tx" : "Rx"), vcoNames[sel_vco], ict_vco);

            lms7002m_spi_modify_csr(self, LMS7002M_SEL_VCO, sel_vco);
            lime_Result status = lms7002m_tune_vco(self, isTx ? LMS7002M_VCO_SXT : LMS7002M_VCO_SXR);
            if (status == lime_Result_Success)
            {
                vco[sel_vco].csw = lms7002m_spi_read_csr(self, LMS7002M_CSW_VCO);
                canDeliverFrequency = true;
                LOG_D(self,
                    "%s : csw=%d %s",
                    vcoNames[sel_vco],
                    vco[sel_vco].csw,
                    (status == lime_Result_Success ? "tune ok" : "tune fail"));
                break;
            }
            else
                LOG_D(self, "%s : failed to lock", vcoNames[sel_vco]);
        }

        if (canDeliverFrequency)
        {
            lms7002m_log(self, lime_LogLevel_Debug, "Selected: %s, CSW_VCO: %i", vcoNames[sel_vco], vco[sel_vco].csw);
            lms7002m_spi_modify_csr(self, LMS7002M_SEL_VCO, sel_vco);
            lms7002m_spi_modify_csr(self, LMS7002M_CSW_VCO, vco[sel_vco].csw);
            break;
        }
        else
        {
            if (ict_vco == 255)
                break;
            ict_vco = ict_vco + 32 > 255 ? 255 : ict_vco + 32; // retry with higher bias current
            lms7002m_spi_modify_csr(self, LMS7002M_ICT_VCO, ict_vco);
        }
    } while (ict_vco <= 255);

    lms7002m_set_active_channel(self, savedChannel);

    if (canDeliverFrequency == false)
        return lms7002m_report_error(
            self, lime_Result_Error, "SetFrequencySX%s(%g MHz) - cannot deliver frequency", isTx ? "T" : "R", LO_freq_hz / 1e6);
    return lime_Result_Success;
}

float lms7002m_get_frequency_sx(lms7002m_context* self, bool isTx)
{
    enum lms7002m_channel savedChannel =
        lms7002m_set_active_channel_readback(self, isTx ? LMS7002M_CHANNEL_SXT : LMS7002M_CHANNEL_SXR);

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

lime_Result lms7002m_set_nco_phase_offset(lms7002m_context* self, bool isTx, uint8_t index, float angle_deg)
{
    if (index > 15)
    {
        return lms7002m_report_error(
            self, lime_Result_InvalidValue, "SetNCOPhaseOffset(index = %d) - index out of range [0, 15]", index);
    }

    const uint16_t addr = isTx ? 0x0244 : 0x0444;
    const uint16_t pho = 65536 * (angle_deg / 360);
    lms7002m_spi_write(self, addr + index, pho);
    return lime_Result_Success;
}

lime_Result lms7002m_set_nco_phase_offset_for_mode_0(lms7002m_context* self, bool isTx, float angle_deg)
{
    const uint16_t addr = isTx ? 0x0241 : 0x0441;
    const uint16_t pho = 65536 * (angle_deg / 360);
    lms7002m_spi_write(self, addr, pho);
    return lime_Result_Success;
}

lime_Result lms7002m_set_nco_phases(
    lms7002m_context* self, bool isTx, const float* const angles_deg, uint8_t count, float frequencyOffset)
{
    lime_Result status = lms7002m_set_nco_frequency(self, isTx, 0, frequencyOffset);
    if (status != lime_Result_Success)
    {
        return status;
    }

    if (angles_deg == NULL)
    {
        return lime_Result_Success;
    }

    for (uint8_t i = 0; i < 16 && i < count; i++)
    {
        status = lms7002m_set_nco_phase_offset(self, isTx, i, angles_deg[i]);
        if (status != lime_Result_Success)
            return status;
    }

    return lms7002m_spi_modify_csr(self, isTx ? LMS7002M_SEL_TX : LMS7002M_SEL_RX, 0);
}

lime_Result lms7002m_set_nco_frequencies(
    lms7002m_context* self, bool isTx, const float* const freq_Hz, uint8_t count, float phaseOffset)
{
    for (uint8_t i = 0; i < 16 && i < count; i++)
    {
        const lime_Result status = lms7002m_set_nco_frequency(self, isTx, i, freq_Hz[i]);
        if (status != lime_Result_Success)
            return status;
    }
    return lms7002m_set_nco_phase_offset_for_mode_0(self, isTx, phaseOffset);
}

lime_Result lms7002m_get_nco_frequencies(lms7002m_context* self, bool isTx, float* const freq_Hz, uint8_t count, float* phaseOffset)
{
    if (freq_Hz == NULL)
    {
        return lime_Result_Success;
    }

    for (int i = 0; i < 16 && i < count; ++i)
    {
        freq_Hz[i] = lms7002m_get_nco_frequency(self, isTx, i);
    }

    if (phaseOffset != NULL)
    {
        uint16_t value = lms7002m_spi_read(self, isTx ? 0x0241 : 0x0441);
        *phaseOffset = 360.0 * value / 65536.0;
    }

    return lime_Result_Success;
}

lime_Result lms7002m_set_gfir_coefficients(
    lms7002m_context* self, bool isTx, uint8_t gfirIndex, const float* const coef, uint8_t coefCount)
{
    if (gfirIndex > 2)
    {
        lms7002m_log(
            self, lime_LogLevel_Warning, "SetGFIRCoefficients: Invalid GFIR index(%i). Will configure GFIR[2].", gfirIndex);
        gfirIndex = 2;
    }

    const uint8_t maxCoefCount = gfirIndex < 2 ? 40 : 120;
    if (coefCount > maxCoefCount)
    {
        return lms7002m_report_error(self,
            lime_Result_OutOfRange,
            "SetGFIRCoefficients: too many coefficients(%i), GFIR[%i] can have only %i",
            coefCount,
            gfirIndex,
            maxCoefCount);
    }

    // actual used coefficients count is multiple of 'bankCount'
    // if coefCount is not multiple, extra '0' coefficients will be written
    const uint8_t bankCount = gfirIndex < 2 ? 5 : 15;
    const uint8_t bankLength = ceil((float)(coefCount) / bankCount);
    const int16_t actualCoefCount = bankLength * bankCount;
    assert(actualCoefCount <= maxCoefCount);

    lms7002m_csr gfirL_param = LMS7002M_GFIR1_L_TXTSP;
    gfirL_param.address += gfirIndex + (isTx ? 0 : 0x0200);
    lms7002m_spi_modify_csr(self, gfirL_param, bankLength - 1);

    const uint16_t startAddr = 0x0280 + (gfirIndex * 0x40) + (isTx ? 0 : 0x0200);
    for (int i = 0; i < actualCoefCount; ++i)
    {
        const uint8_t bank = i / bankLength;
        const uint8_t bankRow = i % bankLength;
        const uint16_t address = (startAddr + (bank * 8) + bankRow) + (24 * (bank / 5));
        int16_t valueToWrite = 0;

        if (i < coefCount)
        {
            valueToWrite = coef[i] * 32768;

            if (coef[i] < -1 || coef[i] > 1)
            {
                lms7002m_log(self,
                    lime_LogLevel_Warning,
                    "Coefficient %f is outside of range [-1:1], incorrect value will be written.",
                    coef[i]);
            }
        }

        lms7002m_spi_write(self, address, (uint16_t)valueToWrite);
    }

    return lime_Result_Success;
}

lime_Result lms7002m_get_gfir_coefficients(
    lms7002m_context* self, bool isTx, uint8_t gfirIndex, float* const coef, uint8_t coefCount)
{
    if (gfirIndex > 2)
    {
        lms7002m_log(self, lime_LogLevel_Warning, "GetGFIRCoefficients: Invalid GFIR index(%i). Will read GFIR[2].", gfirIndex);
        gfirIndex = 2;
    }

    const uint8_t coefLimit = gfirIndex < 2 ? 40 : 120;

    if (coefCount > coefLimit)
    {
        return lms7002m_report_error(
            self, lime_Result_OutOfRange, "GetGFIRCoefficients(coefCount=%d) - exceeds coefLimit=%d", coefCount, coefLimit);
    }

    const uint16_t startAddr = 0x0280 + (gfirIndex * 0x40) + (isTx ? 0 : 0x0200);
    for (uint8_t index = 0; index < coefCount; ++index)
    {
        const int16_t registerValue = (int16_t)(lms7002m_spi_read(self, startAddr + index + 24 * (index / 40)));
        coef[index] = registerValue / 32768.0;
    }

    return lime_Result_Success;
}

lime_Result lms7002m_set_interface_frequency(lms7002m_context* self, float cgen_freq_Hz, const uint8_t hbi, const uint8_t hbd)
{
    const lime_Result status = lms7002m_spi_modify_csr(self, LMS7002M_HBD_OVR_RXTSP, hbd);
    if (status != lime_Result_Success)
        return status;
    lms7002m_spi_modify_csr(self, LMS7002M_HBI_OVR_TXTSP, hbi);

    uint16_t siso = lms7002m_spi_read_csr(self, LMS7002M_LML2_SISODDR);
    const int mclk2src = lms7002m_spi_read_csr(self, LMS7002M_MCLK2SRC);
    if (hbd == 7 || (hbd == 0 && siso == 0)) //bypass
    {
        lms7002m_spi_modify_csr(self, LMS7002M_RXTSPCLKA_DIV, 0);
        lms7002m_spi_modify_csr(self, LMS7002M_RXDIVEN, false);
        lms7002m_spi_modify_csr(self, LMS7002M_MCLK2SRC, (mclk2src & 1) | 0x2);
    }
    else
    {
        const uint8_t divider = pow(2.0, hbd + siso);
        if (divider > 1)
            lms7002m_spi_modify_csr(self, LMS7002M_RXTSPCLKA_DIV, (divider / 2) - 1);
        else
            lms7002m_spi_modify_csr(self, LMS7002M_RXTSPCLKA_DIV, 0);
        lms7002m_spi_modify_csr(self, LMS7002M_RXDIVEN, true);
        lms7002m_spi_modify_csr(self, LMS7002M_MCLK2SRC, mclk2src & 1);
    }

    if (lms7002m_spi_read_csr(self, LMS7002M_RX_MUX) == 0)
    {
        const bool mimoBypass = (hbd == 7) && (siso == 0);
        lms7002m_spi_modify_csr(self, LMS7002M_RXRDCLK_MUX, mimoBypass ? 3 : 1);
        lms7002m_spi_modify_csr(self, LMS7002M_RXWRCLK_MUX, mimoBypass ? 1 : 2);
    }

    siso = lms7002m_spi_read_csr(self, LMS7002M_LML1_SISODDR);
    const int mclk1src = lms7002m_spi_read_csr(self, LMS7002M_MCLK1SRC);
    if (hbi == 7 || (hbi == 0 && siso == 0)) //bypass
    {
        lms7002m_spi_modify_csr(self, LMS7002M_TXTSPCLKA_DIV, 0);
        lms7002m_spi_modify_csr(self, LMS7002M_TXDIVEN, false);
        lms7002m_spi_modify_csr(self, LMS7002M_MCLK1SRC, (mclk1src & 1) | 0x2);
    }
    else
    {
        const uint8_t divider = pow(2.0, hbi + siso);
        if (divider > 1)
            lms7002m_spi_modify_csr(self, LMS7002M_TXTSPCLKA_DIV, (divider / 2) - 1);
        else
            lms7002m_spi_modify_csr(self, LMS7002M_TXTSPCLKA_DIV, 0);
        lms7002m_spi_modify_csr(self, LMS7002M_TXDIVEN, true);
        lms7002m_spi_modify_csr(self, LMS7002M_MCLK1SRC, mclk1src & 1);
    }

    if (lms7002m_spi_read_csr(self, LMS7002M_TX_MUX) == 0)
    {
        const bool mimoBypass = (hbi == 7) && (siso == 0);
        lms7002m_spi_modify_csr(self, LMS7002M_TXRDCLK_MUX, mimoBypass ? 0 : 2);
        lms7002m_spi_modify_csr(self, LMS7002M_TXWRCLK_MUX, 0);
    }

    return lms7002m_set_frequency_cgen(self, cgen_freq_Hz);
}

lime_Result lms7002m_enable_sxtdd(lms7002m_context* self, bool tdd)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, LMS7002M_CHANNEL_SXT);

    lms7002m_spi_modify_csr(self, LMS7002M_PD_LOCH_T2RBUF, tdd ? 0 : 1);
    lms7002m_spi_modify_csr(self, LMS7002M_MAC, 1); // switch to SXR
    lime_Result ret = lms7002m_spi_modify_csr(self, LMS7002M_PD_VCO, tdd ? 1 : 0);

    lms7002m_set_active_channel(self, savedChannel);
    return ret;
}

lime_Result lms7002m_set_dc_offset(lms7002m_context* self, bool isTx, const float I, const float Q)
{
    const bool bypass = I == 0.0 && Q == 0.0;
    if (isTx)
    {
        lms7002m_spi_modify_csr(self, LMS7002M_DC_BYP_TXTSP, bypass ? 1 : 0);
        lms7002m_spi_modify_csr(self, LMS7002M_DCCORRI_TXTSP, lrint(I * 127));
        lms7002m_spi_modify_csr(self, LMS7002M_DCCORRQ_TXTSP, lrint(Q * 127));
    }
    else
    {
        lms7002m_spi_modify_csr(self, LMS7002M_EN_DCOFF_RXFE_RFE, bypass ? 0 : 1);
        unsigned int val = lrint(fabs(I * 63)) + (I < 0 ? 64 : 0);
        lms7002m_spi_modify_csr(self, LMS7002M_DCOFFI_RFE, val);
        val = lrint(fabs(Q * 63)) + (Q < 0 ? 64 : 0);
        lms7002m_spi_modify_csr(self, LMS7002M_DCOFFQ_RFE, val);
    }

    return lime_Result_Success;
}

lime_Result lms7002m_get_dc_offset(lms7002m_context* self, bool isTx, float* const I, float* const Q)
{
    if (I == NULL || Q == NULL)
    {
        return lime_Result_Success;
    }

    if (isTx)
    {
        *I = (int8_t)(lms7002m_spi_read_csr(self, LMS7002M_DCCORRI_TXTSP)) / 127.0; //signed 8-bit
        *Q = (int8_t)(lms7002m_spi_read_csr(self, LMS7002M_DCCORRQ_TXTSP)) / 127.0; //signed 8-bit
        return lime_Result_Success;
    }

    uint16_t i = lms7002m_spi_read_csr(self, LMS7002M_DCOFFI_RFE);
    *I = ((i & 0x40) ? -1.0 : 1.0) * (i & 0x3F) / 63.0;
    uint16_t q = lms7002m_spi_read_csr(self, LMS7002M_DCOFFQ_RFE);
    *Q = ((q & 0x40) ? -1.0 : 1.0) * (q & 0x3F) / 63.0;
    return lime_Result_Success;
}

lime_Result lms7002m_set_i_q_balance(lms7002m_context* self, bool isTx, const float phase, const float gainI, const float gainQ)
{
    const bool bypassPhase = phase == 0.0;
    const bool bypassGain = ((gainI == 1.0) && (gainQ == 1.0)) || ((gainI == 0.0) && (gainQ == 0.0));
    const int iqcorr = lrint(2047 * (phase / (M_PI / 2)));
    const int gcorri = lrint(2047 * gainI);
    const int gcorrq = lrint(2047 * gainQ);

    lms7002m_spi_modify_csr(self, isTx ? LMS7002M_PH_BYP_TXTSP : LMS7002M_PH_BYP_RXTSP, bypassPhase ? 1 : 0);
    lms7002m_spi_modify_csr(self, isTx ? LMS7002M_GC_BYP_TXTSP : LMS7002M_GC_BYP_RXTSP, bypassGain ? 1 : 0);
    lms7002m_spi_modify_csr(self, isTx ? LMS7002M_IQCORR_TXTSP : LMS7002M_IQCORR_RXTSP, iqcorr);
    lms7002m_spi_modify_csr(self, isTx ? LMS7002M_GCORRI_TXTSP : LMS7002M_GCORRI_RXTSP, gcorri);
    lms7002m_spi_modify_csr(self, isTx ? LMS7002M_GCORRQ_TXTSP : LMS7002M_GCORRQ_RXTSP, gcorrq);
    return lime_Result_Success;
}

lime_Result lms7002m_get_i_q_balance(lms7002m_context* self, bool isTx, float* const phase, float* const gainI, float* const gainQ)
{
    if (phase == NULL || gainI == NULL || gainQ == NULL)
    {
        return lime_Result_Success;
    }

    const int iqcorr =
        (int16_t)(lms7002m_spi_read_csr(self, isTx ? LMS7002M_IQCORR_TXTSP : LMS7002M_IQCORR_RXTSP) << 4) >> 4; //sign extend 12-bit
    const int gcorri =
        (int16_t)(lms7002m_spi_read_csr(self, isTx ? LMS7002M_GCORRI_TXTSP : LMS7002M_GCORRI_RXTSP)); //unsigned 11-bit
    const int gcorrq =
        (int16_t)(lms7002m_spi_read_csr(self, isTx ? LMS7002M_GCORRQ_TXTSP : LMS7002M_GCORRQ_RXTSP)); //unsigned 11-bit

    *phase = (M_PI / 2) * iqcorr / 2047.0;
    *gainI = gcorri / 2047.0;
    *gainQ = gcorrq / 2047.0;

    return lime_Result_Success;
}

float lms7002m_get_temperature(lms7002m_context* self)
{
    if (lms7002m_calibrate_internal_adc(self, 32) != lime_Result_Success)
        return 0;
    lms7002m_spi_modify_csr(self, LMS7002M_RSSI_PD, 0);
    lms7002m_spi_modify_csr(self, LMS7002M_RSSI_RSSIMODE, 0);
    const uint16_t biasMux = lms7002m_spi_read_csr(self, LMS7002M_MUX_BIAS_OUT);
    lms7002m_spi_modify_csr(self, LMS7002M_MUX_BIAS_OUT, 2);

    lms7002m_sleep(250);

    const uint16_t reg606 = lms7002m_spi_read(self, 0x0606);
    const float Vtemp = ((reg606 >> 8) & 0xFF) * 1.84;
    const float Vptat = (reg606 & 0xFF) * 1.84;
    const float Vdiff = (Vptat - Vtemp) / 1.05;
    const float temperature = 45.0 + Vdiff;
    lms7002m_spi_modify_csr(self, LMS7002M_MUX_BIAS_OUT, biasMux);
    LOG_D(self, "Vtemp 0x%04X, Vptat 0x%04X, Vdiff = %.2f, temp= %.3f", (reg606 >> 8) & 0xFF, reg606 & 0xFF, Vdiff, temperature);
    return temperature;
}

lime_Result lms7002m_set_clock_frequency(lms7002m_context* self, enum lms7002m_clock_id clk_id, float freq)
{
    switch (clk_id)
    {
    case LMS7002M_CLK_REFERENCE:
        // TODO: recalculate CGEN,SXR/T
        break;
    case LMS7002M_CLK_CGEN:
        return lms7002m_set_frequency_cgen(self, freq);
        break;
    case LMS7002M_CLK_SXR:
        return lms7002m_set_frequency_sx(self, false, freq);
        break;
    case LMS7002M_CLK_SXT:
        return lms7002m_set_frequency_sx(self, true, freq);
        break;
    case LMS7002M_CLK_RXTSP:
    case LMS7002M_CLK_TXTSP:
        return lms7002m_report_error(self, lime_Result_InvalidValue, "RxTSP/TxTSP Clocks are read only");
    default:
        return lms7002m_report_error(self, lime_Result_InvalidValue, "LMS7002M::SetClockFreq Unknown clock id");
    }
    return lime_Result_Success;
}

float lms7002m_get_clock_frequency(lms7002m_context* self, enum lms7002m_clock_id clk_id)
{
    switch (clk_id)
    {
    case LMS7002M_CLK_REFERENCE:
        return lms7002m_get_reference_clock(self);
    case LMS7002M_CLK_SXR:
        return lms7002m_get_frequency_sx(self, false);
    case LMS7002M_CLK_SXT:
        return lms7002m_get_frequency_sx(self, true);
    case LMS7002M_CLK_CGEN:
        return lms7002m_get_frequency_cgen(self);
    case LMS7002M_CLK_RXTSP:
        return lms7002m_get_reference_clock_tsp(self, false);
    case LMS7002M_CLK_TXTSP:
        return lms7002m_get_reference_clock_tsp(self, true);
    default:
        lms7002m_report_error(self, lime_Result_InvalidValue, "Invalid clock ID.");
        return 0;
    }
}

float lms7002m_get_sample_rate(lms7002m_context* self, bool isTx, enum lms7002m_channel ch)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, ch);

    const uint16_t ratio = lms7002m_spi_read_csr(self, isTx ? LMS7002M_HBI_OVR_TXTSP : LMS7002M_HBD_OVR_RXTSP);

    float interface_Hz = lms7002m_get_reference_clock_tsp(self, isTx);

    // If decimation/interpolation is 0 (2^1) or 7 (bypass), interface clocks should not be divided
    if (ratio != 7)
    {
        interface_Hz /= 2 * pow(2.0, ratio);
    }

    lms7002m_set_active_channel(self, savedChannel);
    return interface_Hz;
}

lime_Result lms7002m_set_gfir_filter(lms7002m_context* self, bool isTx, enum lms7002m_channel ch, bool enabled, float bandwidth)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, ch);

    const bool bypassFIR = !enabled;
    if (isTx)
    {
        lms7002m_spi_modify_csr(self, LMS7002M_GFIR1_BYP_TXTSP, bypassFIR);
        lms7002m_spi_modify_csr(self, LMS7002M_GFIR2_BYP_TXTSP, bypassFIR);
        lms7002m_spi_modify_csr(self, LMS7002M_GFIR3_BYP_TXTSP, bypassFIR);
    }
    else
    {
        lms7002m_spi_modify_csr(self, LMS7002M_GFIR1_BYP_RXTSP, bypassFIR);
        lms7002m_spi_modify_csr(self, LMS7002M_GFIR2_BYP_RXTSP, bypassFIR);
        lms7002m_spi_modify_csr(self, LMS7002M_GFIR3_BYP_RXTSP, bypassFIR);

        const bool sisoDDR = lms7002m_spi_read_csr(self, LMS7002M_LML1_SISODDR);
        const bool clockIsNotInverted = !(enabled | sisoDDR);
        if (ch == LMS7002M_CHANNEL_B)
        {
            lms7002m_spi_modify_csr(self, LMS7002M_CDSN_RXBLML, clockIsNotInverted);
            lms7002m_spi_modify_csr(self, LMS7002M_CDS_RXBLML, enabled ? 3 : 0);
        }
        else
        {
            lms7002m_spi_modify_csr(self, LMS7002M_CDSN_RXALML, clockIsNotInverted);
            lms7002m_spi_modify_csr(self, LMS7002M_CDS_RXALML, enabled ? 3 : 0);
        }
    }
    if (!enabled)
    {
        lms7002m_set_active_channel(self, savedChannel);
        return lime_Result_Success;
    }

    if (bandwidth <= 0)
    {
        lms7002m_set_active_channel(self, savedChannel);
        return lime_Result_InvalidValue;
    }

    int ratio = 0;
    if (isTx)
    {
        ratio = lms7002m_spi_read_csr(self, LMS7002M_HBI_OVR_TXTSP);
    }
    else
    {
        ratio = lms7002m_spi_read_csr(self, LMS7002M_HBD_OVR_RXTSP);
    }

    int div = 1;
    if (ratio != 7)
        div = (2 << (ratio));

    bandwidth /= 1e6;
    const float interface_MHz = lms7002m_get_reference_clock_tsp(self, isTx) / 1e6;
    const float w = (bandwidth / 2) / (interface_MHz / div);

    const int L = div > 8 ? 8 : div;
    div -= 1;

    float w2 = w * 1.1;
    if (w2 > 0.495)
    {
        w2 = w * 1.05;
        if (w2 > 0.495)
        {
            lms7002m_log(self, lime_LogLevel_Error, "GFIR LPF cannot be set to the requested bandwidth (%f)", bandwidth);
            lms7002m_set_active_channel(self, savedChannel);
            return lime_Result_Error;
        }
    }

    float coef[120];
    float coef2[40];

    GenerateFilter(L * 15, w, w2, 1.0, 0, coef);
    GenerateFilter(L * 5, w, w2, 1.0, 0, coef2);

    if (isTx)
    {
        lms7002m_spi_modify_csr(self, LMS7002M_GFIR1_N_TXTSP, div);
        lms7002m_spi_modify_csr(self, LMS7002M_GFIR2_N_TXTSP, div);
        lms7002m_spi_modify_csr(self, LMS7002M_GFIR3_N_TXTSP, div);
    }
    else
    {
        lms7002m_spi_modify_csr(self, LMS7002M_GFIR1_N_RXTSP, div);
        lms7002m_spi_modify_csr(self, LMS7002M_GFIR2_N_RXTSP, div);
        lms7002m_spi_modify_csr(self, LMS7002M_GFIR3_N_RXTSP, div);
    }

    lime_Result status = lms7002m_set_gfir_coefficients(self, isTx, 0, coef2, L * 5);
    if (status != lime_Result_Success)
    {
        lms7002m_set_active_channel(self, savedChannel);
        return status;
    }

    status = lms7002m_set_gfir_coefficients(self, isTx, 1, coef2, L * 5);
    if (status != lime_Result_Success)
    {
        lms7002m_set_active_channel(self, savedChannel);
        return status;
    }

    status = lms7002m_set_gfir_coefficients(self, isTx, 2, coef, L * 15);
    if (status != lime_Result_Success)
    {
        lms7002m_set_active_channel(self, savedChannel);
        return status;
    }

    const lime_Result result = lms7002m_reset_logic_registers(self);
    lms7002m_set_active_channel(self, savedChannel);
    return result;
}

lime_Result lms7002m_set_rx_lpf(lms7002m_context* self, float rfBandwidth_Hz)
{
    const int tiaGain = lms7002m_spi_read_csr(self, LMS7002M_G_TIA_RFE);
    if (tiaGain < 1 || tiaGain > 3)
        return lms7002m_report_error(self, lime_Result_InvalidValue, "RxLPF: Invalid G_TIA gain value");

    lms7002m_spi_modify_csr(self, LMS7002M_PD_TIA_RFE, 0);
    lms7002m_spi_modify_csr(self, LMS7002M_EN_G_RFE, 1);

    lms7002m_spi_modify_csr(self, LMS7002M_ICT_TIAMAIN_RFE, 2);
    lms7002m_spi_modify_csr(self, LMS7002M_ICT_TIAOUT_RFE, 2);

    lms7002m_spi_modify_csr(self, LMS7002M_ICT_LPF_IN_RBB, 0x0C);
    lms7002m_spi_modify_csr(self, LMS7002M_ICT_LPF_OUT_RBB, 0x0C);

    lms7002m_spi_modify_csr(self, LMS7002M_ICT_PGA_OUT_RBB, 0x14);
    lms7002m_spi_modify_csr(self, LMS7002M_ICT_PGA_IN_RBB, 0x14);

    const int pgaGain = lms7002m_spi_read_csr(self, LMS7002M_G_PGA_RBB);
    if (pgaGain != 12)
    {
        lms7002m_log(self, lime_LogLevel_Warning, "RxLPF modifying G_PGA_RBB %i -> 12", pgaGain);
        lms7002m_spi_modify_csr(self, LMS7002M_G_PGA_RBB, 12);
    }

    lms7002m_spi_modify_csr(self, LMS7002M_RCC_CTL_PGA_RBB, 0x18);
    lms7002m_spi_modify_csr(self, LMS7002M_C_CTL_PGA_RBB, 1);

    const float rxLpfMin = (tiaGain == 1) ? 4e6 : 1.5e6;
    const float rxLpfMax = 160e6;

    if (rfBandwidth_Hz != 0 && (rfBandwidth_Hz < rxLpfMin || rfBandwidth_Hz > rxLpfMax))
    {
        lms7002m_log(self,
            lime_LogLevel_Warning,
            "Requested RxLPF(%g) is out of range [%g - %g]. Clamping to valid range.",
            rfBandwidth_Hz,
            rxLpfMin,
            rxLpfMax);
        rfBandwidth_Hz = clamp_float(rfBandwidth_Hz, rxLpfMin, rxLpfMax);
    }

    const float bandwidth_MHz = rfBandwidth_Hz / 1e6;

    uint16_t cfb_tia_rfe = 0;
    if (tiaGain == 1)
        cfb_tia_rfe = 120 * 45 / (bandwidth_MHz / 2 / 1.5) - 15;
    else
        cfb_tia_rfe = 120 * 14 / (bandwidth_MHz / 2 / 1.5) - 10;
    cfb_tia_rfe = clamp_uint(cfb_tia_rfe, 0, 4095);

    uint16_t rcomp_tia_rfe = clamp_uint(15 - cfb_tia_rfe * 2 / 100, 0, 15);
    uint16_t ccomp_tia_rfe = clamp_uint((cfb_tia_rfe / 100) + (tiaGain == 1 ? 1 : 0), 0, 15);

    uint16_t c_ctl_lpfl_rbb = clamp_uint(120 * 18 / (bandwidth_MHz / 2 / 0.75) - 103, 0, 2047);
    const uint16_t c_ctl_lpfh_rbb = clamp_uint(120 * 50 / (bandwidth_MHz / 2 / 0.75) - 50, 0, 255);

    lms7002m_log(self,
        lime_LogLevel_Debug,
        "RxLPF(%g): TIA_C=%i, TIA_RCOMP=%i, TIA_CCOMP=%i, RX_L_C=%i, RX_H_C=%i\n",
        rfBandwidth_Hz,
        cfb_tia_rfe,
        rcomp_tia_rfe,
        ccomp_tia_rfe,
        c_ctl_lpfl_rbb,
        c_ctl_lpfh_rbb);

    uint16_t input_ctl_pga_rbb = 4;
    uint16_t powerDowns = 0xD; // 0x0115[3:0]

    const float ifbw = bandwidth_MHz / 2 / 0.75;

    uint16_t rcc_ctl_lpfl_rbb = 0;
    if (ifbw >= 20)
        rcc_ctl_lpfl_rbb = 5;
    else if (ifbw >= 15)
        rcc_ctl_lpfl_rbb = 4;
    else if (ifbw >= 10)
        rcc_ctl_lpfl_rbb = 3;
    else if (ifbw >= 5)
        rcc_ctl_lpfl_rbb = 2;
    else if (ifbw >= 3)
        rcc_ctl_lpfl_rbb = 1;

    if (rfBandwidth_Hz <= 0) // LPF bypass
    {
        lms7002m_log(self, lime_LogLevel_Info, "RxLPF bypassed");
        powerDowns = 0xD;
        input_ctl_pga_rbb = 2;
    }
    else if (rfBandwidth_Hz < rxLpfMin)
    {
        lms7002m_log(
            self, lime_LogLevel_Warning, "RxLPF(%g) frequency too low. Clamping to %g MHz.", rfBandwidth_Hz, rxLpfMin / 1e6);
        if (tiaGain == 1)
        {
            cfb_tia_rfe = 4035;
            rcc_ctl_lpfl_rbb = 1;
            c_ctl_lpfl_rbb = 707;
        }
        else
        {
            cfb_tia_rfe = 3350;
            rcc_ctl_lpfl_rbb = 0;
            c_ctl_lpfl_rbb = 2047;
        }
        rcomp_tia_rfe = 0;
        ccomp_tia_rfe = 15;
        powerDowns = 0x9;
        input_ctl_pga_rbb = 0;
    }
    else if (rxLpfMin <= rfBandwidth_Hz && rfBandwidth_Hz <= 30e6)
    {
        powerDowns = 0x9;
        input_ctl_pga_rbb = 0;
    }
    else if (30e6 <= rfBandwidth_Hz && rfBandwidth_Hz <= rxLpfMax)
    {
        powerDowns = 0x5;
        input_ctl_pga_rbb = 1;
    }

    lms7002m_spi_modify_csr(self, LMS7002M_CFB_TIA_RFE, cfb_tia_rfe);
    lms7002m_spi_modify_csr(self, LMS7002M_RCOMP_TIA_RFE, rcomp_tia_rfe);
    lms7002m_spi_modify_csr(self, LMS7002M_CCOMP_TIA_RFE, ccomp_tia_rfe);
    lms7002m_spi_modify(self, 0x0115, 3, 0, powerDowns);
    lms7002m_spi_modify_csr(self, LMS7002M_INPUT_CTL_PGA_RBB, input_ctl_pga_rbb);
    lms7002m_spi_modify_csr(self, LMS7002M_C_CTL_LPFL_RBB, c_ctl_lpfl_rbb);
    lms7002m_spi_modify_csr(self, LMS7002M_C_CTL_LPFH_RBB, c_ctl_lpfh_rbb);
    lms7002m_spi_modify_csr(self, LMS7002M_RCC_CTL_LPFL_RBB, rcc_ctl_lpfl_rbb);

    const uint16_t rcc_ctl_lpfh_rbb = clamp_float(ifbw / 10 - 2, 0.0, 7.0);
    lms7002m_spi_modify_csr(self, LMS7002M_RCC_CTL_LPFH_RBB, rcc_ctl_lpfh_rbb);

    return lime_Result_Success;
}

lime_Result lms7002m_set_tx_lpf(lms7002m_context* self, float rfBandwidth_Hz)
{
    const float txLpfLowRange[2] = { 5e6, 33e6 };
    const float txLpfHighRange[2] = { 56e6, 160e6 };

    // common setup
    lms7002m_spi_modify(self, 0x0106, 15, 0, 0x318C);
    lms7002m_spi_modify(self, 0x0107, 15, 0, 0x318C);
    lms7002m_spi_modify_csr(self, LMS7002M_ICT_IAMP_FRP_TBB, 8);
    lms7002m_spi_modify_csr(self, LMS7002M_ICT_IAMP_GG_FRP_TBB, 12);
    lms7002m_spi_modify_csr(self, LMS7002M_CCAL_LPFLAD_TBB, 31);
    lms7002m_spi_modify_csr(self, LMS7002M_RCAL_LPFS5_TBB, 255);
    lms7002m_spi_modify_csr(self, LMS7002M_R5_LPF_BYP_TBB, 1);
    lms7002m_spi_modify_csr(self, LMS7002M_BYPLADDER_TBB, 0);

    uint16_t powerDowns = 0x15; // addr 0x0105[4:0]

    if (rfBandwidth_Hz <= 0) // Bypass LPF
    {
        lms7002m_log(self, lime_LogLevel_Info, "TxLPF bypassed");
        lms7002m_spi_modify(self, 0x0105, 4, 0, powerDowns);
        lms7002m_spi_modify_csr(self, LMS7002M_BYPLADDER_TBB, 1);
        return lms7002m_spi_modify_csr(self, LMS7002M_RCAL_LPFS5_TBB, 0);
    }
    else if (rfBandwidth_Hz < txLpfLowRange[0] || txLpfHighRange[1] < rfBandwidth_Hz)
    {
        lms7002m_log(self,
            lime_LogLevel_Warning,
            "Requested TxLPF(%g) bandwidth is out of range [%g - %g]. Clamping to valid value.",
            rfBandwidth_Hz,
            txLpfLowRange[0],
            txLpfHighRange[1]);
        rfBandwidth_Hz = clamp_float(rfBandwidth_Hz, txLpfLowRange[0], txLpfHighRange[1]);
    }

    uint8_t rcal_lpflad = 0;
    uint8_t rcal_lpfh = 0;

    if (rfBandwidth_Hz < 5.3e6)
    {
        lms7002m_log(self, lime_LogLevel_Warning, "TxLPF(%g) setting bandwidth to %g.", rfBandwidth_Hz, txLpfLowRange[0]);
        powerDowns = 0x11;
    }
    else if (rfBandwidth_Hz <= txLpfLowRange[1]) // 5.3-33 MHz
    {
#if 0
        const float rfbandwidth_MHz = rfBandwidth_Hz / 1e6;
        const double LADlog = 20.0 * log10(rfbandwidth_MHz / (2.6 * 2));
        double LADterm1 = 0.0;
        {
            double t1 = 1.92163e-15;
            double t2 = sqrt(5.9304678933309e99 * pow(LADlog, 2) - 1.64373265875744e101 * LADlog + 1.17784161390406e102);
            LADterm1 = t1 * pow(t2 + 7.70095311849832e49 * LADlog - 1.0672267662616e51, 1.0 / 3.0);
        }

        double LADterm2 = 0.0;
        {
            double t1 = 6.50934553014677e18;
            double t2 = sqrt(5.9304678933309e99 * pow(LADlog, 2) - 1.64373265875744e101 * LADlog + 1.17784161390406e102);
            double t3 = t2 + 7.70095311849832e49 * LADlog - 1.0672267662616e51;
            LADterm2 = t1 / pow(t3, 1.0 / 3.0);
        }
        rcal_lpflad = clamp_float(196.916 + LADterm1 - LADterm2, 0.0, 255.0);
#else
        int x = rfBandwidth_Hz / 1e5;

        if (x <= 85)
            rcal_lpflad = 1.04 * x - 54.4;
        else if (x <= 240)
            rcal_lpflad = 0.941 * x - 47.8;
        else
            rcal_lpflad = 0.839 * x - 17.7;
#endif
        powerDowns = 0x11;
    }
    else if (txLpfLowRange[1] <= rfBandwidth_Hz && rfBandwidth_Hz <= txLpfHighRange[0]) // 33-56 MHz gap
    {
        lms7002m_log(self,
            lime_LogLevel_Warning,
            "Requested TxLPF(%g) is in frequency gap [%g-%g], setting bandwidth to %g.",
            rfBandwidth_Hz,
            txLpfLowRange[1],
            txLpfHighRange[0],
            txLpfHighRange[0]);
        powerDowns = 0x07;
    }
    else if (rfBandwidth_Hz <= txLpfHighRange[1]) // <160MHz
    {
#if 0
        const float rfbandwidth_MHz = rfBandwidth_Hz / 1e6;
        const double Hlog = 20 * log10(rfbandwidth_MHz / (28 * 2));
        double Hterm1;
        {
            double t1 = 5.66735e-16;
            double t2 = sqrt(1.21443429517649e103 * pow(Hlog, 2) - 2.85279160551735e104 * Hlog + 1.72772373636442e105);
            double t3 = pow(t2 + 3.48487344845762e51 * Hlog - 4.09310646098208e052, 1.0 / 3.0);
            Hterm1 = t1 * t3;
        }
        double Hterm2;
        {
            double t1 = 2.12037432410767e019;
            double t2 = sqrt(1.21443429517649e103 * pow(Hlog, 2) - 2.85279160551735e104 * Hlog + 1.72772373636442e105);
            double t3 = pow(t2 + 3.48487344845762e51 * Hlog - 4.09310646098208e052, 1.0 / 3.0);
            Hterm2 = t1 / t3;
        }
        rcal_lpfh = clamp_float(197.429 + Hterm1 - Hterm2, 0.0, 255.0);
#else
        int x = rfBandwidth_Hz / 1e6;
        rcal_lpfh = x * 1.13 - 63.3;
#endif
        powerDowns = 0x07;
    }

    lms7002m_log(self, lime_LogLevel_Debug, "TxLPF(%g): LAD=%i, H=%i\n", rfBandwidth_Hz, rcal_lpflad, rcal_lpfh);

    lms7002m_spi_modify_csr(self, LMS7002M_RCAL_LPFLAD_TBB, rcal_lpflad);
    lms7002m_spi_modify_csr(self, LMS7002M_RCAL_LPFH_TBB, rcal_lpfh);
    return lms7002m_spi_modify(self, 0x0105, 4, 0, powerDowns);
}

uint16_t lms7002m_get_rssi_delay(lms7002m_context* self)
{
    const uint16_t sampleCount = (2 << 7) << lms7002m_spi_read_csr(self, LMS7002M_AGC_AVG_RXTSP);
    uint8_t decimation = lms7002m_spi_read_csr(self, LMS7002M_HBD_OVR_RXTSP);
    if (decimation < 6)
        decimation = (2 << decimation);
    else
        decimation = 1; //bypass

    float waitTime = sampleCount / ((lms7002m_get_reference_clock_tsp(self, false) / 2) / decimation);
    return (0xFFFF) - (uint16_t)(waitTime * lms7002m_get_reference_clock(self) / 12);
}

uint32_t lms7002m_get_rssi(lms7002m_context* self)
{
    uint32_t rssi;
    int waitTime = 1000000.0 * (0xFFFF - lms7002m_get_rssi_delay(self)) * 12 / lms7002m_get_reference_clock(self);
    lms7002m_sleep(waitTime);
    lms7002m_flip_rising_edge(self, &LMS7002M_CAPTURE);
    rssi = lms7002m_spi_read(self, 0x040F);
    return (rssi << 2 | (lms7002m_spi_read(self, 0x040E) & 0x3));
}

lime_Result lms7002m_load_dc_reg_iq(lms7002m_context* self, bool isTx, int16_t I, int16_t Q)
{
    if (isTx)
    {
        lms7002m_spi_modify_csr(self, LMS7002M_DC_REG_TXTSP, I);
        lms7002m_spi_modify_csr(self, LMS7002M_TSGDCLDI_TXTSP, 0);
        lms7002m_spi_modify_csr(self, LMS7002M_TSGDCLDI_TXTSP, 1);
        lms7002m_spi_modify_csr(self, LMS7002M_TSGDCLDI_TXTSP, 0);
        lms7002m_spi_modify_csr(self, LMS7002M_DC_REG_TXTSP, Q);
        lms7002m_spi_modify_csr(self, LMS7002M_TSGDCLDQ_TXTSP, 0);
        lms7002m_spi_modify_csr(self, LMS7002M_TSGDCLDQ_TXTSP, 1);
        lms7002m_spi_modify_csr(self, LMS7002M_TSGDCLDQ_TXTSP, 0);
    }
    else
    {
        lms7002m_spi_modify_csr(self, LMS7002M_DC_REG_RXTSP, I);
        lms7002m_spi_modify_csr(self, LMS7002M_TSGDCLDI_RXTSP, 0);
        lms7002m_spi_modify_csr(self, LMS7002M_TSGDCLDI_RXTSP, 1);
        lms7002m_spi_modify_csr(self, LMS7002M_TSGDCLDI_RXTSP, 0);
        lms7002m_spi_modify_csr(self, LMS7002M_DC_REG_RXTSP, Q);
        lms7002m_spi_modify_csr(self, LMS7002M_TSGDCLDQ_RXTSP, 0);
        lms7002m_spi_modify_csr(self, LMS7002M_TSGDCLDQ_RXTSP, 1);
        lms7002m_spi_modify_csr(self, LMS7002M_TSGDCLDQ_RXTSP, 0);
    }
    return lime_Result_Success;
}
