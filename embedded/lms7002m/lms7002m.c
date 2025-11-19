#include "limesuiteng/embedded/lms7002m/lms7002m.h"

#include "limesuiteng/embedded/loglevel.h"

#include "csr_data.h"
#include "lms7002m_context.h"
#include "privates.h"
#include "spi.h"
#include "spi_batch.h"

#ifdef FLOATING_POINT_AVAILABLE
    #include "lms_gfir.h"
#endif

#ifdef __KERNEL__
// TODO: fill linux kernel necessary headers
    #include <linux/kernel.h>
    #include <linux/slab.h>
#else
    #include <assert.h>
    #include <stddef.h>
//#include <stdlib.h> // might have inlined functions with floats, and would fail to compile with SSE disabled
void* malloc(size_t size);
void free(void* ptr);

    #include <string.h>
#endif // __KERNEL__

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

static inline void* lms7002m_malloc(size_t size)
{
#ifdef __KERNEL__
    return kmalloc(size, GFP_KERNEL);
#else
    return malloc(size);
#endif
}

static inline void lms7002m_free(void* ptr)
{
#ifdef __KERNEL__
    kfree(ptr);
#else
    free(ptr);
#endif
}

// convert frequency control word register value into Hz
static inline uint32_t nco_fcw_to_freq(uint32_t fcw, uint32_t tsp_ref_clk)
{
    // Add "0.5"(2147483648) in fixed point calculation for rounding purposes
    return ((uint64_t)tsp_ref_clk * fcw + 2147483648u) / 4294967295u;
}

// convert Hz into frequency control word
static inline uint32_t freq_to_nco_fcw(uint32_t freqOffset, uint32_t tsp_ref_clk)
{
    assert(tsp_ref_clk > 0);
    return (uint64_t)freqOffset * 4294967295u / tsp_ref_clk;
}

#define TO_DECIBEL(R) \
    (struct lms7002m_decibel) \
    { \
        ((int32_t)((R) * (1 << 16))) \
    }

struct lms7002m_context* lms7002m_create(const lms7002m_hooks* hooks)
{
    lms7002m_context* self = lms7002m_malloc(sizeof(lms7002m_context));
    if (self == NULL)
    {
        return self;
    }
    memset(self, 0, sizeof(lms7002m_context));
    self->reference_clock_hz = 30720000;

    memcpy(&self->hooks, hooks, sizeof(lms7002m_hooks));
    return self;
}

void lms7002m_destroy(lms7002m_context* context)
{
    if (context)
        lms7002m_free(context);
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

    // if (!enable)
    // {
    //     bool disable;
    //     if (channel == LMS7002M_CHANNEL_A)
    //         disable = lms7002m_spi_read_csr(self, isTx ? LMS7002M_TXEN_B : LMS7002M_RXEN_B) == 0;
    //     else
    //         disable = lms7002m_spi_read_csr(self, isTx ? LMS7002M_TXEN_A : LMS7002M_RXEN_A) == 0;
    //     lms7002m_spi_modify_csr(self, isTx ? LMS7002M_PD_TX_AFE1 : LMS7002M_PD_RX_AFE1, disable);
    // }
    // else
    //     lms7002m_spi_modify_csr(self, isTx ? LMS7002M_PD_TX_AFE1 : LMS7002M_PD_RX_AFE1, 0);

    // if (channel == LMS7002M_CHANNEL_B)
    //     lms7002m_spi_modify_csr(self, isTx ? LMS7002M_PD_TX_AFE2 : LMS7002M_PD_RX_AFE2, enable ? 0 : 1);

    int disabledChannels = (lms7002m_spi_read_bits(self, LMS7002M_PD_AFE.address, 4, 1) & 0xF); //check if all channels are disabled
    // lms7002m_spi_modify_csr(self, LMS7002M_EN_G_AFE, disabledChannels == 0xF ? 0 : 1);
    // lms7002m_spi_modify_csr(self, LMS7002M_PD_AFE, disabledChannels == 0xF ? 1 : 0);

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

uint32_t lms7002m_get_reference_clock(lms7002m_context* context)
{
    assert(context->reference_clock_hz > 0);
    return context->reference_clock_hz;
}

lime_Result lms7002m_set_reference_clock(lms7002m_context* context, uint32_t frequency_Hz)
{
    if (frequency_Hz <= 0)
        return lime_Result_InvalidValue;

    context->reference_clock_hz = frequency_Hz;
    return lime_Result_Success;
}

static inline uint8_t check_cgen_csw(lms7002m_context* self, uint8_t csw)
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

    // LMS7002M_LOG(self, lime_LogLevel_Debug, "csw %d; interval [%d, %d]", (cswHigh + cswLow) / 2, cswLow, cswHigh);
    uint8_t cmphl = check_cgen_csw(self, (cswHigh + cswLow) / 2);
    if (cmphl == 2)
        return lime_Result_Success;
    // lime::error("TuneVCO(CGEN) - failed to lock (cmphl!=%d)", cmphl);
    return lime_Result_Error;
}

lime_Result lms7002m_set_frequency_cgen(lms7002m_context* self, uint32_t freq_Hz)
{
    if (freq_Hz == 0)
        return lime_Result_InvalidValue;

    const uint32_t refClk = lms7002m_get_reference_clock(self);
    if (refClk == 0)
        return lime_Result_Error;

    const uint64_t cgen_vco_min = 1930000000;
    const uint64_t cgen_vco_max = 2940000000;
    if (freq_Hz > 640000000)
    {
        LMS7002M_LOG(self, lime_LogLevel_Error, "%s: requested frequency(%u) too high", __func__, freq_Hz);
        return lime_Result_OutOfRange;
    }

    //VCO frequency selection according to F_CLKH
    const uint16_t iHdiv_high = (cgen_vco_max / freq_Hz / 2) - 1;
    const uint16_t iHdiv_low = (cgen_vco_min / freq_Hz / 2);
    const uint16_t div_outch_cgen = ((iHdiv_low + iHdiv_high) / 2) & 0xFF;
    uint64_t vco = 2 * (div_outch_cgen + 1) * freq_Hz;
    if (vco <= cgen_vco_min || vco >= cgen_vco_max)
    {
        LMS7002M_LOG(self, lime_LogLevel_Error, "%s: cannot deliver requested frequency (%u Hz)", __func__, freq_Hz);
        return lime_Result_Error;
    }

    uint16_t integerPart = vco / refClk;
    vco -= integerPart * refClk;
    // "Fixed point number" division, take only the fraction part
    uint32_t fractionalPart = (vco << 20) / refClk;
    fractionalPart &= 0xFFFFF;

    integerPart -= 1;

    spi_batch_t batch;
    spi_batch_init(&batch);
    // shadowed registers take effect at the end of SPI transaction.
    // write registers in same SPI transaction to avoid intermediate PLL state changes
    spi_batch_modify_csr(&batch, LMS7002M_INT_SDM_CGEN, integerPart); //INT_SDM_CGEN
    spi_batch_modify(&batch, 0x0087, 15, 0, fractionalPart & 0xFFFF); //FRAC_SDM_CGEN[15:0]
    spi_batch_modify(&batch, 0x0088, 3, 0, fractionalPart >> 16); //FRAC_SDM_CGEN[19:16]
    spi_batch_modify_csr(&batch, LMS7002M_DIV_OUTCH_CGEN, div_outch_cgen); //DIV_OUTCH_CGEN
    lms7002m_spi_batch_flush(&batch, self);

    LMS7002M_LOG(self, lime_LogLevel_Debug, "INT %d, FRAC %d, DIV_OUTCH_CGEN %d", integerPart, fractionalPart, div_outch_cgen);
    LMS7002M_LOG(self, lime_LogLevel_Debug, "CGEN_VCO %lu Hz, RefClk %u Hz", vco, refClk);

    if (lms7002m_tune_cgen_vco(self) != lime_Result_Success)
    {
        LMS7002M_LOG(self, lime_LogLevel_Error, "%s:(%u Hz) failed", __func__, freq_Hz);
        return lime_Result_Error;
    }

    if (self->hooks.on_cgen_frequency_changed)
        return self->hooks.on_cgen_frequency_changed(self->hooks.on_cgen_frequency_changed_userData);

    return lime_Result_Success;
}

uint32_t lms7002m_get_frequency_cgen(lms7002m_context* self)
{
    const uint16_t div_outch_cgen = lms7002m_spi_read_csr(self, LMS7002M_DIV_OUTCH_CGEN);
    const uint16_t gINT = lms7002m_spi_read_bits(self, 0x0088, 13, 0); //read whole register to reduce SPI transfers
    const uint16_t lowerRegister = lms7002m_spi_read_bits(self, 0x0087, 15, 0);

    const uint32_t fractionalPart = ((gINT & 0xF) << 16) | lowerRegister;
    const uint16_t integerPart = (gINT >> 4) + 1;

    uint64_t cgenClk = lms7002m_get_reference_clock(self);

    // fixed point number
    uint64_t fp = ((integerPart << 20) | fractionalPart);
    cgenClk *= fp;
    cgenClk /= (div_outch_cgen + 1);
    cgenClk >>= 1;

    bool roundUp = cgenClk & 0x80000;
    cgenClk >>= 20; // leave only integer part
    if (roundUp)
        cgenClk += 1;
    return cgenClk;
}

static inline int16_t decibel_int(struct lms7002m_decibel a)
{
    return a.data >> 16;
}

static inline bool decibel_ge(struct lms7002m_decibel a, struct lms7002m_decibel b)
{
    return a.data >= b.data;
}

lime_Result lms7002m_set_rbbpga_db(lms7002m_context* self, const struct lms7002m_decibel value, const enum lms7002m_channel channel)
{
    // clang-format off
    const uint16_t rcc_ctl_pga_rbb_lookup_table[32] = {
        31, 30, 29, 29, 28, 27, 26, 26,
        25, 24, 24, 23, 23, 22, 22, 21,
        21, 20, 20, 19, 19, 19, 18, 18,
        18, 17, 17, 17, 16, 16, 16, 16,
    };
    // clang-format on
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, channel);

    uint16_t g_pga_rbb = clamp_int(decibel_int(value) + 12, 0, 31);
    lime_Result ret = lms7002m_spi_modify_csr(self, LMS7002M_G_PGA_RBB, g_pga_rbb);

    // This function was replaced with lookup table
    // int rcc_ctl_pga_rbb = (430.0 * pow(0.65, (g_pga_rbb / 10.0)) - 110.35) / 20.4516 + 16;
    uint16_t rcc_ctl_pga_rbb = rcc_ctl_pga_rbb_lookup_table[g_pga_rbb];

    uint16_t c_ctl_pga_rbb = 0;
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

struct lms7002m_decibel lms7002m_get_rbbpga_db(lms7002m_context* self, const enum lms7002m_channel channel)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, channel);

    uint16_t g_pga_rbb = lms7002m_spi_read_csr(self, LMS7002M_G_PGA_RBB);

    lms7002m_set_active_channel(self, savedChannel);

    struct lms7002m_decibel result = TO_DECIBEL(g_pga_rbb - 12);
    return result;
}

lime_Result lms7002m_set_rfelna_db(lms7002m_context* self, const struct lms7002m_decibel value, const enum lms7002m_channel channel)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, channel);

    const int32_t gmax = 30;
    int32_t val = decibel_int(value) - gmax;

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

struct lms7002m_decibel lms7002m_get_rfelna_db(lms7002m_context* self, const enum lms7002m_channel channel)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, channel);

    const int32_t gmax = 30;
    uint16_t g_lna_rfe = lms7002m_spi_read_csr(self, LMS7002M_G_LNA_RFE);

    int32_t retval = 0;
    const int32_t value_to_minus[16] = { 0, 30, 27, 24, 21, 18, 15, 12, 9, 6, 5, 4, 3, 2, 1, 0 };

    if (g_lna_rfe > 0 && g_lna_rfe < 16)
    {
        retval = gmax - value_to_minus[g_lna_rfe];
    }

    lms7002m_set_active_channel(self, savedChannel);
    return TO_DECIBEL(retval);
}

lime_Result lms7002m_set_rfe_loopback_lna_db(
    lms7002m_context* self, const struct lms7002m_decibel gain, const enum lms7002m_channel channel)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, channel);

    const int32_t gmax = 40;
    const struct lms7002m_decibel val = { gain.data - TO_DECIBEL(gmax).data };

    int g_rxloopb_rfe = 0;
    if (decibel_ge(val, TO_DECIBEL(0)))
        g_rxloopb_rfe = 15;
    else if (decibel_ge(val, TO_DECIBEL(-0.5)))
        g_rxloopb_rfe = 14;
    else if (decibel_ge(val, TO_DECIBEL(-1)))
        g_rxloopb_rfe = 13;
    else if (decibel_ge(val, TO_DECIBEL(-1.6)))
        g_rxloopb_rfe = 12;
    else if (decibel_ge(val, TO_DECIBEL(-2.4)))
        g_rxloopb_rfe = 11;
    else if (decibel_ge(val, TO_DECIBEL(-3)))
        g_rxloopb_rfe = 10;
    else if (decibel_ge(val, TO_DECIBEL(-4)))
        g_rxloopb_rfe = 9;
    else if (decibel_ge(val, TO_DECIBEL(-5)))
        g_rxloopb_rfe = 8;
    else if (decibel_ge(val, TO_DECIBEL(-6.2)))
        g_rxloopb_rfe = 7;
    else if (decibel_ge(val, TO_DECIBEL(-7.5)))
        g_rxloopb_rfe = 6;
    else if (decibel_ge(val, TO_DECIBEL(-9)))
        g_rxloopb_rfe = 5;
    else if (decibel_ge(val, TO_DECIBEL(-11)))
        g_rxloopb_rfe = 4;
    else if (decibel_ge(val, TO_DECIBEL(-14)))
        g_rxloopb_rfe = 3;
    else if (decibel_ge(val, TO_DECIBEL(-17)))
        g_rxloopb_rfe = 2;
    else if (decibel_ge(val, TO_DECIBEL(-24)))
        g_rxloopb_rfe = 1;

    lime_Result ret = lms7002m_spi_modify_csr(self, LMS7002M_G_RXLOOPB_RFE, g_rxloopb_rfe);

    lms7002m_set_active_channel(self, savedChannel);
    return ret;
}

struct lms7002m_decibel lms7002m_get_rfe_loopback_lna_db(lms7002m_context* self, const enum lms7002m_channel channel)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, channel);

    const uint32_t gmax = 40;
    uint16_t g_rxloopb_rfe = lms7002m_spi_read_csr(self, LMS7002M_G_RXLOOPB_RFE);

    struct lms7002m_decibel retval = TO_DECIBEL(0.0);
    const struct lms7002m_decibel value_to_minus[16] = { TO_DECIBEL(0),
        TO_DECIBEL(24),
        TO_DECIBEL(17),
        TO_DECIBEL(14),
        TO_DECIBEL(11),
        TO_DECIBEL(9),
        TO_DECIBEL(7.5),
        TO_DECIBEL(6.2),
        TO_DECIBEL(5),
        TO_DECIBEL(4),
        TO_DECIBEL(3),
        TO_DECIBEL(2.4),
        TO_DECIBEL(1.6),
        TO_DECIBEL(1),
        TO_DECIBEL(0.5),
        TO_DECIBEL(0) };

    if (g_rxloopb_rfe > 0 && g_rxloopb_rfe < 16)
    {
        retval.data = TO_DECIBEL(gmax).data - value_to_minus[g_rxloopb_rfe].data;
    }

    lms7002m_set_active_channel(self, savedChannel);
    return retval;
}

lime_Result lms7002m_set_rfetia_db(lms7002m_context* self, const struct lms7002m_decibel value, const enum lms7002m_channel channel)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, channel);

    const int32_t gmax = 12;
    int32_t val = decibel_int(value) - gmax;

    int g_tia_rfe = 1;

    if (val >= 0)
        g_tia_rfe = 3;
    else if (val >= -3)
        g_tia_rfe = 2;

    uint16_t ret = lms7002m_spi_modify_csr(self, LMS7002M_G_TIA_RFE, g_tia_rfe);
    lms7002m_set_active_channel(self, savedChannel);
    return ret;
}

struct lms7002m_decibel lms7002m_get_rfetia_db(lms7002m_context* self, const enum lms7002m_channel channel)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, channel);
    const uint8_t g_tia_rfe = lms7002m_spi_read_csr(self, LMS7002M_G_TIA_RFE);

    // g_tia_rfe value 0 is invalid
    const int32_t tia_lookup_table[4] = { 0, -12, -3, 0 };
    const int32_t dbmax = tia_lookup_table[g_tia_rfe];

    lms7002m_set_active_channel(self, savedChannel);
    return TO_DECIBEL(dbmax);
}

lime_Result lms7002m_set_trfpad_db(lms7002m_context* self, const struct lms7002m_decibel value, const enum lms7002m_channel channel)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, channel);

    const int32_t pmax = 52;
    int32_t loss_int = pmax - decibel_int(value);

    //different scaling realm
    if (loss_int > 10)
    {
        loss_int = (loss_int + 10) / 2;
    }

    loss_int = clamp_int(loss_int, 0, 31);

    lime_Result ret;
    lms7002m_spi_modify_csr(self, LMS7002M_LOSS_LIN_TXPAD_TRF, loss_int);
    ret = lms7002m_spi_modify_csr(self, LMS7002M_LOSS_MAIN_TXPAD_TRF, loss_int);

    lms7002m_set_active_channel(self, savedChannel);
    return ret;
}

struct lms7002m_decibel lms7002m_get_trfpad_db(lms7002m_context* self, const enum lms7002m_channel channel)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, channel);

    const int32_t pmax = 52;
    uint16_t loss_int = lms7002m_spi_read_csr(self, LMS7002M_LOSS_LIN_TXPAD_TRF);
    if (loss_int > 10)
    {
        lms7002m_set_active_channel(self, savedChannel);
        return TO_DECIBEL(pmax - 10 - 2 * (loss_int - 10));
    }

    lms7002m_set_active_channel(self, savedChannel);
    return TO_DECIBEL(pmax - loss_int);
}

lime_Result lms7002m_set_trf_loopback_pad_db(
    lms7002m_context* self, const struct lms7002m_decibel gain, const enum lms7002m_channel channel)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, channel);

    //there are 4 discrete gain values, use the midpoints
    int val = 3;
    if (decibel_ge(gain, TO_DECIBEL((-1.4 - 0) / 2)))
        val = 0;
    else if (decibel_ge(gain, TO_DECIBEL((-1.4 - 3.3) / 2)))
        val = 1;
    else if (decibel_ge(gain, TO_DECIBEL((-3.3 - 4.3) / 2)))
        val = 2;

    lime_Result ret = lms7002m_spi_modify_csr(self, LMS7002M_L_LOOPB_TXPAD_TRF, val);
    lms7002m_set_active_channel(self, savedChannel);
    return ret;
}

struct lms7002m_decibel lms7002m_get_trf_loopback_pad_db(lms7002m_context* self, const enum lms7002m_channel channel)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, channel);

    uint16_t regValue = lms7002m_spi_read_csr(self, LMS7002M_L_LOOPB_TXPAD_TRF);
    const struct lms7002m_decibel lookup_table[] = { TO_DECIBEL(0.0), TO_DECIBEL(-1.4), TO_DECIBEL(-3.3), TO_DECIBEL(-4.3) };

    struct lms7002m_decibel retval = regValue < 4 ? lookup_table[regValue] : TO_DECIBEL(0.0);

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
        return lime_Result_InvalidValue;
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
        ret = lms7002m_set_band_trf(self, path);
    else
        ret = lms7002m_set_path_rfe(self, path);

    lms7002m_set_active_channel(self, savedChannel);
    return ret;
}

uint32_t lms7002m_get_reference_clock_tsp(lms7002m_context* self, bool isTx)
{
    const uint32_t cgenFreq = lms7002m_get_frequency_cgen(self);
    const uint32_t clklfreq = cgenFreq >> lms7002m_spi_read_csr(self, LMS7002M_CLKH_OV_CLKL_CGEN);
    if (lms7002m_spi_read_csr(self, LMS7002M_EN_ADCCLKH_CLKGN) == 0)
        return isTx ? clklfreq : cgenFreq / 4;

    return isTx ? cgenFreq : clklfreq / 4;
}

// TODO: static
bool lms7002m_get_cgen_locked(lms7002m_context* self)
{
    return (lms7002m_spi_read_bits(self, LMS7002M_VCO_CMPHO_CGEN.address, 13, 12) & 0x3) == 0x2;
}

// TODO: static
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
        LMS7002M_LOG(self, lime_LogLevel_Error, "TuneVCO(%s) - VCO is powered down", moduleName);

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
        LMS7002M_LOG(self, lime_LogLevel_Debug, "TuneVCO(%s) - attempted VCO too high", moduleName);

        lms7002m_set_active_channel(self, savedChannel);
        return lime_Result_Error;
    }

    lms7002m_spi_modify(self, addrCSW_VCO, msb, lsb, 255);
    lms7002m_sleep(settlingTimeMicroseconds);
    cmphl = lms7002m_spi_read_bits(self, addrCMP, 13, 12);
    if (cmphl == 0) //VCO too low
    {
        LMS7002M_LOG(self, lime_LogLevel_Debug, "TuneVCO(%s) - attempted VCO too low", moduleName);

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
        LMS7002M_LOG(
            self, lime_LogLevel_Debug, "TuneVCO(%s) - searching interval [%i:%i]", moduleName, cswSearch[t].high, cswSearch[t].low);
        lms7002m_spi_modify(self, addrCSW_VCO, msb, lsb, cswSearch[t].high);
        //binary search for and high value, and on the way store approximate low value
        LMS7002M_LOG(self, lime_LogLevel_Debug, "%s", "binary search:");
        for (int i = 6; i >= 0; --i)
        {
            cswSearch[t].high |= 1 << i; //CSW_VCO<i>=1
            lms7002m_spi_modify(self, addrCSW_VCO, msb, lsb, cswSearch[t].high);
            lms7002m_sleep(settlingTimeMicroseconds);
            cmphl = lms7002m_spi_read_bits(self, addrCMP, 13, 12);
            LMS7002M_LOG(self, lime_LogLevel_Debug, "csw=%d\tcmphl=%d", cswSearch[t].high, cmphl);
            if (cmphl & 0x01) // reduce CSW
                cswSearch[t].high &= ~(1 << i); //CSW_VCO<i>=0
            if (cmphl == 2 && cswSearch[t].high < cswSearch[t].low)
            {
                cswSearch[t].low = cswSearch[t].high;
                hadLock = true;
            }
        }
        //linear search to make sure there are no gaps, and move away from edge case
        LMS7002M_LOG(self, lime_LogLevel_Debug, "%s", "adjust with linear search:");
        while (cswSearch[t].low <= cswSearch[t].high && cswSearch[t].low > t * 128)
        {
            --cswSearch[t].low;
            lms7002m_spi_modify(self, addrCSW_VCO, msb, lsb, cswSearch[t].low);
            lms7002m_sleep(settlingTimeMicroseconds);
            const uint8_t tempCMPvalue = lms7002m_spi_read_bits(self, addrCMP, 13, 12);
            LMS7002M_LOG(self, lime_LogLevel_Debug, "csw=%d\tcmphl=%d", cswSearch[t].low, tempCMPvalue);
            if (tempCMPvalue != 2)
            {
                ++cswSearch[t].low;
                break;
            }
        }
        if (hadLock)
        {
            LMS7002M_LOG(self,
                lime_LogLevel_Debug,
                "CSW: lowest=%d, highest=%d, will use=%d",
                cswSearch[t].low,
                cswSearch[t].high,
                cswSearch[t].low + (cswSearch[t].high - cswSearch[t].low) / 2);
        }
        else
            LMS7002M_LOG(self, lime_LogLevel_Debug, "%s", "CSW interval failed to lock");
    }

    //check if the intervals are joined
    int16_t cswHigh = 0, cswLow = 0;
    if (cswSearch[0].high == cswSearch[1].low - 1)
    {
        cswHigh = cswSearch[1].high;
        cswLow = cswSearch[0].low;
        LMS7002M_LOG(self, lime_LogLevel_Debug, "CSW is locking in one continuous range: low=%d, high=%d", cswLow, cswHigh);
    }
    //compare which interval is wider
    else
    {
        uint8_t intervalIndex = (cswSearch[1].high - cswSearch[1].low > cswSearch[0].high - cswSearch[0].low);
        cswHigh = cswSearch[intervalIndex].high;
        cswLow = cswSearch[intervalIndex].low;
        LMS7002M_LOG(self, lime_LogLevel_Debug, "choosing wider CSW locking range: low=%d, high=%d", cswLow, cswHigh);
    }

    uint8_t finalCSW = 0;
    if (cswHigh - cswLow <= 1)
    {
        LMS7002M_LOG(self,
            lime_LogLevel_Debug,
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
        LMS7002M_LOG(
            self, lime_LogLevel_Debug, "TuneVCO(%s) - failed lock with final csw=%i, cmphl=%i", moduleName, finalCSW, cmphl);
        return lime_Result_Error;
    }

    LMS7002M_LOG(
        self, lime_LogLevel_Debug, "TuneVCO(%s) - confirmed lock with final csw=%i, cmphl=%i", moduleName, finalCSW, cmphl);
    return lime_Result_Success;
}

static lime_Result lms7002m_write_sx_registers(
    lms7002m_context* self, const uint64_t VCOfreq_hz, uint32_t reference_clock_hz, uint8_t div_loch)
{
    const uint64_t m_dThrF = 5500000000; // VCO frequency threshold to enable additional divider
    const uint64_t divider = reference_clock_hz << (VCOfreq_hz > m_dThrF);

    uint16_t integerPart = VCOfreq_hz / divider;
    // "Fixed point number" division, take only the fraction part
    uint32_t fractionalPart = ((VCOfreq_hz - integerPart * divider) << 20) / divider;
    integerPart -= 4;

    spi_batch_t batch;
    spi_batch_init(&batch);
    // shadowed registers take effect at the end of SPI transaction.
    // write registers in same SPI transaction to avoid intermediate PLL state changes
    spi_batch_modify_csr(&batch, LMS7002M_EN_INTONLY_SDM, 0);
    spi_batch_modify_csr(&batch, LMS7002M_INT_SDM, integerPart); //INT_SDM
    spi_batch_modify(&batch, 0x011D, 15, 0, fractionalPart & 0xFFFF); //FRAC_SDM[15:0]
    spi_batch_modify(&batch, 0x011E, 3, 0, (fractionalPart >> 16)); //FRAC_SDM[19:16]
    spi_batch_modify_csr(&batch, LMS7002M_DIV_LOCH, div_loch); //DIV_LOCH
    spi_batch_modify_csr(&batch, LMS7002M_EN_DIV2_DIVPROG, (VCOfreq_hz > m_dThrF)); //EN_DIV2_DIVPROG
    lms7002m_spi_batch_flush(&batch, self);

    LMS7002M_LOG(self,
        lime_LogLevel_Debug,
        "SX VCO:%lu Hz, RefClk:%u Hz, INT:%u, FRAC:%u, DIV_LOCH:%u, EN_DIV2_DIVPROG:%d",
        VCOfreq_hz,
        reference_clock_hz,
        integerPart,
        fractionalPart,
        div_loch,
        (VCOfreq_hz > m_dThrF));

    return lime_Result_Success;
}

lime_Result lms7002m_set_frequency_sx(lms7002m_context* self, bool isTx, uint64_t LO_freq_hz)
{
    LMS7002M_LOG(self, lime_LogLevel_Debug, "Set %s LO frequency (%lu Hz)", isTx ? "Tx" : "Rx", LO_freq_hz);

    const char* const vcoNames[] = { "VCOL", "VCOM", "VCOH" };
    const uint64_t VCO_min_frequency[3] = { 3800000000, 4961000000, 6306000000 };
    const uint64_t VCO_max_frequency[3] = { 5222000000, 6754000000, 7714000000 };

    struct VCOData {
        uint64_t frequency;
        uint8_t div_loch;
        uint8_t csw;
        bool canDeliverFrequency;
    } vco[3];

    memset(vco, 0, sizeof(vco));

    bool canDeliverFrequency = false;

    //find required VCO frequency
    // div_loch value 7 is not allowed
    for (int8_t div_loch = 6; div_loch >= 0; --div_loch)
    {
        uint64_t VCOfreq = (1 << (div_loch + 1)) * (uint64_t)LO_freq_hz;
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
        LMS7002M_LOG(self,
            lime_LogLevel_Error,
            "%s: %s LO(%lu Hz) - VCO cannot deliver frequency.",
            __func__,
            isTx ? "Tx" : "Rx",
            LO_freq_hz);
        return lime_Result_Error;
    }

    const uint32_t refClk_Hz = lms7002m_get_reference_clock(self);
    if (refClk_Hz == 0)
        return lime_Result_Error;

    enum lms7002m_channel savedChannel =
        lms7002m_set_active_channel_readback(self, isTx ? LMS7002M_CHANNEL_SXT : LMS7002M_CHANNEL_SXR);

    // turn on VCO and comparator
    lms7002m_spi_modify_csr(self, LMS7002M_PD_VCO, 0);
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
                LMS7002M_LOG(self, lime_LogLevel_Debug, "%s skipped", vcoNames[sel_vco]);
                continue;
            }

            lms7002m_write_sx_registers(self, vco[sel_vco].frequency, refClk_Hz, vco[sel_vco].div_loch);
            LMS7002M_LOG(self, lime_LogLevel_Debug, "Tuning %s %s (ICT_VCO:%d):", (isTx ? "Tx" : "Rx"), vcoNames[sel_vco], ict_vco);

            lms7002m_spi_modify_csr(self, LMS7002M_SEL_VCO, sel_vco);
            lime_Result status = lms7002m_tune_vco(self, isTx ? LMS7002M_VCO_SXT : LMS7002M_VCO_SXR);
            if (status == lime_Result_Success)
            {
                vco[sel_vco].csw = lms7002m_spi_read_csr(self, LMS7002M_CSW_VCO);
                canDeliverFrequency = true;
                LMS7002M_LOG(self,
                    lime_LogLevel_Debug,
                    "%s : csw=%d %s",
                    vcoNames[sel_vco],
                    vco[sel_vco].csw,
                    (status == lime_Result_Success ? "tune ok" : "tune fail"));
                break;
            }
            else
                LMS7002M_LOG(self, lime_LogLevel_Debug, "%s : failed to lock", vcoNames[sel_vco]);
        }

        if (canDeliverFrequency)
        {
            LMS7002M_LOG(self, lime_LogLevel_Debug, "Selected: %s, CSW_VCO: %i", vcoNames[sel_vco], vco[sel_vco].csw);
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
    {
        LMS7002M_LOG(
            self, lime_LogLevel_Error, "%s: %s LO(%lu Hz) - cannot deliver frequency", __func__, isTx ? "Tx" : "Rx", LO_freq_hz);
        return lime_Result_Error;
    }
    return lime_Result_Success;
}

uint64_t lms7002m_get_frequency_sx(lms7002m_context* self, bool isTx)
{
    enum lms7002m_channel savedChannel =
        lms7002m_set_active_channel_readback(self, isTx ? LMS7002M_CHANNEL_SXT : LMS7002M_CHANNEL_SXR);

    const uint16_t gINT = lms7002m_spi_read_bits(self, 0x011E, 13, 0); // read whole register to reduce SPI transfers
    const uint16_t lowerRegister = lms7002m_spi_read_bits(self, 0x011D, 15, 0);
    const uint32_t fractionalPart = ((gINT & 0xF) << 16) | lowerRegister;
    const uint16_t integerPart = (gINT >> 4) + 4;

    uint64_t refClk_Hz = lms7002m_get_reference_clock(self);
    const uint16_t div_loch = lms7002m_spi_read_csr(self, LMS7002M_DIV_LOCH);
    const uint16_t en_div2_divprog = lms7002m_spi_read_csr(self, LMS7002M_EN_DIV2_DIVPROG);

    uint64_t vco = refClk_Hz << (en_div2_divprog);
    // fixed point number
    uint64_t fp = ((integerPart << 20) | fractionalPart);
    vco *= fp;
    vco >>= (div_loch + 1);
    bool roundUp = vco & 0x80000;
    vco >>= 20; // leave only integer part
    if (roundUp)
        vco += 1;

    lms7002m_set_active_channel(self, savedChannel);
    return vco;
}

lime_Result lms7002m_set_nco_frequency(lms7002m_context* self, bool isTx, const uint8_t index, uint32_t freq_Hz)
{
    if (index > 15)
    {
        LMS7002M_LOG(self, lime_LogLevel_Error, "%s: invalid NCO index %d.", __func__, index);
        return lime_Result_InvalidValue;
    }

    const uint32_t tsp_ref_clk = lms7002m_get_reference_clock_tsp(self, isTx);
    if (tsp_ref_clk == 0)
        return lime_Result_Error;

    if (2 * freq_Hz > tsp_ref_clk)
    {
        LMS7002M_LOG(
            self, lime_LogLevel_Error, "%s: NCO frequency(%u Hz) out of range [0-%u] Hz", __func__, freq_Hz, tsp_ref_clk / 2);
        return lime_Result_OutOfRange;
    }

    const uint16_t addr = isTx ? 0x0240 : 0x0440;
    const uint32_t fcw = freq_to_nco_fcw(freq_Hz, tsp_ref_clk);
    lms7002m_spi_write(self, addr + 2 + index * 2, (fcw >> 16)); //NCO frequency control word register MSB part.
    lms7002m_spi_write(self, addr + 3 + index * 2, fcw); //NCO frequency control word register LSB part.
    return lime_Result_Success;
}

uint32_t lms7002m_get_nco_frequency(lms7002m_context* self, bool isTx, const uint8_t index)
{
    if (index > 15)
    {
        LMS7002M_LOG(self, lime_LogLevel_Error, "%s: invalid NCO index %u.", __func__, index);
        return 0;
    }

    const uint32_t refClk_Hz = lms7002m_get_reference_clock_tsp(self, isTx);
    const uint16_t addr = isTx ? 0x0240 : 0x0440;
    uint32_t fcw = 0;
    fcw |= lms7002m_spi_read(self, addr + 2 + index * 2) << 16; //NCO frequency control word register MSB part.
    fcw |= lms7002m_spi_read(self, addr + 3 + index * 2); //NCO frequency control word register LSB part.

    return nco_fcw_to_freq(fcw, refClk_Hz);
}

lime_Result lms7002m_set_nco_phase_offset(lms7002m_context* self, bool isTx, uint8_t index, int16_t pho_calculated)
{
    if (index > 15)
    {
        LMS7002M_LOG(self, lime_LogLevel_Error, "%s: invalid NCO index %d.", __func__, index);
        return lime_Result_InvalidValue;
    }

    const uint16_t addr = isTx ? 0x0244 : 0x0444;
    lms7002m_spi_write(self, addr + index, pho_calculated);
    return lime_Result_Success;
}

static int16_t lms7002m_get_nco_phase_offset(lms7002m_context* self, bool isTx, uint8_t index)
{
    if (index > 15)
    {
        LMS7002M_LOG(self, lime_LogLevel_Error, "%s: invalid NCO index %d.", __func__, index);
        return lime_Result_InvalidValue;
    }

    const uint16_t addr = isTx ? 0x0244 : 0x0444;
    return lms7002m_spi_read(self, addr + index);
}

lime_Result lms7002m_set_nco_phase_offset_for_mode_0(lms7002m_context* self, bool isTx, int16_t pho_calculated)
{
    const uint16_t addr = isTx ? 0x0241 : 0x0441;
    lms7002m_spi_write(self, addr, pho_calculated);
    return lime_Result_Success;
}

lime_Result lms7002m_set_nco_phases(
    lms7002m_context* self, bool isTx, const int16_t* const angles_deg, uint8_t count, uint32_t frequencyOffset)
{
    assert(angles_deg);
    if (count > 16)
    {
        LMS7002M_LOG(self, lime_LogLevel_Error, "%s: invalid NCO count %d.", __func__, count);
        return lime_Result_OutOfRange;
    }

    lime_Result status = lms7002m_set_nco_frequency(self, isTx, 0, frequencyOffset);
    if (status != lime_Result_Success)
        return status;

    for (uint8_t i = 0; i < count; ++i)
    {
        status = lms7002m_set_nco_phase_offset(self, isTx, i, angles_deg[i]);
        if (status != lime_Result_Success)
            return status;
    }

    return lms7002m_spi_modify_csr(self, isTx ? LMS7002M_SEL_TX : LMS7002M_SEL_RX, 0);
}

lime_Result lms7002m_get_nco_phases(
    lms7002m_context* self, bool isTx, int16_t* const phoValues, uint8_t count, uint32_t* frequencyOffset)
{
    assert(phoValues);
    if (count > 16)
    {
        LMS7002M_LOG(self, lime_LogLevel_Error, "%s: invalid NCO count %d.", __func__, count);
        return lime_Result_OutOfRange;
    }

    for (int i = 0; i < count; ++i)
        phoValues[i] = lms7002m_get_nco_phase_offset(self, isTx, i);

    if (frequencyOffset != NULL)
        *frequencyOffset = lms7002m_get_nco_frequency(self, isTx, 0);

    return lime_Result_Success;
}

lime_Result lms7002m_set_nco_frequencies(
    lms7002m_context* self, bool isTx, const uint32_t* const freq_Hz, uint8_t count, int16_t phaseOffset)
{
    assert(freq_Hz);
    if (count > 16)
    {
        LMS7002M_LOG(self, lime_LogLevel_Error, "%s: invalid NCO count %d.", __func__, count);
        return lime_Result_OutOfRange;
    }

    for (uint8_t i = 0; i < count; ++i)
    {
        const lime_Result status = lms7002m_set_nco_frequency(self, isTx, i, freq_Hz[i]);
        if (status != lime_Result_Success)
            return status;
    }
    return lms7002m_set_nco_phase_offset_for_mode_0(self, isTx, phaseOffset);
}

lime_Result lms7002m_get_nco_frequencies(
    lms7002m_context* self, bool isTx, uint32_t* const freq_Hz, uint8_t count, int16_t* phaseOffset)
{
    assert(freq_Hz);
    if (count > 16)
    {
        LMS7002M_LOG(self, lime_LogLevel_Error, "%s: invalid NCO count %d.", __func__, count);
        return lime_Result_OutOfRange;
    }

    for (int i = 0; i < count; ++i)
        freq_Hz[i] = lms7002m_get_nco_frequency(self, isTx, i);

    if (phaseOffset != NULL)
        *phaseOffset = lms7002m_spi_read(self, isTx ? 0x0241 : 0x0441);

    return lime_Result_Success;
}

lime_Result lms7002m_set_gfir_coefficients(
    lms7002m_context* self, bool isTx, uint8_t gfirIndex, const int16_t* const coef, uint8_t coefCount)
{
    assert(coef);
    if (gfirIndex > 2)
        return lime_Result_OutOfRange;

    // [dependency] GFIR clock divider and coefficients count depends on TSP oversampling.
    // So sampling rate should be already configured, or has to update GFIR*_N.
    // TODO: But if coefficients max count is affected, what to do with coefficients?
    // If count is increased, coefficients could be reshuffled into expected bank rows and append extra 0
    // If count is decreased, some coefficients would be unused and would not produce the same output.
    const uint8_t ovr = lms7002m_spi_read_csr(self, isTx ? LMS7002M_HBI_OVR_TXTSP : LMS7002M_HBD_OVR_RXTSP);
    const uint8_t oversample = (ovr != 7) ? (2 << ovr) : 1; // 7 is bypass, otherwise 2**(ovr+1)

    const uint8_t bankCount = gfirIndex < 2 ? 5 : 15;
    const uint8_t bankLength = oversample > 8 ? 8 : oversample;

    const uint8_t maxCoefCount = bankCount * bankLength;
    // if coefCount is less than maxCoefCount, extra '0' coefficients will be written
    if (coefCount > maxCoefCount)
    {
        LMS7002M_LOG(self,
            lime_LogLevel_Error,
            "%s: given(%i) coefficients, but %s GFIR%i is limited to %i coefficients, by oversample(%i). "
            "Max coefficients count is equal to oversampling*%i, up to max of %i.",
            __func__,
            coefCount,
            (isTx ? "Tx" : "Rx"),
            gfirIndex + 1,
            maxCoefCount,
            oversample,
            bankCount,
            bankCount * 8);
        return lime_Result_OutOfRange;
    }

    struct lms7002m_csr gfirL_param = LMS7002M_GFIR1_L_TXTSP;
    gfirL_param.address += gfirIndex + (isTx ? 0 : 0x0200);
    // L = ceil(coefCount/bankCount) - 1
    lms7002m_spi_modify_csr(self, gfirL_param, bankLength - 1);

    struct lms7002m_csr gfirN_param = LMS7002M_GFIR1_N_TXTSP;
    gfirN_param.address += gfirIndex + (isTx ? 0 : 0x0200);
    // actual clock division ratio is gfirN + 1
    lms7002m_spi_modify_csr(self, gfirN_param, oversample - 1);

    const uint16_t startAddr = 0x0280 + (gfirIndex * 0x40) + (isTx ? 0 : 0x0200);
    for (int i = 0; i < maxCoefCount; ++i)
    {
        const uint8_t bank = i / bankLength;
        const uint8_t bankRow = i % bankLength;
        const uint16_t address = (startAddr + (bank * 8) + bankRow) + (24 * (bank / 5));
        uint16_t valueToWrite = 0;

        if (i < coefCount)
            valueToWrite = coef[i];

        lms7002m_spi_write(self, address, (uint16_t)valueToWrite);
    }

    return lime_Result_Success;
}

lime_Result lms7002m_get_gfir_coefficients(
    lms7002m_context* self, bool isTx, uint8_t gfirIndex, int16_t* const coef, uint8_t coefCount)
{
    assert(coef);

    if (gfirIndex > 2)
        return lime_Result_OutOfRange;

    // TODO: Readback coefficients in the order that they are used?
    // Actual used coefficients count and their addresses depends on oversampling
    // Coefficients readback won't match what was set, unless the maximum coefficients count was actually used.
    const uint8_t coefLimit = gfirIndex < 2 ? 40 : 120;

    if (coefCount > coefLimit)
    {
        LMS7002M_LOG(self,
            lime_LogLevel_Error,
            "lms7002m_get_gfir_coefficients: GFIR%i has %d coefficients, requested(%d)",
            gfirIndex + 1,
            coefLimit,
            coefCount);
        return lime_Result_OutOfRange;
    }

    const uint16_t startAddr = 0x0280 + (gfirIndex * 0x40) + (isTx ? 0 : 0x0200);
    for (uint8_t index = 0; index < coefCount; ++index)
    {
        const uint16_t registerValue = (uint16_t)(lms7002m_spi_read(self, startAddr + index + 24 * (index / 40)));
        coef[index] = registerValue;
    }

    return lime_Result_Success;
}

lime_Result lms7002m_set_interface_frequency(lms7002m_context* self, uint32_t cgen_freq_Hz, const uint8_t hbi, const uint8_t hbd)
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
        const uint8_t divider = 1 << (hbd + siso);
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
        const uint8_t divider = 1 << (hbi + siso);
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

    // [dependency] GFIR*_N clock dividers depend on HBI, HBD
    // TODO: also affects GFIR*L, which affects coefficients ordering, see: lms7002m_set_gfir_coefficients
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, LMS7002M_CHANNEL_A);
    for (int mac = 1; mac <= 2; ++mac)
    {
        lms7002m_spi_modify_csr(self, LMS7002M_MAC, mac);
        {
            uint8_t gfirN = (hbi == 7) ? 0 : (2 << hbi) - 1;
            lms7002m_spi_modify_csr(self, LMS7002M_GFIR1_N_TXTSP, gfirN);
            lms7002m_spi_modify_csr(self, LMS7002M_GFIR2_N_TXTSP, gfirN);
            lms7002m_spi_modify_csr(self, LMS7002M_GFIR3_N_TXTSP, gfirN);
        }
        {
            uint8_t gfirN = (hbd == 7) ? 0 : (2 << hbd) - 1;
            lms7002m_spi_modify_csr(self, LMS7002M_GFIR1_N_RXTSP, gfirN);
            lms7002m_spi_modify_csr(self, LMS7002M_GFIR2_N_RXTSP, gfirN);
            lms7002m_spi_modify_csr(self, LMS7002M_GFIR3_N_RXTSP, gfirN);
        }
    }
    lms7002m_set_active_channel(self, savedChannel);

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

lime_Result lms7002m_set_dc_offset(lms7002m_context* self, bool isTx, const uint8_t I, const uint8_t Q)
{
    const bool bypass = I == 0 && Q == 0;
    if (isTx)
    {
        lms7002m_spi_modify_csr(self, LMS7002M_DC_BYP_TXTSP, bypass ? 1 : 0);
        lms7002m_spi_modify_csr(self, LMS7002M_DCCORRI_TXTSP, I);
        lms7002m_spi_modify_csr(self, LMS7002M_DCCORRQ_TXTSP, Q);
    }
    else
    {
        lms7002m_spi_modify_csr(self, LMS7002M_EN_DCOFF_RXFE_RFE, bypass ? 0 : 1);
        lms7002m_spi_modify_csr(self, LMS7002M_DCOFFI_RFE, I);
        lms7002m_spi_modify_csr(self, LMS7002M_DCOFFQ_RFE, Q);
    }

    return lime_Result_Success;
}

lime_Result lms7002m_get_dc_offset(lms7002m_context* self, bool isTx, uint8_t* const I, uint8_t* const Q)
{
    assert(I);
    assert(Q);
    if (isTx)
    {
        *I = lms7002m_spi_read_csr(self, LMS7002M_DCCORRI_TXTSP);
        *Q = lms7002m_spi_read_csr(self, LMS7002M_DCCORRQ_TXTSP);
    }
    else
    {
        *I = lms7002m_spi_read_csr(self, LMS7002M_DCOFFI_RFE);
        *Q = lms7002m_spi_read_csr(self, LMS7002M_DCOFFQ_RFE);
    }

    return lime_Result_Success;
}

lime_Result lms7002m_set_i_q_balance(
    lms7002m_context* self, bool isTx, const int16_t iqcorr, const uint16_t gcorri, const uint16_t gcorrq)
{
    const bool bypassPhase = iqcorr == 0;
    const bool bypassGain = ((gcorri == 2047) && (gcorrq == 2047));

    lms7002m_spi_modify_csr(self, isTx ? LMS7002M_PH_BYP_TXTSP : LMS7002M_PH_BYP_RXTSP, bypassPhase ? 1 : 0);
    lms7002m_spi_modify_csr(self, isTx ? LMS7002M_GC_BYP_TXTSP : LMS7002M_GC_BYP_RXTSP, bypassGain ? 1 : 0);
    lms7002m_spi_modify_csr(self, isTx ? LMS7002M_IQCORR_TXTSP : LMS7002M_IQCORR_RXTSP, iqcorr);
    lms7002m_spi_modify_csr(self, isTx ? LMS7002M_GCORRI_TXTSP : LMS7002M_GCORRI_RXTSP, gcorri);
    lms7002m_spi_modify_csr(self, isTx ? LMS7002M_GCORRQ_TXTSP : LMS7002M_GCORRQ_RXTSP, gcorrq);
    return lime_Result_Success;
}

lime_Result lms7002m_get_i_q_balance(
    lms7002m_context* self, bool isTx, int16_t* const iqcorr, uint16_t* const gcorri, uint16_t* const gcorrq)
{
    if (iqcorr != NULL)
        *iqcorr = ((int16_t)lms7002m_spi_read_csr(self, isTx ? LMS7002M_IQCORR_TXTSP : LMS7002M_IQCORR_RXTSP) << 4) >>
                  4; //sign extend 12-bit
    if (gcorri != NULL)
        *gcorri = (uint16_t)(lms7002m_spi_read_csr(self, isTx ? LMS7002M_GCORRI_TXTSP : LMS7002M_GCORRI_RXTSP)); //unsigned 11-bit
    if (gcorrq != NULL)
        *gcorrq = (uint16_t)(lms7002m_spi_read_csr(self, isTx ? LMS7002M_GCORRQ_TXTSP : LMS7002M_GCORRQ_RXTSP)); //unsigned 11-bit

    return lime_Result_Success;
}

lime_Result lms7002m_get_temperature(lms7002m_context* self, int32_t* milliCelsius)
{
    lime_Result ret = lms7002m_calibrate_internal_adc(self, 32);
    if (ret != lime_Result_Success)
        return ret;
    lms7002m_spi_modify_csr(self, LMS7002M_RSSI_PD, 0);
    lms7002m_spi_modify_csr(self, LMS7002M_RSSI_RSSIMODE, 0);
    const uint16_t biasMux = lms7002m_spi_read_csr(self, LMS7002M_MUX_BIAS_OUT);
    lms7002m_spi_modify_csr(self, LMS7002M_MUX_BIAS_OUT, 2);

    lms7002m_sleep(250);

    const uint16_t reg606 = lms7002m_spi_read(self, 0x0606);
    lms7002m_spi_modify_csr(self, LMS7002M_MUX_BIAS_OUT, biasMux);

    const int32_t Vtemp = ((reg606 >> 8) & 0xFF);
    const int32_t Vptat = (reg606 & 0xFF);
    const int32_t Vdiff = (Vptat - Vtemp) * 1752; // * 1840 / 1050;
    const int32_t temperature_mC = 45000 + Vdiff;
    LMS7002M_LOG(self, lime_LogLevel_Debug, "Vtemp(0x%02X), Vptat(0x%02X), temp=%d mC", Vtemp, Vptat, temperature_mC);

    if (milliCelsius != NULL)
        *milliCelsius = temperature_mC;
    return lime_Result_Success;
}

lime_Result lms7002m_set_clock_frequency(lms7002m_context* self, enum lms7002m_clock_id clk_id, uint32_t freq)
{
    switch (clk_id)
    {
    case LMS7002M_CLK_REFERENCE:
        return lms7002m_set_reference_clock(self, freq);
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
    case LMS7002M_CLK_TXTSP: {
        LMS7002M_LOG(self, lime_LogLevel_Error, "%s: RxTSP/TxTSP Clocks are read only.", __func__);
        return lime_Result_InvalidValue;
    }
    default: {
        LMS7002M_LOG(self, lime_LogLevel_Error, "%s: Unknown clock id.", __func__);
        return lime_Result_InvalidValue;
    }
    }
    return lime_Result_Success;
}

uint32_t lms7002m_get_clock_frequency(lms7002m_context* self, enum lms7002m_clock_id clk_id)
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
        LMS7002M_LOG(self, lime_LogLevel_Error, "%s: Invalid clock ID.", __func__);
        return 0;
    }
}

uint32_t lms7002m_get_sample_rate(lms7002m_context* self, bool isTx, enum lms7002m_channel ch)
{
    enum lms7002m_channel savedChannel = lms7002m_set_active_channel_readback(self, ch);

    const uint16_t ratio = lms7002m_spi_read_csr(self, isTx ? LMS7002M_HBI_OVR_TXTSP : LMS7002M_HBD_OVR_RXTSP);

    uint32_t interface_Hz = lms7002m_get_reference_clock_tsp(self, isTx);

    // If decimation/interpolation is 0 (2^1) or 7 (bypass), interface clocks should not be divided
    if (ratio != 7)
    {
        // This function was replaced with right bit shift
        // interface_Hz /= 2 * pow(2.0, ratio);
        interface_Hz >>= 1 + ratio;
    }

    lms7002m_set_active_channel(self, savedChannel);
    return interface_Hz;
}

lime_Result lms7002m_set_rx_lpf(lms7002m_context* self, uint32_t rfBandwidth_Hz)
{
    if (rfBandwidth_Hz == 0)
    {
        LMS7002M_LOG(self, lime_LogLevel_Debug, "%s: RxLPF bypassed", __func__);
        uint16_t powerDowns = 0xD;
        lms7002m_spi_modify_csr(self, LMS7002M_INPUT_CTL_PGA_RBB, 2);
        lms7002m_spi_modify(self, 0x0115, 3, 0, powerDowns);
        return lime_Result_Success;
    }

    const int tiaGain = lms7002m_spi_read_csr(self, LMS7002M_G_TIA_RFE);
    if (tiaGain == 0)
    {
        LMS7002M_LOG(self, lime_LogLevel_Error, "%s: invalid G_TIA gain value.", __func__);
        return lime_Result_InvalidValue;
    }

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
        LMS7002M_LOG(self, lime_LogLevel_Warning, "%s: modifying G_PGA_RBB %i -> 12", __func__, pgaGain);
        lms7002m_spi_modify_csr(self, LMS7002M_G_PGA_RBB, 12);
    }

    lms7002m_spi_modify_csr(self, LMS7002M_RCC_CTL_PGA_RBB, 0x18);
    lms7002m_spi_modify_csr(self, LMS7002M_C_CTL_PGA_RBB, 1);

    const uint32_t rxLpfMin = (tiaGain == 1) ? 4000000 : 1500000;
    const uint32_t rxLpfMax = 160000000;

    if (rfBandwidth_Hz != 0 && (rfBandwidth_Hz < rxLpfMin || rfBandwidth_Hz > rxLpfMax))
    {
        LMS7002M_LOG(self,
            lime_LogLevel_Warning,
            "%s: Requested bandwidth(%u). Clamping to valid range [%u - %u]",
            __func__,
            rfBandwidth_Hz,
            rxLpfMin,
            rxLpfMax);
        rfBandwidth_Hz = clamp_uint(rfBandwidth_Hz, rxLpfMin, rxLpfMax);
    }

    uint16_t cfb_tia_rfe = 0;
    if (tiaGain == 1)
        cfb_tia_rfe = ((uint64_t)120000000 * 45 * 3 / rfBandwidth_Hz) - 15;
    else
        cfb_tia_rfe = ((uint64_t)120000000 * 14 * 3 / rfBandwidth_Hz) - 10;
    cfb_tia_rfe = clamp_uint(cfb_tia_rfe, 0, 4095);

    uint16_t rcomp_tia_rfe = clamp_uint(15 - cfb_tia_rfe * 2 / 100, 0, 15);
    uint16_t ccomp_tia_rfe = clamp_uint((cfb_tia_rfe / 100) + (tiaGain == 1 ? 1 : 0), 0, 15);

    uint16_t c_ctl_lpfl_rbb = clamp_uint(((uint64_t)120000000 * 18 / (rfBandwidth_Hz * 2 / 3)) - 103, 0, 2047);
    const uint16_t c_ctl_lpfh_rbb = clamp_uint(((uint64_t)120000000 * 50 / (rfBandwidth_Hz * 2 / 3)) - 50, 0, 255);

    LMS7002M_LOG(self,
        lime_LogLevel_Debug,
        "%s: bandwidth(%u): TIA_C=%i, TIA_RCOMP=%i, TIA_CCOMP=%i, RX_L_C=%i, RX_H_C=%i\n",
        __func__,
        rfBandwidth_Hz,
        cfb_tia_rfe,
        rcomp_tia_rfe,
        ccomp_tia_rfe,
        c_ctl_lpfl_rbb,
        c_ctl_lpfh_rbb);

    uint16_t input_ctl_pga_rbb = 4;
    uint16_t powerDowns = 0xD; // 0x0115[3:0]

    const uint32_t ifbw = rfBandwidth_Hz * 2 / 3;

    uint16_t rcc_ctl_lpfl_rbb = 0;
    if (ifbw >= 20000000)
        rcc_ctl_lpfl_rbb = 5;
    else if (ifbw >= 15000000)
        rcc_ctl_lpfl_rbb = 4;
    else if (ifbw >= 10000000)
        rcc_ctl_lpfl_rbb = 3;
    else if (ifbw >= 5000000)
        rcc_ctl_lpfl_rbb = 2;
    else if (ifbw >= 3000000)
        rcc_ctl_lpfl_rbb = 1;

    if (rfBandwidth_Hz < rxLpfMin)
    {
        LMS7002M_LOG(self,
            lime_LogLevel_Warning,
            "%s: bandwidth(%u) frequency too low. Clamping to %u Hz.",
            __func__,
            rfBandwidth_Hz,
            rxLpfMin);
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
    else if (rxLpfMin <= rfBandwidth_Hz && rfBandwidth_Hz <= 30000000)
    {
        powerDowns = 0x9;
        input_ctl_pga_rbb = 0;
    }
    else if (30000000 <= rfBandwidth_Hz && rfBandwidth_Hz <= rxLpfMax)
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

    const uint16_t rcc_ctl_lpfh_rbb = clamp_int(ifbw / 10000000 - 2, 0, 7);
    lms7002m_spi_modify_csr(self, LMS7002M_RCC_CTL_LPFH_RBB, rcc_ctl_lpfh_rbb);

    return lime_Result_Success;
}

lime_Result lms7002m_set_tx_lpf(lms7002m_context* self, uint32_t rfBandwidth_Hz)
{
    const uint32_t txLpfLowRange[2] = { 5000000, 33000000 };
    const uint32_t txLpfHighRange[2] = { 56000000, 160000000 };

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
        LMS7002M_LOG(self, lime_LogLevel_Debug, "%s: TxLPF bypassed.", __func__);
        lms7002m_spi_modify(self, 0x0105, 4, 0, powerDowns);
        lms7002m_spi_modify_csr(self, LMS7002M_BYPLADDER_TBB, 1);
        return lms7002m_spi_modify_csr(self, LMS7002M_RCAL_LPFS5_TBB, 0);
    }
    else if (rfBandwidth_Hz < txLpfLowRange[0] || txLpfHighRange[1] < rfBandwidth_Hz)
    {
        LMS7002M_LOG(self,
            lime_LogLevel_Warning,
            "%s: requested bandwidth(%u). Clamping to range [%u - %u]",
            __func__,
            rfBandwidth_Hz,
            txLpfLowRange[0],
            txLpfHighRange[1]);
        rfBandwidth_Hz = clamp_uint(rfBandwidth_Hz, txLpfLowRange[0], txLpfHighRange[1]);
    }

    uint8_t rcal_lpflad = 0;
    uint8_t rcal_lpfh = 0;

    if (rfBandwidth_Hz < 5300000)
    {
        LMS7002M_LOG(
            self, lime_LogLevel_Warning, "%s: requested bandwidth(%u), set to %u.", __func__, rfBandwidth_Hz, txLpfLowRange[0]);
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
        // approximation using only integers
        int x = rfBandwidth_Hz / 100000;
        if (x <= 85)
            rcal_lpflad = (104 * x - 5440) / 100; // 1.04 * x - 54.4
        else if (x <= 240)
            rcal_lpflad = (941 * x - 47800) / 1000; // 0.941 * x - 47.8
        else
            rcal_lpflad = (839 * x - 17700) / 1000; // 0.839 * x - 17.7
#endif
        powerDowns = 0x11;
    }
    else if (txLpfLowRange[1] <= rfBandwidth_Hz && rfBandwidth_Hz <= txLpfHighRange[0]) // 33-56 MHz gap
    {
        LMS7002M_LOG(self,
            lime_LogLevel_Warning,
            "%s: requested bandwidth(%u) is in frequency gap [%u-%u], setting bandwidth to %u.",
            __func__,
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
        // approximation using only integers
        int x = rfBandwidth_Hz / 1000000;
        rcal_lpfh = (x * 113 - 6330) / 100; // x * 1.13 - 63.3
#endif
        powerDowns = 0x07;
    }

    LMS7002M_LOG(self, lime_LogLevel_Debug, "%s: bandwidth(%u): LAD=%i, H=%i\n", __func__, rfBandwidth_Hz, rcal_lpflad, rcal_lpfh);

    lms7002m_spi_modify_csr(self, LMS7002M_RCAL_LPFLAD_TBB, rcal_lpflad);
    lms7002m_spi_modify_csr(self, LMS7002M_RCAL_LPFH_TBB, rcal_lpfh);
    return lms7002m_spi_modify(self, 0x0105, 4, 0, powerDowns);
}

static uint16_t lms7002m_get_rssi_delay(lms7002m_context* self)
{
    const uint16_t sampleCount = (2 << 7) << lms7002m_spi_read_csr(self, LMS7002M_AGC_AVG_RXTSP); // 0-7
    uint8_t decimation = lms7002m_spi_read_csr(self, LMS7002M_HBD_OVR_RXTSP); // 0-7

    // 5 and 6 are undefined decimation ratios
    if (decimation < 5)
        decimation = (2 << decimation);
    else
        decimation = 1; //bypass

    const uint64_t ref_clk = lms7002m_get_reference_clock(self);
    const uint64_t ref_clk_tsp = lms7002m_get_reference_clock_tsp(self, false);
    uint64_t delay = (uint64_t)sampleCount * decimation * ref_clk / ref_clk_tsp / 6;
    return (0xFFFF) - (uint16_t)(delay);
}

uint32_t lms7002m_get_rssi(lms7002m_context* self)
{
    EXPECT(self, lms7002m_spi_read_csr(self, LMS7002M_AGC_BYP_RXTSP) == 0); // ensure AGC is enabled, otherwise RSSI value will be 0
    uint32_t rssi;
    int waitTime = 1000000 * (0xFFFF - lms7002m_get_rssi_delay(self)) * 12 / lms7002m_get_reference_clock(self);
    lms7002m_sleep(waitTime);
    lms7002m_trigger_rising_edge(self, &LMS7002M_CAPTURE);
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
