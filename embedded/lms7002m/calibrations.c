#include "limesuiteng/embedded/lms7002m/lms7002m.h"
#include "limesuiteng/embedded/types.h"

#include "csr_data.h"
#include "lms7002m_context.h"
#include "privates.h"
#include "spi.h"

#include <string.h>

#ifdef __KERNEL__
// TODO: add linux kernel headers
#else
    #include <stdio.h>
#endif

static const uint8_t calibUserBwDivider = 5;
static const uint32_t offsetNCO = 100000;
static const uint32_t calibrationSXOffset_Hz = 1000000;

static const char* const get_lna_name(uint16_t value)
{
    switch (value)
    {
    case 0:
    default:
        return "none";
    case 1:
        return "LNAH";
    case 2:
        return "LNAL";
    case 3:
        return "LNAW";
    }
}

// basic math functions
#define math_abs(x) (x >= 0 ? x : -x)
#define math_pow2(power) (1 << power)

///APPROXIMATE conversion
// static const uint32_t maxRSSI = 0x15FF4; // 90100
// static float chip_rssi_to_dbfs(uint32_t rssi)
// {
//     if (rssi == 0)
//         rssi = 1;
//     return 20 * log10((float)(rssi) / maxRSSI);
// }

// static uint32_t dbfs_to_chip_rssi(float dBFS)
// {
//     return maxRSSI * pow(10.0, dBFS / 20.0);
// }

static const char* rssi_to_string(uint32_t rssi)
{
    static char rssi_string[64];
    snprintf(rssi_string, sizeof(rssi_string), "(RSSI: 0x%05X)", rssi);
    return rssi_string;
}

inline static int16_t to_signed(int16_t val, uint8_t msb)
{
    val <<= 15 - msb;
    val >>= 15 - msb;
    return val;
}

typedef struct {
    const uint16_t* addr;
    const uint16_t* val;
    const uint16_t* mask;
    const uint8_t cnt;
    const uint16_t* wrOnlyAddr;
    const uint16_t* wrOnlyData;
    const uint8_t wrOnlyAddrCnt;
    const uint8_t wrOnlyDataCnt;
} RegisterBatch;

static void lms7002m_write_masked_registers(lms7002m_context* self, const RegisterBatch* regs)
{
    for (uint8_t i = regs->cnt; i; --i)
    {
        const uint8_t index = i - 1;
        lms7002m_spi_write(
            self, regs->addr[index], (lms7002m_spi_read(self, regs->addr[index]) & ~regs->mask[index]) | regs->val[index]);
    }
    for (uint8_t i = regs->wrOnlyAddrCnt; i; --i)
    {
        const uint8_t index = i - 1;
        lms7002m_spi_write(self, regs->wrOnlyAddr[index], i > regs->wrOnlyDataCnt ? 0 : regs->wrOnlyData[index]);
    }
}

static void lms7002m_set_defaults_sx(lms7002m_context* self)
{
    const uint16_t SXAddr[] = { 0x011C, 0x011D, 0x011E, 0x011F, 0x0121, 0x0122, 0x0123 };
    const uint16_t SXdefVals[] = { 0xAD43, 0x0400, 0x0780, 0x3640, 0x3404, 0x033F, 0x067B };

    for (uint8_t i = 0; i < sizeof(SXAddr) / sizeof(uint16_t); ++i)
        lms7002m_spi_write(self, SXAddr[i], SXdefVals[i]);

    //keep 0x0120[7:0]ICT_VCO bias value intact
    lms7002m_spi_modify_csr(self, LMS7002M_VDIV_VCO, 0xB9FF);
}

static bool lms7002m_is_pll_tuned(lms7002m_context* self, const bool isTx)
{
    if (lms7002m_spi_read_bits(self, 0x0123, 13, 12) == 2)
        return true;
    lime_Result result = lms7002m_tune_vco(self, isTx ? LMS7002M_VCO_SXT : LMS7002M_VCO_SXR);
    return result == lime_Result_Success;
}

static void lms7002m_load_dc_reg_tx_iq(lms7002m_context* self)
{
    lms7002m_spi_write(self, 0x020C, 0x7FFF);
    lms7002m_trigger_rising_edge(self, &LMS7002M_TSGDCLDI_TXTSP);
    lms7002m_spi_write(self, 0x020C, 0x8000);
    lms7002m_trigger_rising_edge(self, &LMS7002M_TSGDCLDQ_TXTSP);
}

static lime_Result lms7002m_setup_cgen(lms7002m_context* self)
{
    //const uint8_t cgenMultiplier = clamp_int((lms7002m_get_frequency_cgen(self) / 46.08e6) + 0.5, 2, 13);
    const uint8_t cgenMultiplier = clamp_int((lms7002m_get_frequency_cgen(self) / 46080000), 2, 13);
    uint8_t gfir3n = 4 * cgenMultiplier;
    if (lms7002m_spi_read_csr(self, LMS7002M_EN_ADCCLKH_CLKGN) == 1)
    {
        gfir3n /= math_pow2(lms7002m_spi_read_csr(self, LMS7002M_CLKH_OV_CLKL_CGEN));
    }

    uint8_t power = 0x3F;
    for (; power; power >>= 1)
    {
        if (gfir3n >= power)
        {
            break;
        }
    }
    lms7002m_spi_modify_csr(self, LMS7002M_GFIR3_N_RXTSP, power);

    //CGEN VCO is powered up in SetFrequencyCGEN/Tune

    // disable CGEN callback during calibration, to avoid unnecessary dependencies configuration
    lms7002m_on_cgen_frequency_changed_hook callbackStore = self->hooks.on_cgen_frequency_changed;
    self->hooks.on_cgen_frequency_changed = NULL;
    lime_Result result = lms7002m_set_frequency_cgen(self, 46080000 * cgenMultiplier);
    self->hooks.on_cgen_frequency_changed = callbackStore;
    return result;
}

static void lms7002m_set_rx_gfir3_coefficients(lms7002m_context* self)
{
    //FIR coefficients symmetrical, storing only one half
    const int16_t firCoeffs[] = {
        8,
        4,
        0,
        -6,
        -11,
        -16,
        -20,
        -22,
        -22,
        -20,
        -14,
        -5,
        6,
        20,
        34,
        46,
        56,
        61,
        58,
        48,
        29,
        3,
        -29,
        -63,
        -96,
        -123,
        -140,
        -142,
        -128,
        -94,
        -44,
        20,
        93,
        167,
        232,
        280,
        302,
        291,
        244,
        159,
        41,
        -102,
        -258,
        -409,
        -539,
        -628,
        -658,
        -614,
        -486,
        -269,
        34,
        413,
        852,
        1328,
        1814,
        2280,
        2697,
        3038,
        3277,
        3401,
    };
    uint8_t index = 0;
    for (; index < sizeof(firCoeffs) / sizeof(int16_t); ++index)
        lms7002m_spi_write(self, 0x0500 + index + 24 * (index / 40), firCoeffs[index]);
    for (; index < sizeof(firCoeffs) / sizeof(int16_t) * 2; ++index)
        lms7002m_spi_write(self, 0x0500 + index + 24 * (index / 40), firCoeffs[119 - index]);
}

static void lms7002m_enable_mimo_buffers_if_necessary(lms7002m_context* self)
{
    //modifications when calibrating channel B
    const uint16_t x0020val = lms7002m_spi_read(self, 0x0020);
    if ((x0020val & 0x3) == 2)
    {
        lms7002m_spi_modify_csr(self, LMS7002M_MAC, 1);
        lms7002m_spi_modify_csr(self, LMS7002M_EN_NEXTRX_RFE, 1);
        lms7002m_spi_modify_csr(self, LMS7002M_EN_NEXTTX_TRF, 1);
        lms7002m_spi_write(self, 0x0020, x0020val);
    }
}

static void lms7002m_enable_channel_power_controls(lms7002m_context* self)
{
    uint16_t afe = lms7002m_spi_read(self, 0x0082);
    uint16_t value = lms7002m_spi_read(self, 0x0020);
    if ((value & 3) == 1)
    {
        value = value | 0x0014;
        afe &= ~0x14;
    }
    else
    {
        value = value | 0x0028;
        afe &= ~0x0A;
    }
    lms7002m_spi_write(self, 0x0020, value);
    lms7002m_spi_write(self, 0x0082, afe);
}

static lime_Result lms7002m_calibrate_rx_setup(lms7002m_context* self, uint32_t bandwidthRF, bool extLoopback)
{
    const uint16_t x0020val = lms7002m_spi_read(self, 0x0020);
    //rfe
    {
        const uint16_t RxSetupAddr[] = { 0x0084, 0x0085, 0x00AE, 0x010C, 0x010D, 0x0113, 0x0115, 0x0119, 0x0020 };
        const uint16_t RxSetupData[] = { 0x0400, 0x0001, 0xF000, 0x0000, 0x0046, 0x000C, 0x0000, 0x0000, 0xFFFC };
        const uint16_t RxSetupMask[] = { 0xF8FF, 0x0007, 0xF000, 0x001A, 0x0046, 0x003C, 0xC000, 0x8000, 0xFFFC };
        const uint16_t RxSetupWrOnlyAddr[] = { 0x0100,
            0x0101,
            0x0102,
            0x0103,
            0x0104,
            0x0105,
            0x0106,
            0x0107,
            0x0108,
            0x0109,
            0x010A,
            0x0200,
            0x0201,
            0x0202,
            0x0208,
            0x0240,
            0x0400,
            0x0401,
            0x0402,
            0x0403,
            0x0407,
            0x040A,
            0x040C,
            0x0440,
            0x05C0,
            0x05CB,
            0x0203,
            0x0204,
            0x0205,
            0x0206,
            0x0207,
            0x0241,
            0x0404,
            0x0405,
            0x0406,
            0x0408,
            0x0409,
            0x0441,
            0x05C1,
            0x05C2,
            0x05C3,
            0x05C4,
            0x05C5,
            0x05C6,
            0x05C7,
            0x05C8,
            0x05C9,
            0x05CA,
            0x05CC,
            0x0081 };
        const uint16_t RxSetupWrOnlyData[] = { 0x3408,
            0x6001,
            0x3180,
            0x0A12,
            0x0088,
            0x0007,
            0x318C,
            0x318C,
            0x0426,
            0x61C1,
            0x104C,
            0x008D,
            0x07FF,
            0x07FF,
            0x2070,
            0x0020,
            0x0081,
            0x07FF,
            0x07FF,
            0x4000,
            0x0700,
            0x1000,
            0x2098,
            0x0020,
            0x00FF,
            0x2020 };
        const RegisterBatch batch = { RxSetupAddr,
            RxSetupData,
            RxSetupMask,
            sizeof(RxSetupAddr) / sizeof(uint16_t),
            RxSetupWrOnlyAddr,
            RxSetupWrOnlyData,
            sizeof(RxSetupWrOnlyAddr) / sizeof(uint16_t),
            sizeof(RxSetupWrOnlyData) / sizeof(uint16_t) };
        lms7002m_write_masked_registers(self, &batch);
    }

    //AFE
    if ((x0020val & 0x3) == 1)
        lms7002m_spi_modify_csr(self, LMS7002M_PD_TX_AFE1, 0);
    else
        lms7002m_spi_modify_csr(self, LMS7002M_PD_TX_AFE2, 0);

    if (extLoopback) // external loopback
    {
        const uint8_t band1_band2 = 2;
        lms7002m_spi_modify(self, 0x0103, 11, 10, band1_band2);
        if (lms7002m_spi_read_csr(self, LMS7002M_SEL_PATH_RFE) != 0)
            return lime_Result_Error;
    }
    else //chip internal loopbacks
    {
        switch (lms7002m_spi_read_csr(self, LMS7002M_SEL_PATH_RFE))
        {
        case 2: //LNAL
            lms7002m_spi_modify(self, 0x0103, 11, 10, 1);
            break;
        case 3: //LNAW
        case 1: //LNAH
            lms7002m_spi_modify(self, 0x0103, 11, 10, 2);
            break;
        default:
            return lime_Result_Error;
        }
    }

    lms7002m_spi_modify_csr(self, LMS7002M_MAC, 2); //Get freq already changes/restores ch

    lime_Result status;
    if (lms7002m_spi_read_csr(self, LMS7002M_PD_LOCH_T2RBUF) == 0) //isTDD
    {
        //in TDD do nothing
        lms7002m_spi_modify_csr(self, LMS7002M_MAC, 1);
        lms7002m_set_defaults_sx(self);
        lms7002m_spi_modify_csr(self, LMS7002M_ICT_VCO, 255);
        const uint64_t sxt_lo = lms7002m_get_frequency_sx(self, true) - bandwidthRF / calibUserBwDivider - 9000000;
        status = lms7002m_set_frequency_sx(self, false, sxt_lo);
    }
    else
    {
        //SXR
        lms7002m_spi_modify_csr(self, LMS7002M_MAC, 1);
        //check if Rx is tuned
        if (!lms7002m_is_pll_tuned(self, false))
            return lime_Result_Error;
        const uint64_t SXRfreqHz = lms7002m_get_frequency_sx(self, false);

        //SXT
        lms7002m_spi_modify_csr(self, LMS7002M_MAC, 2);
        lms7002m_set_defaults_sx(self);
        lms7002m_spi_modify_csr(self, LMS7002M_ICT_VCO, 255);
        status = lms7002m_set_frequency_sx(self, true, SXRfreqHz + bandwidthRF / calibUserBwDivider + 9000000);
    }
    lms7002m_spi_write(self, 0x0020, x0020val);
    if (status != lime_Result_Success)
        return status;

    lms7002m_load_dc_reg_tx_iq(self);

    //CGEN
    status = lms7002m_setup_cgen(self);
    if (status != lime_Result_Success)
        return status;
    lms7002m_set_rx_gfir3_coefficients(self);
    lms7002m_set_nco_frequency(self, true, 0, 9000000);
    lms7002m_set_nco_frequency(self, false, 0, bandwidthRF / calibUserBwDivider - offsetNCO);
    //modifications when calibrating channel B
    lms7002m_enable_mimo_buffers_if_necessary(self);
    lms7002m_enable_channel_power_controls(self);
    return lime_Result_Success;
}

int16_t lms7002m_read_analog_dc(lms7002m_context* self, const uint16_t addr)
{
    const uint16_t mask = addr < 0x05C7 ? 0x03FF : 0x003F;
    lms7002m_spi_write(self, addr, 0);
    lms7002m_spi_write(self, addr, 0x4000);
    const uint16_t value = lms7002m_spi_read(self, addr);
    lms7002m_spi_write(self, addr, value & ~0xC000);
    int16_t result = (value & mask);
    if (value & (mask + 1))
        result *= -1;
    return result;
}

static void lms7002m_write_analog_dc(lms7002m_context* self, const uint16_t addr, int16_t value)
{
    const uint16_t mask = addr < 0x05C7 ? 0x03FF : 0x003F;
    int16_t regValue = 0;
    if (value < 0)
    {
        regValue |= (mask + 1);
        regValue |= (math_abs(value + mask) & mask);
    }
    else
        regValue |= (math_abs(value + mask + 1) & mask);
    lms7002m_spi_write(self, addr, regValue);
    lms7002m_spi_write(self, addr, regValue | 0x8000);
}

static void lms7002m_adjust_auto_dc(lms7002m_context* self, const uint16_t address, bool tx)
{
    int16_t initVal = lms7002m_read_analog_dc(self, address);
    int16_t minValue = initVal;

    uint16_t rssi = lms7002m_get_rssi(self);
    uint16_t minRSSI = rssi;

    const uint16_t range = tx ? 1023 : 63;
    lms7002m_write_analog_dc(self, address, clamp_int(initVal + 1, -range, range));

    const int8_t valChange = lms7002m_get_rssi(self) < rssi ? 1 : -1;
    for (uint8_t i = 8; i; --i)
    {
        initVal = clamp_int(initVal + valChange, -range, range);
        lms7002m_write_analog_dc(self, address, initVal);
        rssi = lms7002m_get_rssi(self);
        if (rssi < minRSSI)
        {
            minRSSI = rssi;
            minValue = initVal;
        }
    }
    lms7002m_write_analog_dc(self, address, minValue);
}

static void lms7002m_calibrate_rx_dc_auto(lms7002m_context* self)
{
    uint16_t dcRegAddr = 0x5C7;
    const uint8_t ch = lms7002m_spi_read_csr(self, LMS7002M_MAC);
    lms7002m_spi_modify_csr(self, LMS7002M_EN_G_TRF, 0);
    lms7002m_spi_modify_csr(self, LMS7002M_DC_BYP_RXTSP, 1);

    // Auto calibration
    lms7002m_spi_modify_csr(self, LMS7002M_DCMODE, 1);
    if (ch == 1)
    {
        lms7002m_spi_modify_csr(self, LMS7002M_PD_DCDAC_RXA, 0);
        lms7002m_spi_modify_csr(self, LMS7002M_PD_DCCMP_RXA, 0);
        lms7002m_spi_write(self, 0x05C2, 0xFF30);
    }
    else
    {
        lms7002m_spi_modify_csr(self, LMS7002M_PD_DCDAC_RXB, 0);
        lms7002m_spi_modify_csr(self, LMS7002M_PD_DCCMP_RXB, 0);
        lms7002m_spi_write(self, 0x05C2, 0xFFC0);
        dcRegAddr += 2;
    }

    {
        while (lms7002m_spi_read(self, 0x05C1) & 0xF000)
            ;
    }

    LMS7002M_LOG(self,
        lime_LogLevel_Debug,
        "Rx DC auto   I: %3i, Q: %3i, %s",
        lms7002m_read_analog_dc(self, dcRegAddr),
        lms7002m_read_analog_dc(self, dcRegAddr + 1),
        rssi_to_string(lms7002m_get_rssi(self)));

    // Manual adjustments
    lms7002m_spi_modify_csr(self, LMS7002M_GCORRQ_RXTSP, 0);
    lms7002m_adjust_auto_dc(self, dcRegAddr, false);
    lms7002m_spi_modify_csr(self, LMS7002M_GCORRQ_RXTSP, 2047);
    lms7002m_adjust_auto_dc(self, dcRegAddr + 1, false);

    LMS7002M_LOG(self,
        lime_LogLevel_Debug,
        "Rx DC manual I: %3i, Q: %3i, %s",
        lms7002m_read_analog_dc(self, dcRegAddr),
        lms7002m_read_analog_dc(self, dcRegAddr + 1),
        rssi_to_string(lms7002m_get_rssi(self)));

    lms7002m_spi_modify_csr(self, LMS7002M_DC_BYP_RXTSP, 0); // DC_BYP 0
    LMS7002M_LOG(self, lime_LogLevel_Debug, "RxTSP DC corrector enabled %s", rssi_to_string(lms7002m_get_rssi(self)));
    lms7002m_spi_modify_csr(self, LMS7002M_EN_G_TRF, 1);
}

static lime_Result lms7002m_check_saturation_rx(lms7002m_context* self, const uint32_t bandwidth_Hz, bool extLoopback)
{
    uint32_t target_rssi = 28492; //dbfs_to_chip_rssi(-10.0);
    uint8_t cg_iamp = (uint8_t)lms7002m_spi_read_csr(self, LMS7002M_CG_IAMP_TBB);

    lms7002m_spi_modify_csr(self, LMS7002M_CMIX_SC_RXTSP, 1);
    lms7002m_spi_modify_csr(self, LMS7002M_CMIX_BYP_RXTSP, 0);
    lms7002m_set_nco_frequency(self, false, 0, bandwidth_Hz / calibUserBwDivider - offsetNCO);

    uint32_t rssi = 0;
    if (extLoopback)
    {
        int8_t g_lossmain = 15;
        lms7002m_spi_modify_csr(self, LMS7002M_LOSS_MAIN_TXPAD_TRF, g_lossmain);
        rssi = lms7002m_get_rssi(self);
        LMS7002M_LOG(self,
            lime_LogLevel_Debug,
            "Initial gains:\tLOSS_MAIN_TXPAD: %2i, CG_IAMP: %2i | %s",
            g_lossmain,
            cg_iamp,
            rssi_to_string(rssi));
        while (rssi < target_rssi)
        {
            g_lossmain -= 1;
            if (g_lossmain < 0)
                break;
            lms7002m_spi_modify_csr(self, LMS7002M_LOSS_MAIN_TXPAD_TRF, g_lossmain);
            rssi = lms7002m_get_rssi(self);
        }
    }
    else
    {
        uint8_t g_rxloopb_rfe = 2;
        lms7002m_spi_modify_csr(self, LMS7002M_G_RXLOOPB_RFE, g_rxloopb_rfe);
        rssi = lms7002m_get_rssi(self);
        LMS7002M_LOG(self,
            lime_LogLevel_Debug,
            "Initial gains:\tG_RXLOOPB: %2i, CG_IAMP: %2i | %s",
            g_rxloopb_rfe,
            cg_iamp,
            rssi_to_string(rssi));
        while (rssi < target_rssi)
        {
            g_rxloopb_rfe += 2;
            if (g_rxloopb_rfe > 15)
                break;
            lms7002m_spi_modify_csr(self, LMS7002M_G_RXLOOPB_RFE, g_rxloopb_rfe);
            rssi = lms7002m_get_rssi(self);
        }
    }

    target_rssi = 4024; //dbfs_to_chip_rssi(-27);
    while (rssi < target_rssi)
    {
        cg_iamp += 2;
        if (cg_iamp > 63 - 6)
            break;
        lms7002m_spi_modify_csr(self, LMS7002M_CG_IAMP_TBB, cg_iamp);
        rssi = lms7002m_get_rssi(self);
    }

    while (rssi < target_rssi)
    {
        cg_iamp += 1;
        if (cg_iamp > 62)
            break;
        lms7002m_spi_modify_csr(self, LMS7002M_CG_IAMP_TBB, cg_iamp);
        rssi = lms7002m_get_rssi(self);
    }

    if (extLoopback)
    {
        LMS7002M_LOG(self,
            lime_LogLevel_Debug,
            "Adjusted gains:\tLOSS_MAIN_TXPAD: %2i, CG_IAMP: %2i | %s",
            lms7002m_spi_read_csr(self, LMS7002M_LOSS_MAIN_TXPAD_TRF),
            lms7002m_spi_read_csr(self, LMS7002M_CG_IAMP_TBB),
            rssi_to_string(rssi));
    }
    else
    {
        LMS7002M_LOG(self,
            lime_LogLevel_Debug,
            "Adjusted gains: G_RXLOOPB: %2i, CG_IAMP: %2i | %s",
            lms7002m_spi_read_csr(self, LMS7002M_G_RXLOOPB_RFE),
            lms7002m_spi_read_csr(self, LMS7002M_CG_IAMP_TBB),
            rssi_to_string(rssi));
    }

    const int32_t expectedRSSI_level = 2849; // dbfs_to_chip_rssi(-30);
    const int32_t failureRSSI_level = 285; // dbfs_to_chip_rssi(-50)
    if (rssi < expectedRSSI_level)
    {
        char expectedRSSI_string[32];
        strcpy(expectedRSSI_string, rssi_to_string(expectedRSSI_level));
        LMS7002M_LOG(self,
            rssi > failureRSSI_level ? lime_LogLevel_Warning : lime_LogLevel_Error,
            "Low calibration test signal level %s, expected to be more than %s."
            " Calibration results might be impacted. Try re-calibrating or adjusting the RX gains.",
            rssi_to_string(rssi),
            expectedRSSI_string);
    }
    return rssi > failureRSSI_level ? lime_Result_Success : lime_Result_Error;
}

/** @brief Binary search information */
typedef struct {
    struct lms7002m_csr param; ///< The address and the value of where to search
    int16_t result; ///< The result of the search
    int16_t minValue; ///< Minumum value of the search
    int16_t maxValue; ///< Maximum value of the search
} BinSearchParam;

static void lms7002m_binary_search(lms7002m_context* self, BinSearchParam* args)
{
    int16_t left = args->minValue;
    int16_t right = args->maxValue;

    const uint16_t addr = args->param.address;
    const uint8_t msb = args->param.msb;
    const uint8_t lsb = args->param.lsb;
    lms7002m_spi_modify(self, addr, msb, lsb, right);

    uint16_t rssiLeft = ~0;
    uint16_t rssiRight = lms7002m_get_rssi(self);
    while (right - left >= 1)
    {
        if (rssiLeft < rssiRight)
        {
            lms7002m_spi_modify(self, addr, msb, lsb, right);
            rssiRight = lms7002m_get_rssi(self);
        }
        else
        {
            lms7002m_spi_modify(self, addr, msb, lsb, left);
            rssiLeft = lms7002m_get_rssi(self);
        }

        const int16_t step = (right - left) / 2;
        if (step <= 0)
            break;
        if (rssiLeft < rssiRight)
            right -= step;
        else
            left += step;
    }
    args->result = rssiLeft < rssiRight ? left : right;
    lms7002m_spi_modify(self, addr, msb, lsb, args->result);
}

static void lms7002m_calibrate_iq_imbalance(lms7002m_context* self, bool isTx)
{
    const char* const dirName = isTx ? "Tx" : "Rx";

    uint16_t gcorriAddress;
    uint16_t gcorrqAddress;
    BinSearchParam argsPhase;
    BinSearchParam argsGain;
    argsGain.param.msb = 10;
    argsGain.param.lsb = 0;
    argsPhase.param.msb = 11;
    argsPhase.param.lsb = 0;
    if (isTx)
    {
        gcorrqAddress = 0x0201;
        gcorriAddress = 0x0202;
        argsPhase.param.address = 0x0203;
    }
    else
    {
        gcorrqAddress = 0x0401;
        gcorriAddress = 0x0402;
        argsPhase.param.address = 0x0403;
    }

    argsPhase.maxValue = 128;
    argsPhase.minValue = -128;
    lms7002m_binary_search(self, &argsPhase);
    LMS7002M_LOG(
        self, lime_LogLevel_Debug, "#0 %s IQCORR: %i, %s", dirName, argsPhase.result, rssi_to_string(lms7002m_get_rssi(self)));

    //coarse gain
    {
        lms7002m_spi_write(self, gcorriAddress, 2047 - 64);
        lms7002m_spi_write(self, gcorrqAddress, 2047);
        const uint16_t rssiIgain = lms7002m_get_rssi(self);
        lms7002m_spi_write(self, gcorriAddress, 2047);
        lms7002m_spi_write(self, gcorrqAddress, 2047 - 64);
        const uint16_t rssiQgain = lms7002m_get_rssi(self);

        if (rssiIgain < rssiQgain)
            argsGain.param.address = gcorriAddress;
        else
            argsGain.param.address = gcorrqAddress;
        lms7002m_spi_write(self, gcorrqAddress, 2047);
    }
    argsGain.maxValue = 2047;
    argsGain.minValue = 2047 - 512;
    lms7002m_binary_search(self, &argsGain);

    LMS7002M_LOG(self,
        lime_LogLevel_Debug,
        "#1 %s GAIN_%s: %i, %s",
        dirName,
        (argsGain.param.address == gcorriAddress ? "I" : "Q"),
        argsGain.result,
        rssi_to_string(lms7002m_get_rssi(self)));

    argsPhase.maxValue = argsPhase.result + 16;
    argsPhase.minValue = argsPhase.result - 16;
    lms7002m_binary_search(self, &argsPhase);

    LMS7002M_LOG(
        self, lime_LogLevel_Debug, "#2 %s IQCORR: %i, %s", dirName, argsPhase.result, rssi_to_string(lms7002m_get_rssi(self)));

    lms7002m_spi_write(self, argsGain.param.address, argsGain.result);
    lms7002m_spi_modify(self, argsPhase.param.address, argsPhase.param.msb, argsPhase.param.lsb, argsPhase.result);
}

lime_Result lms7002m_calibrate_rx(lms7002m_context* self, uint32_t bandwidthRF, bool extLoopback, bool dcOnly)
{
    if (bandwidthRF < offsetNCO * calibUserBwDivider)
        bandwidthRF = offsetNCO * calibUserBwDivider;

    const uint16_t x0020val = lms7002m_spi_read(self, 0x0020); //remember used channel

    LMS7002M_LOG(self,
        lime_LogLevel_Debug,
        "Rx calibrate ch.%s @ %lu Hz, BW: %u Hz, RF input: %s, PGA: %i, LNA: %i, TIA: %i",
        (x0020val & 0x3) == 1 ? "A" : "B",
        lms7002m_get_frequency_sx(self, false),
        bandwidthRF,
        get_lna_name(lms7002m_spi_read_csr(self, LMS7002M_SEL_PATH_RFE)),
        lms7002m_spi_read_csr(self, LMS7002M_G_PGA_RBB),
        lms7002m_spi_read_csr(self, LMS7002M_G_LNA_RFE),
        lms7002m_spi_read_csr(self, LMS7002M_G_TIA_RFE));
    lms7002m_save_chip_state(self, false);

    lime_Result status = lms7002m_calibrate_rx_setup(self, bandwidthRF, extLoopback);
    if (status != 0)
        goto RxCalibrationEndStage;
    lms7002m_calibrate_rx_dc_auto(self);
    if (dcOnly)
        goto RxCalibrationEndStage;
    if (!extLoopback)
    {
        if ((uint8_t)lms7002m_spi_read_csr(self, LMS7002M_SEL_PATH_RFE) == 2)
        {
            lms7002m_spi_modify_csr(self, LMS7002M_PD_RLOOPB_2_RFE, 0);
            lms7002m_spi_modify_csr(self, LMS7002M_EN_INSHSW_LB2_RFE, 0);
        }
        else
        {
            lms7002m_spi_modify_csr(self, LMS7002M_PD_RLOOPB_1_RFE, 0);
            lms7002m_spi_modify_csr(self, LMS7002M_EN_INSHSW_LB1_RFE, 0);
        }
    }

    const uint8_t mac = x0020val & 0x0003;
    lms7002m_spi_modify_csr(self, LMS7002M_MAC, 2);
    if (lms7002m_spi_read_csr(self, LMS7002M_PD_LOCH_T2RBUF) == false)
    {
        lms7002m_spi_modify_csr(self, LMS7002M_PD_LOCH_T2RBUF, 1);
        //TDD MODE
        lms7002m_spi_modify_csr(self, LMS7002M_MAC, 1);
        lms7002m_spi_modify_csr(self, LMS7002M_PD_VCO, 0);
    }
    lms7002m_spi_modify_csr(self, LMS7002M_MAC, mac);
    status = lms7002m_check_saturation_rx(self, bandwidthRF, extLoopback);
    if (status != lime_Result_Success)
        goto RxCalibrationEndStage;
    lms7002m_spi_modify_csr(self, LMS7002M_CMIX_SC_RXTSP, 0);
    lms7002m_spi_modify_csr(self, LMS7002M_CMIX_BYP_RXTSP, 0);
    lms7002m_set_nco_frequency(self, false, 0, bandwidthRF / calibUserBwDivider + offsetNCO);
    lms7002m_calibrate_iq_imbalance(self, false);
RxCalibrationEndStage : {
    const uint16_t gcorri = lms7002m_spi_read_csr(self, LMS7002M_GCORRI_RXTSP);
    const uint16_t gcorrq = lms7002m_spi_read_csr(self, LMS7002M_GCORRQ_RXTSP);
    const uint16_t phaseOffset = lms7002m_spi_read_csr(self, LMS7002M_IQCORR_RXTSP);
    lms7002m_save_chip_state(self, true);
    lms7002m_spi_write(self, 0x0020, x0020val);
    if (status != lime_Result_Success)
    {
        LMS7002M_LOG(self, lime_LogLevel_Debug, "%s", "Rx calibration failed");
        return status;
    }
    // dc corrector values not overwritten by chip state restore
    if (!dcOnly)
    {
        lms7002m_spi_modify_csr(self, LMS7002M_GCORRI_RXTSP, gcorri);
        lms7002m_spi_modify_csr(self, LMS7002M_GCORRQ_RXTSP, gcorrq);
        lms7002m_spi_modify_csr(self, LMS7002M_IQCORR_RXTSP, phaseOffset);
    }
    LMS7002M_LOG(self, lime_LogLevel_Debug, "%s", "Rx | DC   | GAIN | PHASE");
    LMS7002M_LOG(self, lime_LogLevel_Debug, "%s", "---+------+------+------");
    LMS7002M_LOG(self,
        lime_LogLevel_Debug,
        "I: | %4i | %4i | %i",
        lms7002m_read_analog_dc(self, (x0020val & 1) ? 0x5C7 : 0x5C8),
        gcorri,
        to_signed(phaseOffset, 11));
    LMS7002M_LOG(
        self, lime_LogLevel_Debug, "Q: | %4i | %4i |", lms7002m_read_analog_dc(self, (x0020val & 1) ? 0x5C9 : 0x5CA), gcorrq);
}
    lms7002m_spi_modify_csr(self, LMS7002M_DCMODE, 1);
    if (x0020val & 0x1)
        lms7002m_spi_modify_csr(self, LMS7002M_PD_DCDAC_RXA, 0);
    else
        lms7002m_spi_modify_csr(self, LMS7002M_PD_DCDAC_RXB, 0);
    lms7002m_spi_modify(self, 0x040C, 2, 0, 0); //DC_BYP 0, GC_BYP 0, PH_BYP 0
    lms7002m_spi_modify(self, 0x040C, 8, 8, 0); //DCLOOP_STOP
    LMS7002M_LOG(self, lime_LogLevel_Info, "%s", "Rx calibration finished");
    return lime_Result_Success;
}

static lime_Result lms7002m_calibrate_tx_setup(lms7002m_context* self, uint32_t bandwidthRF, bool extLoopback)
{
    const uint16_t x0020val = lms7002m_spi_read(self, 0x0020); //remember used channel

    if ((x0020val & 0x3) == 1)
        lms7002m_spi_modify_csr(self, LMS7002M_PD_RX_AFE1, 0);
    else
        lms7002m_spi_modify_csr(self, LMS7002M_PD_RX_AFE2, 0);
    {
        const uint16_t TxSetupAddr[] = { 0x0084, 0x0085, 0x00AE, 0x0101, 0x0200, 0x0201, 0x0202, 0x0208, 0x0020 };
        const uint16_t TxSetupData[] = { 0x0400, 0x0001, 0xF000, 0x0001, 0x000C, 0x07FF, 0x07FF, 0x0000, 0xFFFC };
        const uint16_t TxSetupMask[] = { 0xF8FF, 0x0007, 0xF000, 0x1801, 0x000C, 0x07FF, 0x07FF, 0xF10B, 0xFFFC };
        const uint16_t TxSetupWrOnlyAddr[] = { 0x010C,
            0x010D,
            0x010E,
            0x010F,
            0x0110,
            0x0111,
            0x0112,
            0x0113,
            0x0114,
            0x0115,
            0x0116,
            0x0117,
            0x0118,
            0x0119,
            0x011A,
            0x0400,
            0x0401,
            0x0402,
            0x0403,
            0x0407,
            0x040A,
            0x040C,
            0x0440,
            0x0441,
            0x0442,
            0x0443,
            0x0409,
            0x0408,
            0x0406,
            0x0405,
            0x0404,
            0x0081 };
        const uint16_t TxSetupWrOnlyData[] = { 0x88E5,
            0x009E,
            0x2040,
            0x30C6,
            0x0994,
            0x0083,
            0x4032,
            0x03DF,
            0x008D,
            0x0005,
            0x8180,
            0x280C,
            0x218C,
            0x3180,
            0x2E02,
            0x0081,
            0x07FF,
            0x07FF,
            0x4000,
            0x0700,
            0x1001,
            0x2098 }; // rest of values will be written as zeros
        const RegisterBatch batch = { TxSetupAddr,
            TxSetupData,
            TxSetupMask,
            sizeof(TxSetupAddr) / sizeof(uint16_t),
            TxSetupWrOnlyAddr,
            TxSetupWrOnlyData,
            sizeof(TxSetupWrOnlyAddr) / sizeof(uint16_t),
            sizeof(TxSetupWrOnlyData) / sizeof(uint16_t) };
        lms7002m_write_masked_registers(self, &batch);
    }
    lms7002m_set_rx_gfir3_coefficients(self);
    lime_Result status = lms7002m_setup_cgen(self);
    if (status != lime_Result_Success)
        return status;
    //SXR
    const uint8_t mac = x0020val & 0x0003;
    lms7002m_spi_modify_csr(self, LMS7002M_MAC, 1); //switch to ch. A
    lms7002m_set_defaults_sx(self);
    lms7002m_spi_modify_csr(self, LMS7002M_ICT_VCO, 255);
    {
        const uint64_t SXRfreq = lms7002m_get_frequency_sx(self, true) - bandwidthRF / calibUserBwDivider - calibrationSXOffset_Hz;
        //SX VCO is powered up in SetFrequencySX/Tune
        status = lms7002m_set_frequency_sx(self, false, SXRfreq);
        if (status != lime_Result_Success)
        {
            lms7002m_spi_read(self, x0020val); //restore used channel
            return status;
        }
    }

    //SXT{
    lms7002m_spi_modify_csr(self, LMS7002M_MAC, 2); //switch to ch. B
    lms7002m_spi_modify_csr(self, LMS7002M_PD_LOCH_T2RBUF, 1);
    //check if Tx is tuned
    if (!lms7002m_is_pll_tuned(self, true))
    {
        lms7002m_spi_write(self, 0x0020, x0020val); //restore used channel
        return lime_Result_Error;
    }

    lms7002m_spi_modify_csr(self, LMS7002M_MAC, mac); //restore used channel

    lms7002m_load_dc_reg_tx_iq(self);
    lms7002m_set_nco_frequency(self, true, 0, bandwidthRF / calibUserBwDivider);

    {
        const uint8_t sel_band1_2_trf = (uint8_t)lms7002m_spi_read_bits(self, 0x0103, 11, 10);
        if (extLoopback)
        {
            lms7002m_spi_modify_csr(self, LMS7002M_PD_LNA_RFE, 0);
            if (sel_band1_2_trf == 1 || sel_band1_2_trf == 2)
            {
                lms7002m_spi_modify_csr(self, LMS7002M_SEL_PATH_RFE, 0);
                lms7002m_spi_modify(self, 0x010D, 2, 1, ~(0 - 1)); //EN_INSHSW_*_RFE

                //check if correct tx band for external loop
                if (0 != !(sel_band1_2_trf - 1))
                    return lime_Result_Error;
            }
            else
            {
                LMS7002M_LOG(
                    self, lime_LogLevel_Error, "%s", "Tx Calibration: external calibration is not supported on selected Tx Band");
                return lime_Result_Error;
            }
        }
        else
        {
            if (sel_band1_2_trf != 0x1 && sel_band1_2_trf != 0x2) //BAND1
            {
                LMS7002M_LOG(self, lime_LogLevel_Error, "%s", "Tx Calibration: band not selected");
                return lime_Result_Error;
            }
            lms7002m_spi_modify_csr(self, LMS7002M_SEL_PATH_RFE, sel_band1_2_trf + 1);
            lms7002m_spi_modify(self, 0x010C, 6, 5, sel_band1_2_trf ^ 0x3);
            lms7002m_spi_modify(self, 0x010D, 4, 3, sel_band1_2_trf ^ 0x3);
        }
    }
    //if calibrating ch. B enable buffers
    lms7002m_enable_mimo_buffers_if_necessary(self);
    lms7002m_enable_channel_power_controls(self);

    return lime_Result_Success;
}

static lime_Result lms7002m_check_saturation_tx_rx(lms7002m_context* self, uint32_t bandwidthRF, bool extLoopback)
{
    const uint16_t saturationLevel = 20498; //dbfs_to_chip_rssi(-12.86);

    lms7002m_spi_modify_csr(self, LMS7002M_DC_BYP_RXTSP, 0);
    lms7002m_spi_modify_csr(self, LMS7002M_CMIX_BYP_RXTSP, 0);

    lms7002m_set_nco_frequency(self, false, 0, calibrationSXOffset_Hz - offsetNCO + (bandwidthRF / calibUserBwDivider) * 2);

    uint8_t g_pga = (uint8_t)lms7002m_spi_read_csr(self, LMS7002M_G_PGA_RBB);
    uint8_t g_rfe = 0;
    if (extLoopback)
    {
        lms7002m_spi_modify_csr(self, LMS7002M_G_LNA_RFE, g_rfe);
    }
    else
    {
        g_rfe = (uint8_t)lms7002m_spi_read_csr(self, LMS7002M_G_RXLOOPB_RFE);
    }

    uint32_t rssi = lms7002m_get_rssi(self);
    LMS7002M_LOG(self,
        lime_LogLevel_Debug,
        "Receiver saturation search, target level: %i (%s)",
        saturationLevel,
        rssi_to_string(saturationLevel));
    LMS7002M_LOG(self,
        lime_LogLevel_Debug,
        "initial  PGA: %2i, %s: %2i, %s",
        g_pga,
        (extLoopback ? "LNA" : "RXLOOPB"),
        g_rfe,
        rssi_to_string(rssi));
    while (rssi < saturationLevel)
    {
        if (g_rfe < 15)
            ++g_rfe;
        else
            break;

        if (extLoopback)
            lms7002m_spi_modify_csr(self, LMS7002M_G_LNA_RFE, g_rfe);
        else
            lms7002m_spi_modify_csr(self, LMS7002M_G_RXLOOPB_RFE, g_rfe);

        rssi = lms7002m_get_rssi(self);
    }

    {
        uint16_t rssi_prev = rssi;
        while (g_pga < 25 && g_rfe == 15 && rssi < saturationLevel)
        {
            if (g_pga < 25)
                ++g_pga;
            else
                break;

            lms7002m_spi_modify_csr(self, LMS7002M_G_PGA_RBB, g_pga);
            rssi = lms7002m_get_rssi(self);
            //if ((float)rssi / rssi_prev < 1.05) // pga should give ~1dB change
            if ((rssi * 100) / rssi_prev < 105) // pga should give ~1dB change
                break;

            rssi_prev = rssi;
        }
    }
    LMS7002M_LOG(self,
        lime_LogLevel_Debug,
        "adjusted PGA: %2i, %s: %2i, %s",
        lms7002m_spi_read_csr(self, LMS7002M_G_PGA_RBB),
        (extLoopback ? "LNA" : "RXLOOPB"),
        g_rfe,
        rssi_to_string(rssi));

    const uint32_t expectedRSSI_level = 2849; // dbfs_to_chip_rssi(-30.0);
    const int32_t failureRSSI_level = 285; // dbfs_to_chip_rssi(-50)
    if (rssi < expectedRSSI_level)
    {
        char expectedRSSI_string[32];
        strcpy(expectedRSSI_string, rssi_to_string(expectedRSSI_level));
        LMS7002M_LOG(self,
            rssi > failureRSSI_level ? lime_LogLevel_Warning : lime_LogLevel_Error,
            "Low calibration test signal level %s, expected to be more than %s."
            " Calibration results might be impacted. Try re-calibrating or adjusting the TX gains.",
            rssi_to_string(rssi),
            expectedRSSI_string);
        if (rssi < failureRSSI_level)
            return lime_Result_Error;
    }
    lms7002m_spi_modify_csr(self, LMS7002M_CMIX_BYP_RXTSP, 1);
    lms7002m_spi_modify_csr(self, LMS7002M_DC_BYP_RXTSP, 1);
    return lime_Result_Success;
}

static void lms7002m_tx_dc_binary_search(lms7002m_context* self, BinSearchParam* args)
{
    int16_t left = args->minValue;
    int16_t right = args->maxValue;
    lms7002m_write_analog_dc(self, args->param.address, right);

    uint16_t rssiLeft = ~0;
    uint16_t rssiRight = lms7002m_get_rssi(self);

    while (right - left >= 1)
    {
        if (rssiLeft < rssiRight)
        {
            lms7002m_write_analog_dc(self, args->param.address, right);
            rssiRight = lms7002m_get_rssi(self);
        }
        else
        {
            lms7002m_write_analog_dc(self, args->param.address, left);
            rssiLeft = lms7002m_get_rssi(self);
        }

        const int16_t step = (right - left) / 2;
        if (step == 0)
            break;
        if (rssiLeft < rssiRight)
            right -= step;
        else
            left += step;
    }

    args->result = rssiLeft < rssiRight ? left : right;
    lms7002m_write_analog_dc(self, args->param.address, args->result);
}

static void lms7002m_calibrate_tx_dc_auto(lms7002m_context* self)
{
    const uint8_t ch = lms7002m_spi_read_csr(self, LMS7002M_MAC);
    lms7002m_spi_modify_csr(self, LMS7002M_EN_G_TRF, 1);
    lms7002m_spi_modify_csr(self, LMS7002M_CMIX_BYP_TXTSP, 0);
    lms7002m_spi_modify_csr(self, LMS7002M_CMIX_BYP_RXTSP, 0);

    lms7002m_spi_modify_csr(self, LMS7002M_DC_BYP_TXTSP, 1);
    //auto calibration
    lms7002m_spi_modify_csr(self, LMS7002M_DCMODE, 1);

    BinSearchParam iparams;
    iparams.param.msb = 10;
    iparams.param.lsb = 0;

    BinSearchParam qparams;
    qparams.param.msb = 10;
    qparams.param.lsb = 0;

    if (ch == 1)
    {
        iparams.param.address = 0x5C3; // DC_TXAI;
        qparams.param.address = 0x5C4; // DC_TXAQ;
        lms7002m_spi_modify_csr(self, LMS7002M_PD_DCDAC_TXA, 0);
        lms7002m_spi_modify_csr(self, LMS7002M_PD_DCCMP_TXA, 0);
    }
    else
    {
        iparams.param.address = 0x5C5; // DC_TXBI;
        qparams.param.address = 0x5C6; // DC_TXBQ;
        lms7002m_spi_modify_csr(self, LMS7002M_PD_DCDAC_TXB, 0);
        lms7002m_spi_modify_csr(self, LMS7002M_PD_DCCMP_TXB, 0);
    }
    lms7002m_write_analog_dc(self, iparams.param.address, 0);
    lms7002m_write_analog_dc(self, qparams.param.address, 0);

    const int16_t offset[3] = { 1023, 128, 8 };
    uint8_t i;
    iparams.result = 0; //ReadAnalogDC(iparams.param.address);
    qparams.result = 0; //ReadAnalogDC(qparams.param.address);
    for (i = 0; i < 3; ++i)
    {
        iparams.minValue = clamp_int(iparams.result - offset[i], -1024, 1023);
        iparams.maxValue = clamp_int(iparams.result + offset[i], -1024, 1023);
        qparams.minValue = clamp_int(qparams.result - offset[i], -1024, 1023);
        qparams.maxValue = clamp_int(qparams.result + offset[i], -1024, 1023);

        lms7002m_tx_dc_binary_search(self, &iparams);
        lms7002m_tx_dc_binary_search(self, &qparams);
        LMS7002M_LOG(self,
            lime_LogLevel_Debug,
            "#%i Tx DC manual I: %4i, Q: %4i, %s",
            i,
            lms7002m_read_analog_dc(self, iparams.param.address),
            lms7002m_read_analog_dc(self, qparams.param.address),
            rssi_to_string(lms7002m_get_rssi(self)));
    }
}

lime_Result lms7002m_calibrate_tx(lms7002m_context* self, uint32_t bandwidthRF, bool extLoopback)
{
    const uint16_t x0020val = lms7002m_spi_read(self, 0x0020);
    LMS7002M_LOG(self,
        lime_LogLevel_Debug,
        "Tx ch.%s , BW: %u Hz, RF output: %s, Gain: %i, loopb: %s",
        (x0020val & 3) == 0x1 ? "A" : "B",
        bandwidthRF,
        lms7002m_spi_read_csr(self, LMS7002M_SEL_BAND1_TRF) == 1 ? "BAND1" : "BAND2",
        lms7002m_spi_read_csr(self, LMS7002M_CG_IAMP_TBB),
        extLoopback ? "external" : "internal");

    lms7002m_save_chip_state(self, false);
    lime_Result status = lms7002m_calibrate_tx_setup(self, bandwidthRF, extLoopback);
    if (status != lime_Result_Success)
        goto TxCalibrationEnd; //go to ending stage to restore registers
    lms7002m_calibrate_rx_dc_auto(self);
    status = lms7002m_check_saturation_tx_rx(self, bandwidthRF, extLoopback);
    if (status != lime_Result_Success)
        goto TxCalibrationEnd;
    lms7002m_calibrate_rx_dc_auto(self);

    lms7002m_set_nco_frequency(self, false, 0, calibrationSXOffset_Hz - offsetNCO + (bandwidthRF / calibUserBwDivider));
    lms7002m_calibrate_tx_dc_auto(self);
    lms7002m_set_nco_frequency(self, false, 0, calibrationSXOffset_Hz - offsetNCO);
    lms7002m_calibrate_iq_imbalance(self, true);
TxCalibrationEnd : {
    //analog dc is not overwritten by chip state restore
    const uint16_t gcorri = lms7002m_spi_read_csr(self, LMS7002M_GCORRI_TXTSP);
    const uint16_t gcorrq = lms7002m_spi_read_csr(self, LMS7002M_GCORRQ_TXTSP);
    const uint16_t phaseOffset = lms7002m_spi_read_csr(self, LMS7002M_IQCORR_TXTSP);
    lms7002m_save_chip_state(self, true);
    lms7002m_spi_write(self, 0x0020, x0020val);
    if (status != lime_Result_Success)
    {
        LMS7002M_LOG(self, lime_LogLevel_Debug, "%s", "Tx calibration failed");
        return status;
    }
    lms7002m_spi_modify_csr(self, LMS7002M_GCORRI_TXTSP, gcorri);
    lms7002m_spi_modify_csr(self, LMS7002M_GCORRQ_TXTSP, gcorrq);
    lms7002m_spi_modify_csr(self, LMS7002M_IQCORR_TXTSP, phaseOffset);

    LMS7002M_LOG(self, lime_LogLevel_Debug, "%s", "Tx | DC   | GAIN | PHASE");
    LMS7002M_LOG(self, lime_LogLevel_Debug, "%s", "---+------+------+------");
    LMS7002M_LOG(self,
        lime_LogLevel_Debug,
        "I: | %4i | %4i | %i",
        lms7002m_read_analog_dc(self, (x0020val & 1) ? 0x5C3 : 0x5C5),
        gcorri,
        to_signed(phaseOffset, 11));
    LMS7002M_LOG(
        self, lime_LogLevel_Debug, "Q: | %4i | %4i |", lms7002m_read_analog_dc(self, (x0020val & 1) ? 0x5C4 : 0x5C6), gcorrq);
}

    lms7002m_spi_modify_csr(self, LMS7002M_DCMODE, 1);
    if ((x0020val & 1) == 1)
        lms7002m_spi_modify_csr(self, LMS7002M_PD_DCDAC_TXA, 0);
    else
        lms7002m_spi_modify_csr(self, LMS7002M_PD_DCDAC_TXB, 0);
    lms7002m_spi_modify_csr(self, LMS7002M_DC_BYP_TXTSP, 1);
    lms7002m_spi_modify(self, 0x0208, 1, 0, 0); //GC_BYP PH_BYP
    return 0;
}

lime_Result lms7002m_calibrate_internal_adc(lms7002m_context* self, int clkDiv)
{
    if (lms7002m_spi_read_csr(self, LMS7002M_MASK) == 0)
        return lime_Result_NotSupported;

    const uint16_t biasMux = lms7002m_spi_read_csr(self, LMS7002M_MUX_BIAS_OUT);
    lms7002m_spi_modify_csr(self, LMS7002M_MUX_BIAS_OUT, 1);

    lms7002m_spi_write(self, 0x0600, 0x0F01);
    lms7002m_spi_write(self, 0x0602, 0x2000);
    lms7002m_spi_write(self, 0x0603, 0x0000);
    lms7002m_spi_modify_csr(self, LMS7002M_RSSI_PD, 0);
    lms7002m_spi_modify_csr(self, LMS7002M_RSSI_RSSIMODE, 1);
    lms7002m_spi_modify_csr(self, LMS7002M_DAC_CLKDIV, clkDiv);
    lms7002m_spi_modify_csr(self, LMS7002M_RSSI_BIAS, 8);
    lms7002m_spi_modify_csr(self, LMS7002M_RSSI_DAC_VAL, 170);

    uint8_t bias = lms7002m_spi_read_csr(self, LMS7002M_RSSI_BIAS);
    uint16_t regValue = lms7002m_spi_read(self, 0x0601);
    while (((regValue >> 5) & 0x1) != 1)
    {
        if (bias > 31)
        {
            LMS7002M_LOG(self, lime_LogLevel_Error, "%s", "Temperature internal ADC calibration failed");
            return lime_Result_Error;
        }
        ++bias;
        lms7002m_spi_modify_csr(self, LMS7002M_RSSI_BIAS, bias);
        regValue = lms7002m_spi_read(self, 0x0601);
    }
    lms7002m_spi_modify_csr(self, LMS7002M_RSSI_PD, 0);
    lms7002m_spi_modify_csr(self, LMS7002M_MUX_BIAS_OUT, biasMux);
    lms7002m_spi_modify_csr(self, LMS7002M_RSSI_RSSIMODE, 0);
    return lime_Result_Success;
}

lime_Result lms7002m_calibrate_rp_bias(lms7002m_context* self)
{
    if (lms7002m_spi_read_csr(self, LMS7002M_MASK) == 0)
        return lime_Result_NotSupported;

    lms7002m_calibrate_internal_adc(self, 32);
    lms7002m_spi_modify_csr(self, LMS7002M_RSSI_PD, 0);
    lms7002m_spi_modify_csr(self, LMS7002M_RSSI_RSSIMODE, 0);

    const uint16_t biasMux = lms7002m_spi_read_csr(self, LMS7002M_MUX_BIAS_OUT);
    lms7002m_spi_modify_csr(self, LMS7002M_MUX_BIAS_OUT, 1);

    lms7002m_sleep(250);

    uint16_t reg606 = lms7002m_spi_read(self, 0x0606);
    uint16_t Vref = (reg606 >> 8) & 0xFF;
    uint16_t Vptat = reg606 & 0xFF;

    if (Vref > Vptat)
    {
        uint16_t rpCalib = lms7002m_spi_read_csr(self, LMS7002M_RP_CALIB_BIAS);
        while (Vref > Vptat)
        {
            --rpCalib;
            lms7002m_spi_modify_csr(self, LMS7002M_RP_CALIB_BIAS, rpCalib);
            reg606 = lms7002m_spi_read(self, 0x0606);
            Vref = (reg606 >> 8) & 0xFF;
            Vptat = reg606 & 0xFF;
        }
    }
    if (Vref < Vptat)
    {
        uint16_t rpCalib = lms7002m_spi_read_csr(self, LMS7002M_RP_CALIB_BIAS);
        while (Vref < Vptat)
        {
            ++rpCalib;
            lms7002m_spi_modify_csr(self, LMS7002M_RP_CALIB_BIAS, rpCalib);
            reg606 = lms7002m_spi_read(self, 0x0606);
            Vref = (reg606 >> 8) & 0xFF;
            Vptat = reg606 & 0xFF;
        }
    }
    lms7002m_spi_modify_csr(self, LMS7002M_MUX_BIAS_OUT, biasMux);
    return lime_Result_Success;
}

lime_Result lms7002m_calibrate_analog_rssi_dc_offset(lms7002m_context* self)
{
    lms7002m_spi_modify_csr(self, LMS7002M_EN_INSHSW_W_RFE, 1);
    lms7002m_calibrate_internal_adc(self, 0);
    lms7002m_spi_modify_csr(self, LMS7002M_PD_RSSI_RFE, 0);
    lms7002m_spi_modify_csr(self, LMS7002M_PD_TIA_RFE, 0);

    lms7002m_spi_write(self, 0x0640, 22 << 4);

    lms7002m_spi_modify_csr(self, LMS7002M_RSSIDC_DCO2, 0);

    int value = -63;
    uint8_t wrValue = math_abs(value);
    if (value < 0)
        wrValue |= 0x40;
    lms7002m_spi_modify_csr(self, LMS7002M_RSSIDC_DCO1, wrValue);
    uint8_t cmp = lms7002m_spi_read_csr(self, LMS7002M_RSSIDC_CMPSTATUS);
    uint8_t cmpPrev = cmp;
    int8_t edges[2];
    uint8_t edgesIndex = 0;
    for (value = -63; value < 64; ++value)
    {
        wrValue = math_abs(value);
        if (value < 0)
            wrValue |= 0x40;
        lms7002m_spi_modify_csr(self, LMS7002M_RSSIDC_DCO1, wrValue);
        lms7002m_sleep(5);
        cmp = lms7002m_spi_read_csr(self, LMS7002M_RSSIDC_CMPSTATUS);
        if (cmp != cmpPrev)
        {
            edges[edgesIndex++] = value;
            cmpPrev = cmp;

            if (edgesIndex > 1)
            {
                break;
            }
        }
    }
    if (edgesIndex != 2)
    {
        LMS7002M_LOG(self, lime_LogLevel_Debug, "%s: Failed to find value.", __func__);
        return lime_Result_Error;
    }

    const int8_t found = (edges[0] + edges[1]) / 2;
    wrValue = math_abs(found);
    if (found < 0)
        wrValue |= 0x40;
    lms7002m_spi_modify_csr(self, LMS7002M_RSSIDC_DCO1, wrValue);
    lms7002m_spi_modify_csr(self, LMS7002M_EN_INSHSW_W_RFE, 0);
    return lime_Result_Success;
}
