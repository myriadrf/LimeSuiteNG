#include "LimeSDR_Micro.h"

#include "limesuiteng/embedded/lms7002m/lms7002m.h"
#include "limesuiteng/embedded/loglevel.h"

#include "embedded/lms7002m/spi.h"
#include "embedded/lms7002m/csr_data.h"
#include "embedded/lms7002m/privates.h"

#include "src/interface/IDCCorrector.h"
#include "src/interface/IQuadratureErrorCorrector.h"

#include <chrono>
#include <thread>

#include "gnuPlotPipe.h"
#include "DSP/FFT/FFT.h"
#include "streaming/samplesConversion.h"

#include "chips/LA9310/LA9310.h"

#define PLOT_GRAPHS 0
#if 0 // print debug messages
    #define printf_dbg_log(...) \
        do \
        { \
            printf(__VA_ARGS__); \
        } while (0)
#else
    #define printf_dbg_log(format, ...)
#endif

namespace lime {

static const uint8_t calibUserBwDivider = 5;
static const uint32_t offsetNCO = 100000;
static const uint32_t calibrationSXOffset_Hz = 1000000;

// basic math functions
#define math_abs(x) (x >= 0 ? x : -x)
#define math_pow2(power) (1 << power)

static const bool showPlots = true;

#if PLOT_GRAPHS
GNUPlotPipe plot;
GNUPlotPipe plotSamples;
#endif

struct CalibrationContext {
    VSPA_iqplayer* vspa;
    PHYTimer* phytimer;
    lms7002m_context* rfsoc;
    float sampleRate;
};

static void PlotSamples(const complex16_t* samples, size_t count)
{
#if PLOT_GRAPHS
    if (!showPlots)
        return;

    plotSamples.writef("set yrange[%f:%f]\n", -32768.0, 32767.0);
    plotSamples.write("plot '-' with lines, '-' with lines\n");
    uint32_t i = 0;
    for (; i < count; ++i)
        plotSamples.writef("%i %i\n", i, samples[i].real());
    plotSamples.write("e\n");
    i = 0;
    for (; i < count; ++i)
        plotSamples.writef("%i %i\n", i, samples[i].imag());
    plotSamples.write("e\n");
    plotSamples.flush();
#endif
}

static void PlotBins(std::vector<float> bins)
{
#if PLOT_GRAPHS
    if (!showPlots)
        return;

    const float sampleRate = 30.72e6;
    plot.writef("set xrange[%f:%f]\n set yrange[%i:%i]\n", -sampleRate / 2, sampleRate / 2, -120, 0);
    plot.write("plot '-' with lines\n");
    const int fftSize = bins.size();
    for (int j = fftSize / 2; j < fftSize; ++j)
        plot.writef("%f %f\n", sampleRate * (j - fftSize) / fftSize, bins[j]);
    for (int j = 0; j < fftSize / 2; ++j)
        plot.writef("%f %f\n", sampleRate * j / fftSize, bins[j]);
    plot.write("e\n");
    plot.flush();
#endif
}

static float la9310_get_rssi(CalibrationContext* ctx, float freq_offset)
{
    OpStatus status;
    status = ctx->vspa->StartRx();
    if (status != OpStatus::Success)
        printf("Failed start rx\n");
    // Disable all Rx and Tx DMA triggers
    constexpr uint8_t ids[] = { 3, 4, 11 };
    for (const auto id : ids)
    {
        PHYTimerControl timer = ctx->phytimer->GetTimerControl(id);
        timer.TriggerDirectly(PHYTimerControl::TriggerLogic::ForceOne);
    }
    const int samplesToRead = 2 * 512;
    complex16_t samples[samplesToRead];

    int samplesGot = 0;
    while (samplesGot < samplesToRead)
    {
        int readSize = (samplesToRead - samplesGot) * sizeof(complex16_t);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        uint32_t size_received = ctx->vspa->Receive(0, reinterpret_cast<uint32_t*>(&samples[samplesGot]), readSize, NULL);
        samplesGot += size_received / sizeof(complex16_t);
    }
    status = ctx->vspa->StopRx();
    if (status != OpStatus::Success)
        printf("Failed stop rx\n");

    std::vector<complex32f_t> fsamples(samplesToRead);
    for (int i = 0; i < samplesToRead; ++i)
        Rescale(fsamples[i], samples[i]);
    auto bins = lime::FFT::Calc(fsamples, FFT::WindowFunctionType::HANNING);
    lime::FFT::ConvertToDBFS(bins);
    PlotSamples(samples, samplesGot);
    PlotBins(bins);
    // int32_t rssi = 32760 + bins[0] * 50;
    // printf("dbFS %f, rssi:%i\n", bins[0], rssi);

    int binIndex = round(freq_offset / (ctx->sampleRate / samplesToRead));
    if (binIndex < 0)
        binIndex += samplesToRead;
    return bins[binIndex];
}

static float la9310_get_dc_rssi(CalibrationContext* ctx)
{
    return la9310_get_rssi(ctx, 0);
}

static float la9310_get_tx_signal_rssi(CalibrationContext* ctx)
{
    float tx_offset = 1e6;
    return la9310_get_rssi(ctx, tx_offset);
}

typedef struct {
    float (*set_and_measure)(int32_t value, void* userData);
    void* userData{};
    int32_t result; ///< The result of the search
    int32_t minValue; ///< Minumum value of the search
    int32_t maxValue; ///< Maximum value of the search
} BisectSearchParam;

static void bisection_find_min_rssi(BisectSearchParam* args)
{
    int32_t left = args->minValue;
    int32_t right = args->maxValue;

    float rssiLeft = args->set_and_measure(left, args->userData);
    float rssiRight = args->set_and_measure(right, args->userData);
    while (right - left > 1)
    {
        const int32_t step = (right - left) / 2;
        if (step <= 0)
            break;
        if (rssiLeft < rssiRight)
            right -= step;
        else
            left += step;

        if (rssiLeft < rssiRight)
            rssiRight = args->set_and_measure(right, args->userData);
        else
            rssiLeft = args->set_and_measure(left, args->userData);
    }
    args->result = rssiLeft < rssiRight ? left : right;
    args->set_and_measure(args->result, args->userData);
}

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

inline static int16_t to_signed(int16_t val, uint8_t msb)
{
    val <<= 15 - msb;
    val >>= 15 - msb;
    return val;
}

static const char* rssi_to_string(float rssi)
{
    static char rssi_string[64];
    snprintf(rssi_string, sizeof(rssi_string), "(RSSI: %3.3f dBFS)", rssi);
    return rssi_string;
}

static OpStatus ResultToStatus(lime_Result result)
{
    return static_cast<lime::OpStatus>(result);
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

static void lms7002m_write_masked_registers(lms7002m_context* rfsoc, const RegisterBatch* regs)
{
    for (uint8_t i = regs->cnt; i; --i)
    {
        const uint8_t index = i - 1;
        lms7002m_spi_write(
            rfsoc, regs->addr[index], (lms7002m_spi_read(rfsoc, regs->addr[index]) & ~regs->mask[index]) | regs->val[index]);
    }
    for (uint8_t i = regs->wrOnlyAddrCnt; i; --i)
    {
        const uint8_t index = i - 1;
        lms7002m_spi_write(rfsoc, regs->wrOnlyAddr[index], i > regs->wrOnlyDataCnt ? 0 : regs->wrOnlyData[index]);
    }
}

static void lms7002m_enable_mimo_buffers_if_necessary(lms7002m_context* rfsoc)
{
    //modifications when calibrating channel B
    const uint16_t x0020val = lms7002m_spi_read(rfsoc, 0x0020);
    if ((x0020val & 0x3) == 2)
    {
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_MAC, 1);
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_EN_NEXTRX_RFE, 1);
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_EN_NEXTTX_TRF, 1);
        lms7002m_spi_write(rfsoc, 0x0020, x0020val);
    }
}

static bool lms7002m_is_pll_tuned(lms7002m_context* rfsoc, const bool isTx)
{
    if (lms7002m_spi_read_bits(rfsoc, 0x0123, 13, 12) == 2)
        return true;
    lime_Result result = lms7002m_tune_vco(rfsoc, isTx ? LMS7002M_VCO_SXT : LMS7002M_VCO_SXR);
    return result == lime_Result_Success;
}

static void lms7002m_set_defaults_sx(lms7002m_context* rfsoc)
{
    const uint16_t SXAddr[] = { 0x011C, 0x011D, 0x011E, 0x011F, 0x0121, 0x0122, 0x0123 };
    const uint16_t SXdefVals[] = { 0xAD43, 0x0400, 0x0780, 0x3640, 0x3404, 0x033F, 0x067B };

    for (uint8_t i = 0; i < sizeof(SXAddr) / sizeof(uint16_t); ++i)
        lms7002m_spi_write(rfsoc, SXAddr[i], SXdefVals[i]);

    //keep 0x0120[7:0]ICT_VCO bias value intact
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_VDIV_VCO, 0xB9FF);
}

static void lms7002m_enable_channel_power_controls(lms7002m_context* rfsoc)
{
    uint16_t afe = lms7002m_spi_read(rfsoc, 0x0082);
    uint16_t value = lms7002m_spi_read(rfsoc, 0x0020);
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
    lms7002m_spi_write(rfsoc, 0x0020, value);
    lms7002m_spi_write(rfsoc, 0x0082, afe);
}

static void lms7002m_write_analog_dc(lms7002m_context* rfsoc, const uint16_t addr, int16_t value)
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
    lms7002m_spi_write(rfsoc, addr, regValue);
    lms7002m_spi_write(rfsoc, addr, regValue | 0x8000);
}

static float WriteRxDCI_Analog(int32_t value, void* context)
{
    CalibrationContext* ctx = static_cast<CalibrationContext*>(context);
    const uint16_t addrI = 0x5C7;
    lms7002m_write_analog_dc(ctx->rfsoc, addrI, value);
    return la9310_get_rssi(ctx, 0);
}

static float WriteRxDCQ_Analog(int32_t value, void* context)
{
    const uint16_t addrQ = 0x5C8;
    CalibrationContext* ctx = static_cast<CalibrationContext*>(context);
    lms7002m_write_analog_dc(ctx->rfsoc, addrQ, value);
    return la9310_get_rssi(ctx, 0);
}

static constexpr float tx_dc_if = 2e6;
static constexpr float tx_iq_imag_if = -1.83e6;
static constexpr float tx_test_signal_if = 5.83e6;
static float WriteTxDCI_Analog(int32_t value, void* context)
{
    CalibrationContext* ctx = static_cast<CalibrationContext*>(context);
    const uint16_t addrI = 0x5C3;
    lms7002m_write_analog_dc(ctx->rfsoc, addrI, value);
    return la9310_get_rssi(ctx, tx_dc_if);
}

static float WriteTxDCQ_Analog(int32_t value, void* context)
{
    const uint16_t addrQ = 0x5C4;
    CalibrationContext* ctx = static_cast<CalibrationContext*>(context);
    lms7002m_write_analog_dc(ctx->rfsoc, addrQ, value);
    return la9310_get_rssi(ctx, tx_dc_if);
}

static float WriteRxDCI_Digital(int32_t value, void* context)
{
    CalibrationContext* ctx = static_cast<CalibrationContext*>(context);
    ctx->vspa->GetRxDCCorrector()->SetDCI(value);
    return la9310_get_rssi(ctx, 0);
}

static float WriteRxDCQ_Digital(int32_t value, void* context)
{
    CalibrationContext* ctx = static_cast<CalibrationContext*>(context);
    ctx->vspa->GetRxDCCorrector()->SetDCQ(value);
    return la9310_get_rssi(ctx, 0);
}

static float WriteTxDCI_Digital(int32_t value, void* context)
{
    CalibrationContext* ctx = static_cast<CalibrationContext*>(context);
    ctx->vspa->GetTxDCCorrector()->SetDCI(value);
    return la9310_get_rssi(ctx, tx_dc_if);
}

static float WriteTxDCQ_Digital(int32_t value, void* context)
{
    CalibrationContext* ctx = static_cast<CalibrationContext*>(context);
    ctx->vspa->GetTxDCCorrector()->SetDCQ(value);
    return la9310_get_rssi(ctx, tx_dc_if);
}

static void limesdrmicro_calibrate_rx_dc(CalibrationContext* ctx)
{
    lms7002m_context* rfsoc = ctx->rfsoc;

    uint16_t dcRegAddr = 0x5C7;
    const uint8_t ch = lms7002m_spi_read_csr(rfsoc, LMS7002M_MAC);
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_EN_G_TRF, 0);

    lms7002m_spi_modify_csr(rfsoc, LMS7002M_DCMODE, 1);
    if (ch == 1)
    {
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_DCDAC_RXA, 0);
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_DCCMP_RXA, 0);
    }
    else
    {
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_DCDAC_RXB, 0);
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_DCCMP_RXB, 0);
        dcRegAddr += 2;
    }

    lms7002m_write_analog_dc(rfsoc, dcRegAddr, 0);
    lms7002m_write_analog_dc(rfsoc, dcRegAddr + 1, 0);

    int16_t dci = 0, dcq = 0;
    for (int i = 0; i < 2; ++i)
    {
        BisectSearchParam search;
        search.set_and_measure = WriteRxDCI_Analog;
        search.userData = ctx;
        search.minValue = dci - (63 >> i);
        search.maxValue = dci + (63 >> i);
        search.result = 0;
        bisection_find_min_rssi(&search);
        dci = search.result;

        search.set_and_measure = WriteRxDCQ_Analog;
        search.userData = ctx;
        search.minValue = dcq - (63 >> i);
        search.maxValue = dcq + (63 >> i);
        search.result = 0;
        bisection_find_min_rssi(&search);
        dcq = search.result;
    }

    LMS7002M_LOG(rfsoc,
        lime_LogLevel_Debug,
        "RxDC offset (analog) I:%3i, Q:%3i, %s",
        lms7002m_read_analog_dc(rfsoc, dcRegAddr),
        lms7002m_read_analog_dc(rfsoc, dcRegAddr + 1),
        rssi_to_string(la9310_get_dc_rssi(ctx)));

    // digital
    {
        const int16_t range = 16384;
        BisectSearchParam search;
        search.set_and_measure = WriteRxDCI_Digital;
        search.userData = ctx;
        search.minValue = -range;
        search.maxValue = range;
        search.result = 0;
        bisection_find_min_rssi(&search);
        dci = search.result;

        search.set_and_measure = WriteRxDCQ_Digital;
        search.userData = ctx;
        search.minValue = -range;
        search.maxValue = range;
        search.result = 0;
        bisection_find_min_rssi(&search);
        dcq = search.result;

        LMS7002M_LOG(rfsoc,
            lime_LogLevel_Debug,
            "RxDC offset (digital) I:%3i, Q:%3i, %s",
            dci,
            dcq,
            rssi_to_string(la9310_get_dc_rssi(ctx)));
    }
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_EN_G_TRF, 1);
}

static lime_Result lms7002m_calibrate_rx_setup(lms7002m_context* rfsoc, uint32_t bandwidthRF, bool extLoopback)
{
    const uint16_t x0020val = lms7002m_spi_read(rfsoc, 0x0020);
    //rfe
    {
        const uint16_t RxSetupAddr[] = { 0x0084, 0x0085, 0x00AE, 0x010C, 0x010D, 0x0113, 0x0115, 0x0119, 0x0020 };
        const uint16_t RxSetupData[] = { 0x0400, 0x0001, 0xF000, 0x0000, 0x0046, 0x000C, 0x0000, 0x1000, 0xFFFC };
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
        lms7002m_write_masked_registers(rfsoc, &batch);
    }

    lms7002m_spi_modify_csr(rfsoc, LMS7002M_OSW_PGA_RBB, 1);

    //AFE
    if ((x0020val & 0x3) == 1)
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_TX_AFE1, 0);
    else
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_TX_AFE2, 0);

    if (extLoopback) // external loopback
    {
        const uint8_t band1_band2 = 2;
        lms7002m_spi_modify(rfsoc, 0x0103, 11, 10, band1_band2);
        if (lms7002m_spi_read_csr(rfsoc, LMS7002M_SEL_PATH_RFE) != 0)
            return lime_Result_Error;
    }
    else //chip internal loopbacks
    {
        switch (lms7002m_spi_read_csr(rfsoc, LMS7002M_SEL_PATH_RFE))
        {
        case 2: //LNAL
            lms7002m_spi_modify(rfsoc, 0x0103, 11, 10, 1);
            break;
        case 3: //LNAW
        case 1: //LNAH
            lms7002m_spi_modify(rfsoc, 0x0103, 11, 10, 2);
            break;
        default:
            return lime_Result_Error;
        }
    }

    lms7002m_spi_modify_csr(rfsoc, LMS7002M_MAC, 2); //Get freq already changes/restores ch

    lime_Result status;
    if (lms7002m_spi_read_csr(rfsoc, LMS7002M_PD_LOCH_T2RBUF) == 0) //isTDD
    {
        //in TDD do nothing
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_MAC, 1);
        lms7002m_set_defaults_sx(rfsoc);
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_ICT_VCO, 255);
        const uint64_t sxt_lo = lms7002m_get_frequency_sx(rfsoc, true) - bandwidthRF / calibUserBwDivider - 9000000;
        status = lms7002m_set_frequency_sx(rfsoc, false, sxt_lo);
    }
    else
    {
        //SXR
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_MAC, 1);
        //check if Rx is tuned
        if (!lms7002m_is_pll_tuned(rfsoc, false))
            return lime_Result_Error;
        const uint64_t SXRfreqHz = lms7002m_get_frequency_sx(rfsoc, false);

        //SXT
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_MAC, 2);
        lms7002m_set_defaults_sx(rfsoc);
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_ICT_VCO, 255);
        status = lms7002m_set_frequency_sx(rfsoc, true, SXRfreqHz + bandwidthRF / calibUserBwDivider + 9000000);
    }
    lms7002m_spi_write(rfsoc, 0x0020, x0020val);
    if (status != lime_Result_Success)
        return status;

    // lms7002m_load_dc_reg_tx_iq(rfsoc);

    //CGEN
    // status = lms7002m_setup_cgen(rfsoc);
    // if (status != lime_Result_Success)
    //     return status;

    // lms7002m_set_rx_gfir3_coefficients(rfsoc);
    lms7002m_set_nco_frequency(rfsoc, true, 0, 9000000);
    // lms7002m_set_nco_frequency(rfsoc, false, 0, bandwidthRF / calibUserBwDivider - offsetNCO);
    //modifications when calibrating channel B
    lms7002m_enable_mimo_buffers_if_necessary(rfsoc);
    lms7002m_enable_channel_power_controls(rfsoc);
    return lime_Result_Success;
}

static lime_Result lms7002m_check_saturation_rx(CalibrationContext* ctx, const uint32_t bandwidth_Hz, bool extLoopback)
{
    lms7002m_context* rfsoc = ctx->rfsoc;
    float target_rssi = -10.0; // dbfs_
    uint8_t cg_iamp = (uint8_t)lms7002m_spi_read_csr(rfsoc, LMS7002M_CG_IAMP_TBB);

    lms7002m_spi_modify_csr(rfsoc, LMS7002M_CMIX_SC_RXTSP, 1);
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_CMIX_BYP_RXTSP, 0);
    lms7002m_set_nco_frequency(rfsoc, false, 0, bandwidth_Hz / calibUserBwDivider - offsetNCO);

    float rssi = 0;
    if (extLoopback)
    {
        int8_t g_lossmain = 15;
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_LOSS_MAIN_TXPAD_TRF, g_lossmain);
        rssi = la9310_get_tx_signal_rssi(ctx);
        LMS7002M_LOG(rfsoc,
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
            lms7002m_spi_modify_csr(rfsoc, LMS7002M_LOSS_MAIN_TXPAD_TRF, g_lossmain);
            rssi = la9310_get_tx_signal_rssi(ctx);
        }
    }
    else
    {
        uint8_t g_rxloopb_rfe = 2;
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_G_RXLOOPB_RFE, g_rxloopb_rfe);
        rssi = la9310_get_tx_signal_rssi(ctx);
        LMS7002M_LOG(rfsoc,
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
            lms7002m_spi_modify_csr(rfsoc, LMS7002M_G_RXLOOPB_RFE, g_rxloopb_rfe);
            rssi = la9310_get_tx_signal_rssi(ctx);
        }
    }

    target_rssi = -17.0; // dBFS
    while (rssi < target_rssi)
    {
        if (cg_iamp > 57)
            cg_iamp += 1;
        else
            cg_iamp += 2;

        if (cg_iamp > 63)
            break;
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_CG_IAMP_TBB, cg_iamp);
        rssi = la9310_get_tx_signal_rssi(ctx);
    }

    if (extLoopback)
    {
        LMS7002M_LOG(rfsoc,
            lime_LogLevel_Debug,
            "Adjusted gains:\tLOSS_MAIN_TXPAD: %2i, CG_IAMP: %2i | %s",
            lms7002m_spi_read_csr(rfsoc, LMS7002M_LOSS_MAIN_TXPAD_TRF),
            lms7002m_spi_read_csr(rfsoc, LMS7002M_CG_IAMP_TBB),
            rssi_to_string(rssi));
    }
    else
    {
        LMS7002M_LOG(rfsoc,
            lime_LogLevel_Debug,
            "Adjusted gains: G_RXLOOPB: %2i, CG_IAMP: %2i | %s",
            lms7002m_spi_read_csr(rfsoc, LMS7002M_G_RXLOOPB_RFE),
            lms7002m_spi_read_csr(rfsoc, LMS7002M_CG_IAMP_TBB),
            rssi_to_string(rssi));
    }

    const float expectedRSSI_level = -30.0;
    const float failureRSSI_level = -50.0;
    if (rssi < expectedRSSI_level)
    {
        char expectedRSSI_string[32];
        strcpy(expectedRSSI_string, rssi_to_string(expectedRSSI_level));
        LMS7002M_LOG(rfsoc,
            rssi > failureRSSI_level ? lime_LogLevel_Warning : lime_LogLevel_Error,
            "Low calibration test signal level %s, expected to be more than %s."
            " Calibration results might be impacted. Try re-calibrating or adjusting the RX gains.",
            rssi_to_string(rssi),
            expectedRSSI_string);
    }
    return rssi > failureRSSI_level ? lime_Result_Success : lime_Result_Error;
}

static float lastPhase = 0;
static float lastGain = 0;
static float WriteRxIQPhase(int32_t value, void* context)
{
    CalibrationContext* ctx = static_cast<CalibrationContext*>(context);
    const float phase_degrees = value / 1000.0;
    ctx->vspa->GetRxQEC()->SetImbalance(lastGain, phase_degrees);
    const float rssi = la9310_get_rssi(ctx, -1e6);
    printf_dbg_log("Rx IQ phase: %g deg, gain: %g , rssi : %fdBFS\n", phase_degrees, lastGain, rssi);
    lastPhase = phase_degrees;
    return rssi;
}

static float WriteRxIQGain(int32_t value, void* context)
{
    CalibrationContext* ctx = static_cast<CalibrationContext*>(context);
    const float imbalance_db = value / 1000.0;
    ctx->vspa->GetRxQEC()->SetImbalance(imbalance_db, lastPhase);
    const float rssi = la9310_get_rssi(ctx, -1e6);
    printf_dbg_log("Rx IQ gain: %g , phase: %g, rssi : %fdBFS\n", imbalance_db, lastPhase, rssi);
    lastGain = imbalance_db;
    return rssi;
}

static float WriteTxIQPhase(int32_t value, void* context)
{
    CalibrationContext* ctx = static_cast<CalibrationContext*>(context);
    const float phase_degrees = value / 1000.0;
    ctx->vspa->GetTxQEC()->SetImbalance(lastGain, phase_degrees);
    const float rssi = la9310_get_rssi(ctx, tx_iq_imag_if);
    printf_dbg_log("Tx IQ phase: %g deg, gain: %g , rssi : %fdBFS\n", phase_degrees, lastGain, rssi);
    lastPhase = phase_degrees;
    return rssi;
}

static float WriteTxIQGain(int32_t value, void* context)
{
    CalibrationContext* ctx = static_cast<CalibrationContext*>(context);
    const float imbalance_db = value / 1000.0;
    ctx->vspa->GetTxQEC()->SetImbalance(imbalance_db, lastPhase);
    const float rssi = la9310_get_rssi(ctx, tx_iq_imag_if);
    printf_dbg_log("Tx IQ gain: %g , phase: %g, rssi : %fdBFS\n", imbalance_db, lastPhase, rssi);
    lastGain = imbalance_db;
    return rssi;
}

static void lms7002m_calibrate_iq_imbalance(CalibrationContext* ctx, bool isTx)
{
    lms7002m_context* rfsoc = ctx->rfsoc;
    const char* const dirName = isTx ? "Tx" : "Rx";
    lastPhase = 0;
    lastGain = 0;

    BisectSearchParam phaseSearch;
    phaseSearch.minValue = -45000;
    phaseSearch.maxValue = 45000;
    phaseSearch.result = 0;
    phaseSearch.set_and_measure = isTx ? WriteTxIQPhase : WriteRxIQPhase;
    phaseSearch.userData = ctx;
    bisection_find_min_rssi(&phaseSearch);

    BisectSearchParam gainSearch;
    gainSearch.minValue = -6000;
    gainSearch.maxValue = 6000;
    gainSearch.result = 0;
    gainSearch.set_and_measure = isTx ? WriteTxIQGain : WriteRxIQGain;
    gainSearch.userData = ctx;
    bisection_find_min_rssi(&gainSearch);

    phaseSearch.minValue = phaseSearch.result - 1500;
    phaseSearch.maxValue = phaseSearch.result + 1500;
    phaseSearch.result = 0;
    phaseSearch.set_and_measure = isTx ? WriteTxIQPhase : WriteRxIQPhase;
    phaseSearch.userData = ctx;
    bisection_find_min_rssi(&phaseSearch);

    gainSearch.minValue = gainSearch.result - 1000;
    gainSearch.maxValue = gainSearch.result + 1000;
    gainSearch.result = 0;
    gainSearch.set_and_measure = isTx ? WriteTxIQGain : WriteRxIQGain;
    ;
    gainSearch.userData = ctx;
    bisection_find_min_rssi(&gainSearch);

    const float imbalance_db = gainSearch.result / 1000.0;
    const float phase_deg = phaseSearch.result / 1000.0;
    if (isTx)
        ctx->vspa->GetTxQEC()->SetImbalance(imbalance_db, phase_deg);
    else
        ctx->vspa->GetRxQEC()->SetImbalance(imbalance_db, phase_deg);
    const float rssi = la9310_get_rssi(ctx, -1e6);
    printf_dbg_log("QEC gain: %gdb, phase: %gdeg, rssi : %fdBFS\n", imbalance_db, phase_deg, rssi);
}

OpStatus LimeSDR_Micro::CalibrateRx()
{
    CalibrationContext context;
    context.vspa = &la9310->vspa;
    context.phytimer = &la9310->phytimer;
    context.rfsoc = mLMSChips[0]->mC_impl;
    context.sampleRate = GetSampleRate(0, TRXDir::Rx, 0, nullptr);

    lms7002m_context* rfsoc = context.rfsoc;

    context.vspa->GetRxDCCorrector()->SetDCOffset(complex16_t(0, 0));
    context.vspa->GetRxQEC()->SetImbalance(0, 0);

    context.vspa->SetupRx(0, 1024 * 1024, 16384 * sizeof(complex16_t) * 64);
    const bool extLoopback = false;
    const bool dcOnly = false;
    const uint32_t bandwidthRF = 5e6;
    // if (bandwidthRF < offsetNCO * calibUserBwDivider)
    //     bandwidthRF = offsetNCO * calibUserBwDivider;

    const uint16_t x0020val = lms7002m_spi_read(rfsoc, 0x0020); //remember used channel

    LMS7002M_LOG(rfsoc,
        lime_LogLevel_Debug,
        "Rx calibrate ch.%s @ %lu Hz, BW: %u Hz, RF input: %s, PGA: %i, LNA: %i, TIA: %i",
        (x0020val & 0x3) == 1 ? "A" : "B",
        lms7002m_get_frequency_sx(rfsoc, false),
        bandwidthRF,
        get_lna_name(lms7002m_spi_read_csr(rfsoc, LMS7002M_SEL_PATH_RFE)),
        lms7002m_spi_read_csr(rfsoc, LMS7002M_G_PGA_RBB),
        lms7002m_spi_read_csr(rfsoc, LMS7002M_G_LNA_RFE),
        lms7002m_spi_read_csr(rfsoc, LMS7002M_G_TIA_RFE));
    lms7002m_save_chip_state(rfsoc, false);

    lime_Result status = lms7002m_calibrate_rx_setup(rfsoc, bandwidthRF, extLoopback);
    if (status != 0)
        goto RxCalibrationEndStage;
    limesdrmicro_calibrate_rx_dc(&context);

    if (dcOnly)
        goto RxCalibrationEndStage;
    if (!extLoopback)
    {
        if ((uint8_t)lms7002m_spi_read_csr(rfsoc, LMS7002M_SEL_PATH_RFE) == 2)
        {
            lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_RLOOPB_2_RFE, 0);
            lms7002m_spi_modify_csr(rfsoc, LMS7002M_EN_INSHSW_LB2_RFE, 0);
        }
        else
        {
            lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_RLOOPB_1_RFE, 0);
            lms7002m_spi_modify_csr(rfsoc, LMS7002M_EN_INSHSW_LB1_RFE, 0);
        }
    }

    {
        const uint8_t mac = x0020val & 0x0003;
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_MAC, 2);
        if (lms7002m_spi_read_csr(rfsoc, LMS7002M_PD_LOCH_T2RBUF) == false)
        {
            lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_LOCH_T2RBUF, 1);
            //TDD MODE
            lms7002m_spi_modify_csr(rfsoc, LMS7002M_MAC, 1);
            lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_VCO, 0);
        }
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_MAC, mac);
        status = lms7002m_check_saturation_rx(&context, bandwidthRF, extLoopback);
        if (status != lime_Result_Success)
            goto RxCalibrationEndStage;
        // lms7002m_spi_modify_csr(rfsoc, LMS7002M_CMIX_SC_RXTSP, 0);
        // lms7002m_spi_modify_csr(rfsoc, LMS7002M_CMIX_BYP_RXTSP, 0);
        // lms7002m_set_nco_frequency(rfsoc, false, 0, bandwidthRF / calibUserBwDivider + offsetNCO);
        lms7002m_calibrate_iq_imbalance(&context, false);
    }
RxCalibrationEndStage : {
    lms7002m_save_chip_state(rfsoc, true);
    lms7002m_spi_write(rfsoc, 0x0020, x0020val);
    if (status != lime_Result_Success)
    {
        LMS7002M_LOG(rfsoc, lime_LogLevel_Debug, "%s", "Rx calibration failed");
        return ResultToStatus(status);
    }
}
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_DCMODE, 1);
    if (x0020val & 0x1)
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_DCDAC_RXA, 0);
    else
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_DCDAC_RXB, 0);
    LMS7002M_LOG(rfsoc, lime_LogLevel_Info, "%s", "Rx calibration finished");
    return ResultToStatus(lime_Result_Success);
}

static lime_Result lms7002m_calibrate_tx_setup(lms7002m_context* rfsoc, uint32_t bandwidthRF, bool extLoopback)
{
    const uint16_t x0020val = lms7002m_spi_read(rfsoc, 0x0020); //remember used channel

    if ((x0020val & 0x3) == 1)
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_RX_AFE1, 0);
    else
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_RX_AFE2, 0);
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
        lms7002m_write_masked_registers(rfsoc, &batch);
    }

    lms7002m_spi_modify_csr(rfsoc, LMS7002M_OSW_PGA_RBB, 1);
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_TSTIN_TBB, 3);
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_EN_DIR_TRF, 0);
    // lms7002m_set_rx_gfir3_coefficients(rfsoc);
    // lime_Result status = lms7002m_setup_cgen(rfsoc);
    // if (status != lime_Result_Success)
    //     return status;
    //SXR
    const uint8_t mac = x0020val & 0x0003;
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_MAC, 1); //switch to ch. A
    lms7002m_set_defaults_sx(rfsoc);
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_ICT_VCO, 255);
    {
        const uint64_t SXRfreq = lms7002m_get_frequency_sx(rfsoc, true) - bandwidthRF / calibUserBwDivider - calibrationSXOffset_Hz;
        //SX VCO is powered up in SetFrequencySX/Tune
        lime_Result status = lms7002m_set_frequency_sx(rfsoc, false, SXRfreq);
        if (status != lime_Result_Success)
        {
            lms7002m_spi_read(rfsoc, x0020val); //restore used channel
            return status;
        }
    }

    //SXT{
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_MAC, 2); //switch to ch. B
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_LOCH_T2RBUF, 1);
    //check if Tx is tuned
    if (!lms7002m_is_pll_tuned(rfsoc, true))
    {
        lms7002m_spi_write(rfsoc, 0x0020, x0020val); //restore used channel
        return lime_Result_Error;
    }

    lms7002m_spi_modify_csr(rfsoc, LMS7002M_MAC, mac); //restore used channel

    // lms7002m_load_dc_reg_tx_iq(rfsoc);
    // lms7002m_set_nco_frequency(rfsoc, true, 0, bandwidthRF / calibUserBwDivider);

    {
        const uint8_t sel_band1_2_trf = (uint8_t)lms7002m_spi_read_bits(rfsoc, 0x0103, 11, 10);
        if (extLoopback)
        {
            lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_LNA_RFE, 0);
            if (sel_band1_2_trf == 1 || sel_band1_2_trf == 2)
            {
                lms7002m_spi_modify_csr(rfsoc, LMS7002M_SEL_PATH_RFE, 0);
                lms7002m_spi_modify(rfsoc, 0x010D, 2, 1, ~(0 - 1)); //EN_INSHSW_*_RFE

                //check if correct tx band for external loop
                if (0 != !(sel_band1_2_trf - 1))
                    return lime_Result_Error;
            }
            else
            {
                LMS7002M_LOG(
                    rfsoc, lime_LogLevel_Error, "%s", "Tx Calibration: external calibration is not supported on selected Tx Band");
                return lime_Result_Error;
            }
        }
        else
        {
            if (sel_band1_2_trf != 0x1 && sel_band1_2_trf != 0x2) //BAND1
            {
                LMS7002M_LOG(rfsoc, lime_LogLevel_Error, "%s", "Tx Calibration: band not selected");
                return lime_Result_Error;
            }
            lms7002m_spi_modify_csr(rfsoc, LMS7002M_SEL_PATH_RFE, sel_band1_2_trf + 1);
            lms7002m_spi_modify(rfsoc, 0x010C, 6, 5, sel_band1_2_trf ^ 0x3);
            lms7002m_spi_modify(rfsoc, 0x010D, 4, 3, sel_band1_2_trf ^ 0x3);
        }
    }
    //if calibrating ch. B enable buffers
    lms7002m_enable_mimo_buffers_if_necessary(rfsoc);
    lms7002m_enable_channel_power_controls(rfsoc);

    return lime_Result_Success;
}

static lime_Result lms7002m_check_saturation_tx_rx(CalibrationContext* ctx, uint32_t bandwidthRF, bool extLoopback)
{
    const float signalOffset = 5.841e6;
    const float saturationLevel = -12.86; // dBFS
    lms7002m_context* rfsoc = ctx->rfsoc;
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_DC_BYP_RXTSP, 0);
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_CMIX_BYP_RXTSP, 0);

    // lms7002m_set_nco_frequency(rfsoc, false, 0, calibrationSXOffset_Hz - offsetNCO + (bandwidthRF / calibUserBwDivider) * 2);

    uint8_t g_pga = (uint8_t)lms7002m_spi_read_csr(rfsoc, LMS7002M_G_PGA_RBB);
    uint8_t g_rfe = 0;
    if (extLoopback)
    {
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_G_LNA_RFE, g_rfe);
    }
    else
    {
        g_rfe = (uint8_t)lms7002m_spi_read_csr(rfsoc, LMS7002M_G_RXLOOPB_RFE);
    }

    float rssi = la9310_get_rssi(ctx, signalOffset);
    LMS7002M_LOG(rfsoc, lime_LogLevel_Debug, "Receiver saturation search, target level: (%s)", rssi_to_string(saturationLevel));
    LMS7002M_LOG(rfsoc,
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
            lms7002m_spi_modify_csr(rfsoc, LMS7002M_G_LNA_RFE, g_rfe);
        else
            lms7002m_spi_modify_csr(rfsoc, LMS7002M_G_RXLOOPB_RFE, g_rfe);

        rssi = la9310_get_rssi(ctx, signalOffset);
        printf_dbg_log("g_rfe: %i, rssi: %s\n", g_rfe, rssi_to_string(rssi));
    }

    {
        float rssi_prev = rssi;
        while (g_pga < 25 && g_rfe == 15 && rssi < saturationLevel)
        {
            if (g_pga < 25)
                ++g_pga;
            else
                break;

            lms7002m_spi_modify_csr(rfsoc, LMS7002M_G_PGA_RBB, g_pga);
            rssi = la9310_get_rssi(ctx, signalOffset);

            printf_dbg_log("g_pga: %i, rssi: %s\n", g_pga, rssi_to_string(rssi));

            if (rssi - rssi_prev < 0.9)
                break;
            //if ((float)rssi / rssi_prev < 1.05) // pga should give ~1dB change
            // if ((rssi * 100) / rssi_prev < 105) // pga should give ~1dB change
            //     break;

            rssi_prev = rssi;
        }
    }
    LMS7002M_LOG(rfsoc,
        lime_LogLevel_Debug,
        "adjusted PGA: %2i, %s: %2i, %s",
        lms7002m_spi_read_csr(rfsoc, LMS7002M_G_PGA_RBB),
        (extLoopback ? "LNA" : "RXLOOPB"),
        g_rfe,
        rssi_to_string(rssi));

    const float expectedRSSI_level = -30.0; //2849; // dbfs_to_chip_rssi(-30.0);
    const float failureRSSI_level = -50; //285; // dbfs_to_chip_rssi(-50)
    if (rssi < expectedRSSI_level)
    {
        char expectedRSSI_string[32];
        strcpy(expectedRSSI_string, rssi_to_string(expectedRSSI_level));
        LMS7002M_LOG(rfsoc,
            rssi > failureRSSI_level ? lime_LogLevel_Warning : lime_LogLevel_Error,
            "Low calibration test signal level %s, expected to be more than %s."
            " Calibration results might be impacted. Try re-calibrating or adjusting the TX gains.",
            rssi_to_string(rssi),
            expectedRSSI_string);
        if (rssi < failureRSSI_level)
            return lime_Result_Error;
    }
    return lime_Result_Success;
}

static void limesdrmicro_calibrate_tx_dc(CalibrationContext* ctx)
{
    lms7002m_context* rfsoc = ctx->rfsoc;
    uint16_t dcRegAddr = 0x5C3;
    const uint8_t ch = lms7002m_spi_read_csr(rfsoc, LMS7002M_MAC);
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_EN_G_TRF, 1);

    lms7002m_spi_modify_csr(rfsoc, LMS7002M_DCMODE, 1);
    if (ch == 1)
    {
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_DCDAC_RXA, 0);
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_DCCMP_RXA, 0);
    }
    else
    {
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_DCDAC_RXB, 0);
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_DCCMP_RXB, 0);
        dcRegAddr += 2;
    }

    lms7002m_write_analog_dc(rfsoc, dcRegAddr, 0);
    lms7002m_write_analog_dc(rfsoc, dcRegAddr + 1, 0);

    int16_t dci = 0, dcq = 0;
    for (int i = 0; i < 2; ++i)
    {
        BisectSearchParam search;
        search.set_and_measure = WriteTxDCI_Analog;
        search.userData = ctx;
        search.minValue = dci - (1023 >> i);
        search.maxValue = dci + (1023 >> i);
        search.result = 0;
        bisection_find_min_rssi(&search);
        dci = search.result;

        search.set_and_measure = WriteTxDCQ_Analog;
        search.userData = ctx;
        search.minValue = dcq - (1023 >> i);
        search.maxValue = dcq + (1023 >> i);
        search.result = 0;
        bisection_find_min_rssi(&search);
        dcq = search.result;
    }

    LMS7002M_LOG(rfsoc,
        lime_LogLevel_Debug,
        "TxDC offset (analog) I:%3i, Q:%3i, %s",
        lms7002m_read_analog_dc(rfsoc, dcRegAddr),
        lms7002m_read_analog_dc(rfsoc, dcRegAddr + 1),
        rssi_to_string(la9310_get_rssi(ctx, tx_dc_if)));

    // digital
    {
        const int16_t range = 16384;
        BisectSearchParam search;
        search.set_and_measure = WriteTxDCI_Digital;
        search.userData = ctx;
        search.minValue = -range;
        search.maxValue = range;
        search.result = 0;
        bisection_find_min_rssi(&search);
        dci = search.result;

        search.set_and_measure = WriteTxDCQ_Digital;
        search.userData = ctx;
        search.minValue = -range;
        search.maxValue = range;
        search.result = 0;
        bisection_find_min_rssi(&search);
        dcq = search.result;

        LMS7002M_LOG(rfsoc,
            lime_LogLevel_Debug,
            "TxDC offset (digital) I:%3i, Q:%3i, %s",
            dci,
            dcq,
            rssi_to_string(la9310_get_rssi(ctx, tx_dc_if)));
    }
}

OpStatus LimeSDR_Micro::CalibrateTx()
{
    CalibrationContext context;
    context.vspa = &la9310->vspa;
    context.phytimer = &la9310->phytimer;
    context.rfsoc = mLMSChips[0]->mC_impl;
    context.sampleRate = GetSampleRate(0, TRXDir::Rx, 0, nullptr);

    lms7002m_context* rfsoc = context.rfsoc;

    context.vspa->GetTxDCCorrector()->SetDCOffset(complex16_t(0, 0));
    context.vspa->GetTxQEC()->SetImbalance(0, 0);
    context.vspa->StartTxTone(false);

    context.vspa->SetupRx(0, 1024 * 1024, 16384 * sizeof(complex16_t) * 64);

    const uint32_t bandwidthRF = 5e6;
    bool extLoopback = false;

    const uint16_t x0020val = lms7002m_spi_read(rfsoc, 0x0020);
    LMS7002M_LOG(rfsoc,
        lime_LogLevel_Debug,
        "Tx ch.%s , BW: %u Hz, RF output: %s, Gain: %i, loopb: %s",
        (x0020val & 3) == 0x1 ? "A" : "B",
        bandwidthRF,
        lms7002m_spi_read_csr(rfsoc, LMS7002M_SEL_BAND1_TRF) == 1 ? "BAND1" : "BAND2",
        lms7002m_spi_read_csr(rfsoc, LMS7002M_CG_IAMP_TBB),
        extLoopback ? "external" : "internal");

    lms7002m_save_chip_state(rfsoc, false);
    lime_Result status = lms7002m_calibrate_tx_setup(rfsoc, bandwidthRF, extLoopback);
    if (status != lime_Result_Success)
        goto TxCalibrationEnd; //go to ending stage to restore registers

    lms7002m_spi_modify_csr(rfsoc, LMS7002M_EN_TXTSP, 0);
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_TX_AFE1, 1);
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_EN_G_TRF, 0);
    limesdrmicro_calibrate_rx_dc(&context);
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_EN_G_TRF, 1);

    context.vspa->StartTxTone(true);
    status = lms7002m_check_saturation_tx_rx(&context, bandwidthRF, extLoopback);
    if (status != lime_Result_Success)
        goto TxCalibrationEnd;
    context.vspa->StartTxTone(false);
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_EN_G_TRF, 0);
    limesdrmicro_calibrate_rx_dc(&context);
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_EN_G_TRF, 1);

    // lms7002m_set_nco_frequency(rfsoc, false, 0, calibrationSXOffset_Hz - offsetNCO + (bandwidthRF / calibUserBwDivider));
    context.vspa->StartTxTone(true);
    limesdrmicro_calibrate_tx_dc(&context);
    // lms7002m_set_nco_frequency(rfsoc, false, 0, calibrationSXOffset_Hz - offsetNCO);
    // context.vspa->StartTxTone(true);
    lms7002m_calibrate_iq_imbalance(&context, true);
    context.vspa->StartTxTone(false);
TxCalibrationEnd : {
    lms7002m_save_chip_state(rfsoc, true);
    lms7002m_spi_write(rfsoc, 0x0020, x0020val);
    if (status != lime_Result_Success)
    {
        LMS7002M_LOG(rfsoc, lime_LogLevel_Debug, "%s", "Tx calibration failed");
        return ResultToStatus(status);
    }

    lms7002m_spi_modify_csr(rfsoc, LMS7002M_DCMODE, 1);
    if ((x0020val & 1) == 1)
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_DCDAC_TXA, 0);
    else
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_DCDAC_TXB, 0);
    return ResultToStatus(lime_Result_Success);
}
}

} // namespace lime