#include "LimeSDR_Micro.h"

#include "limesuiteng/embedded/lms7002m/lms7002m.h"
#include "limesuiteng/embedded/loglevel.h"

#include "chips/LMS7002M/DCCorrector.h"

#include "embedded/lms7002m/spi.h"
#include "embedded/lms7002m/spi_batch.h"
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

#include <functional>
#include <iostream>

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
    float lo_diff;
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

static void PlotBins(std::vector<float> bins, float sampleRate)
{
#if PLOT_GRAPHS
    if (!showPlots)
        return;

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
    status = ctx->vspa->StopRx();
    if (status != OpStatus::Success)
        printf("Failed stop rx\n");
    ctx->vspa->ClearStats();
    status = ctx->vspa->StartRx();
    if (status != OpStatus::Success)
        printf("Failed start rx\n");
    // Enable all Rx and Tx DMA triggers
    constexpr uint8_t ids[] = { 1, 2, 3, 4 };
    for (const auto id : ids)
    {
        PHYTimerControl timer = ctx->phytimer->GetTimerControl(id);
        timer.TriggerDirectly(PHYTimerControl::TriggerLogic::ForceOne);
    }
    const int samplesToRead = 512;
    complex16_t samples[samplesToRead];

    int samplesGot = 0;
    auto t1 = std::chrono::high_resolution_clock::now();
    auto t2 = t1;
    do
    {
        int readSize = (samplesToRead - samplesGot) * sizeof(complex16_t);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        uint32_t size_received = ctx->vspa->Receive(0, reinterpret_cast<uint32_t*>(&samples[samplesGot]), readSize, NULL);
        samplesGot += size_received / sizeof(complex16_t);
        t2 = std::chrono::high_resolution_clock::now();
    } while (samplesGot < samplesToRead &&
             std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1) < std::chrono::milliseconds(100));

    if (samplesGot == 0)
    {
        lime::error("Failed to get RSSI\n");
        return NAN;
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
    PlotBins(bins, ctx->sampleRate);
    // int32_t rssi = 32760 + bins[0] * 50;

    int binIndex = (freq_offset / ctx->sampleRate) * samplesToRead;
    if (binIndex < 0)
        binIndex += samplesToRead;

    // printf("b[%i]:%g rssi:%g dbFS\n", binIndex, freq_offset, bins[binIndex]);
    return bins[binIndex];
}

template<typename Ctrl, typename M, typename controlT>
static controlT bisection_find_min_rssi2(lime::Range<controlT> range, Ctrl&& setControl, M&& measure)
{
    controlT left = range.min;
    controlT right = range.max;

    setControl(left);
    auto rssiLeft = measure();
    // std::cout << "Set:" << left << " | m:" << rssiLeft << std::endl;
    setControl(right);
    auto rssiRight = measure();
    // std::cout << "Set:" << right << " | m:" << rssiRight << std::endl;
    while (right - left > range.step)
    {
        const controlT delta = (right - left) / 2;
        if (delta <= range.step)
            break;
        if (rssiLeft < rssiRight)
            right -= delta;
        else
            left += delta;

        if (rssiLeft < rssiRight)
        {
            setControl(right);
            rssiRight = measure();
            // std::cout << "Set:" << right << " | m:" << rssiRight << std::endl;
        }
        else
        {
            setControl(left);
            rssiLeft = measure();
            // std::cout << "Set:" << left << " | m:" << rssiLeft << std::endl;
        }
    }
    controlT result = rssiLeft < rssiRight ? left : right;
    setControl(result);
    return result;
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

static void limesdrmicro_calibrate_rx_dc(CalibrationContext* ctx, uint8_t channel)
{
    lms7002m_context* rfsoc = ctx->rfsoc;
    LMS7002M_LOG(rfsoc, lime_LogLevel_Debug, ">%s", __func__);

    spi_batch_t batch;
    spi_batch_init(&batch);
    spi_batch_modify_csr(&batch, LMS7002M_EN_G_TRF, 0); // disable transmitter
    spi_batch_modify_csr(&batch, LMS7002M_DCMODE, 1); // Rx DC correctors selection
    lms7002m_spi_batch_flush(&batch, rfsoc);

    std::shared_ptr<IDCCorrector> digitalDC = ctx->vspa->GetRxDCCorrector();
    digitalDC->SetDCOffset(complex16_t(0, 0));

    std::unique_ptr<IDCCorrector> analogDC = std::make_unique<LMS7002M_DC>(rfsoc, TRXDir::Rx, channel);
    analogDC->Enabled(true);
    int16_t dci = 0, dcq = 0;
    analogDC->SetDCOffset(complex16_t(dci, dcq));

    // analog
    for (int i = 0; i < 1; ++i)
    {
        dci = bisection_find_min_rssi2(
            analogDC->GetRange(),
            [&analogDC](float value) { analogDC->SetDCI(value); },
            [&ctx]() { return la9310_get_rssi(ctx, 0); });

        dcq = bisection_find_min_rssi2(
            analogDC->GetRange(),
            [&analogDC](float value) { analogDC->SetDCQ(value); },
            [&ctx]() { return la9310_get_rssi(ctx, 0); });
    }

    LMS7002M_LOG(
        rfsoc, lime_LogLevel_Verbose, "RxDC offset (analog) I:%3i, Q:%3i, %s", dci, dcq, rssi_to_string(la9310_get_rssi(ctx, 0)));

    // digital
    {
        dci = bisection_find_min_rssi2(
            digitalDC->GetRange(),
            [&digitalDC](float value) { digitalDC->SetDCI(value); },
            [&ctx]() { return la9310_get_rssi(ctx, 0); });

        dcq = bisection_find_min_rssi2(
            digitalDC->GetRange(),
            [&digitalDC](float value) { digitalDC->SetDCQ(value); },
            [&ctx]() { return la9310_get_rssi(ctx, 0); });

        LMS7002M_LOG(rfsoc,
            lime_LogLevel_Verbose,
            "RxDC offset (digital) I:%3i, Q:%3i, %s",
            dci,
            dcq,
            rssi_to_string(la9310_get_rssi(ctx, 0)));
    }
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_EN_G_TRF, 1);
}

static OpStatus lms7002m_calibrate_rx_setup(CalibrationContext* ctx, uint32_t calibrationRF)
{
    lms7002m_context* rfsoc = ctx->rfsoc;
    const uint16_t x0020val = lms7002m_spi_read(rfsoc, 0x0020);

    // use external ADC/DAC
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_OSW_PGA_RBB, 1);
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_TSTIN_TBB, 3);

    // Select Tx band for internal loopback
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
            return OpStatus::Error;
        }
    }

    lms7002m_spi_modify_csr(rfsoc, LMS7002M_MAC, 2);

    lime_Result status = lime_Result_Success;
    if (lms7002m_spi_read_csr(rfsoc, LMS7002M_PD_LOCH_T2RBUF) == 0) //isTDD
    {
        // do nothing
        const double sxt_lo = lms7002m_get_frequency_sx(rfsoc, true);
        LMS7002M_LOG(rfsoc, lime_LogLevel_Debug, "RxLO@%g, TxLO@%g", sxt_lo, sxt_lo);
        ctx->lo_diff = 0;
    }
    else
    {
        //SXR
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_MAC, 1);
        //check if Rx is tuned
        if (!lms7002m_is_pll_tuned(rfsoc, false))
            return OpStatus::Error;
        const double sxr_lo = lms7002m_get_frequency_sx(rfsoc, false);

        //SXT
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_MAC, 2);
        ctx->lo_diff = (ctx->sampleRate / 8);
        const double sxt_lo = sxr_lo - ctx->lo_diff;
        status = lms7002m_set_frequency_sx(rfsoc, true, sxt_lo);
        LMS7002M_LOG(rfsoc, lime_LogLevel_Debug, "RxLO@%g, TxLO@%g", sxr_lo, sxt_lo);
    }
    lms7002m_spi_write(rfsoc, 0x0020, x0020val);
    if (status != lime_Result_Success)
        return OpStatus::Error;

    // modifications when calibrating channel B
    lms7002m_enable_mimo_buffers_if_necessary(rfsoc);
    lms7002m_enable_channel_power_controls(rfsoc);
    return OpStatus::Success;
}

static OpStatus RaiseGainsToOptimalLevel(CalibrationContext* ctx, const uint32_t bandwidth_Hz)
{
    lms7002m_context* rfsoc = ctx->rfsoc;
    const float txToneFreq = ctx->lo_diff + bandwidth_Hz;
    const int txToneBin = 256 * (2 * txToneFreq / ctx->sampleRate);

    const float rxMeasureOffset = bandwidth_Hz;
    float target_rssi = -10.0; // dbfs

    lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_LNA_RFE, 1);

    LMS7002M_LOG(
        rfsoc, lime_LogLevel_Debug, ">Gains setup: RxMeasure@%+g, TxTestSignal@%+g[%i],", rxMeasureOffset, txToneFreq, txToneBin);

    uint8_t cg_iamp = lms7002m_spi_read_csr(rfsoc, LMS7002M_CG_IAMP_TBB);
    int8_t g_lossmain = 15;
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_LOSS_MAIN_TXPAD_TRF, g_lossmain);

    OpStatus status = ctx->vspa->GenerateTxTone(true, txToneBin);
    if (status != OpStatus::Success)
    {
        printf("Failed to generate tone\n");
        return status;
    }

    float rssi = 0;
    uint8_t g_rxloopb_rfe = 2;
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_G_RXLOOPB_RFE, g_rxloopb_rfe);
    rssi = la9310_get_rssi(ctx, rxMeasureOffset);
    LMS7002M_LOG(rfsoc,
        lime_LogLevel_Debug,
        "Initial Rx gains:\tG_RXLOOPB: %2i, CG_IAMP: %2i LOSS_MAIN: %2i| %s",
        g_rxloopb_rfe,
        cg_iamp,
        g_lossmain,
        rssi_to_string(rssi));
    while (rssi < target_rssi)
    {
        g_rxloopb_rfe += 2;
        if (g_rxloopb_rfe > 15)
            break;
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_G_RXLOOPB_RFE, g_rxloopb_rfe);
        rssi = la9310_get_rssi(ctx, rxMeasureOffset);
        LMS7002M_LOG(rfsoc, lime_LogLevel_Debug, "G_RXLOOPB: %2i, | %s", g_rxloopb_rfe, rssi_to_string(rssi));
    }

    rssi = la9310_get_rssi(ctx, rxMeasureOffset);
    LMS7002M_LOG(rfsoc,
        lime_LogLevel_Debug,
        "Initial Tx gains:\tLOSS_MAIN_TXPAD: %2i, CG_IAMP: %2i | %s",
        g_lossmain,
        cg_iamp,
        rssi_to_string(rssi));
    while (rssi < target_rssi)
    {
        g_lossmain -= 1;
        if (g_lossmain < 0)
            break;
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_LOSS_MAIN_TXPAD_TRF, g_lossmain);
        rssi = la9310_get_rssi(ctx, rxMeasureOffset);
        LMS7002M_LOG(rfsoc, lime_LogLevel_Debug, "LOSS_MAIN_TXPAD: %2i, | %s", g_lossmain, rssi_to_string(rssi));
    }

    target_rssi = -17.0; // dBFS
    while (rssi < target_rssi)
    {
        const int step = cg_iamp > 57 ? 1 : 2;
        cg_iamp += step;

        if (cg_iamp > 63)
            break;
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_CG_IAMP_TBB, cg_iamp);
        rssi = la9310_get_rssi(ctx, rxMeasureOffset);
        LMS7002M_LOG(rfsoc, lime_LogLevel_Debug, "CG_IAMP_TBB: %2i, | %s", cg_iamp, rssi_to_string(rssi));
    }

    LMS7002M_LOG(rfsoc,
        lime_LogLevel_Debug,
        "Adjusted gains: G_RXLOOPB: %2i, CG_IAMP: %2i | %s",
        lms7002m_spi_read_csr(rfsoc, LMS7002M_G_RXLOOPB_RFE),
        lms7002m_spi_read_csr(rfsoc, LMS7002M_CG_IAMP_TBB),
        rssi_to_string(rssi));

    ctx->vspa->GenerateTxTone(false);

    lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_LNA_RFE, 0);

    const float expectedRSSI_level = -30.0;
    const float failureRSSI_level = -50.0;
    if (rssi < expectedRSSI_level)
    {
        char expectedRSSI_string[32];
        strcpy(expectedRSSI_string, rssi_to_string(expectedRSSI_level));
        LMS7002M_LOG(rfsoc,
            rssi > failureRSSI_level ? lime_LogLevel_Warning : lime_LogLevel_Error,
            "%s: Low calibration test signal level %s, expected to be more than %s."
            " Calibration results might be impacted. Try re-calibrating or adjusting the RX gains.",
            __func__,
            rssi_to_string(rssi),
            expectedRSSI_string);
    }
    return rssi > failureRSSI_level ? OpStatus::Success : OpStatus::Error;
}

static OpStatus la9310_calibrate_iq_imbalance(CalibrationContext* ctx, bool isTx, double calibrationRF)
{
    float txToneFreq;
    float rxMeasureOffset;
    if (isTx)
    {
        txToneFreq = calibrationRF;
        rxMeasureOffset = ctx->lo_diff - txToneFreq;
    }
    else
    {
        txToneFreq = ctx->lo_diff + calibrationRF;
        rxMeasureOffset = -calibrationRF;
    }
    const int txToneBin = 256 * (2 * txToneFreq / ctx->sampleRate);
    lms7002m_context* rfsoc = ctx->rfsoc;

    const char* const dirName = isTx ? "Tx" : "Rx";
    LMS7002M_LOG(rfsoc,
        lime_LogLevel_Debug,
        ">%s QEC setup: RxMeasure@%+g, TxTestSignal@%+g[%i]",
        dirName,
        rxMeasureOffset,
        txToneFreq,
        txToneBin);

    std::shared_ptr<IQuadratureErrorCorrector> iqcorr = isTx ? ctx->vspa->GetTxQEC() : ctx->vspa->GetRxQEC();
    iqcorr->SetImbalance(0, 0);

    lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_LNA_RFE, 1);

    OpStatus status;
    if (isTx)
        status = ctx->vspa->StartTxTone(true, txToneBin);
    else
        status = ctx->vspa->GenerateTxTone(true, txToneBin);
    if (status != OpStatus::Success)
    {
        printf("Failed to generate tone\n");
        return status;
    }

    float lastPhase = 0;
    float lastGain = 0;

    LMS7002M_LOG(
        ctx->rfsoc, lime_LogLevel_Verbose, "QEC gain range [%g:%g]", iqcorr->GetGainRange().min, iqcorr->GetGainRange().max);
    lastGain = bisection_find_min_rssi2(
        iqcorr->GetGainRange(),
        [&iqcorr, &lastPhase](float value) { iqcorr->SetImbalance(value, lastPhase); },
        [&ctx, rxMeasureOffset]() { return la9310_get_rssi(ctx, rxMeasureOffset); });
    LMS7002M_LOG(
        ctx->rfsoc, lime_LogLevel_Verbose, "QEC gain: %g db, (RSSI: %f dBFS)", lastGain, la9310_get_rssi(ctx, rxMeasureOffset));

    LMS7002M_LOG(
        ctx->rfsoc, lime_LogLevel_Verbose, "QEC phase range [%g:%g]", iqcorr->GetPhaseRange().min, iqcorr->GetPhaseRange().max);
    lastPhase = bisection_find_min_rssi2(
        iqcorr->GetPhaseRange(),
        [&iqcorr, &lastGain](float value) { iqcorr->SetImbalance(lastGain, value); },
        [&ctx, rxMeasureOffset]() { return la9310_get_rssi(ctx, rxMeasureOffset); });
    LMS7002M_LOG(
        ctx->rfsoc, lime_LogLevel_Verbose, "QEC phase: %g, (RSSI: %f dBFS)", lastPhase, la9310_get_rssi(ctx, rxMeasureOffset));

    // repeat gain search again, just around the last found value
    auto gainLocalRange = iqcorr->GetGainRange();
    float gainSpan = (gainLocalRange.max - gainLocalRange.min) / 16;
    gainLocalRange.min = std::clamp(lastGain - gainSpan / 2, gainLocalRange.min, gainLocalRange.max);
    gainLocalRange.max = std::clamp(lastGain + gainSpan / 2, gainLocalRange.min, gainLocalRange.max);
    LMS7002M_LOG(ctx->rfsoc, lime_LogLevel_Verbose, "QEC gain range [%g:%g]", gainLocalRange.min, gainLocalRange.max);
    lastGain = bisection_find_min_rssi2(
        gainLocalRange,
        [&iqcorr, &lastPhase](float value) { iqcorr->SetImbalance(value, lastPhase); },
        [&ctx, rxMeasureOffset]() { return la9310_get_rssi(ctx, rxMeasureOffset); });
    LMS7002M_LOG(
        ctx->rfsoc, lime_LogLevel_Verbose, "QEC gain: %g db, (RSSI: %f dBFS)", lastGain, la9310_get_rssi(ctx, rxMeasureOffset));

    // repeat phase search again, just around the last found value
    auto phaseLocalRange = iqcorr->GetPhaseRange();
    float phaseSpan = (phaseLocalRange.max - phaseLocalRange.min) / 16;
    phaseLocalRange.min = std::clamp(lastPhase - phaseSpan / 2, phaseLocalRange.min, phaseLocalRange.max);
    phaseLocalRange.max = std::clamp(lastPhase + phaseSpan / 2, phaseLocalRange.min, phaseLocalRange.max);
    LMS7002M_LOG(ctx->rfsoc, lime_LogLevel_Verbose, "QEC phase range [%g:%g]", phaseLocalRange.min, phaseLocalRange.max);
    lastPhase = bisection_find_min_rssi2(
        phaseLocalRange,
        [&iqcorr, &lastGain](float value) { iqcorr->SetImbalance(lastGain, value); },
        [&ctx, rxMeasureOffset]() { return la9310_get_rssi(ctx, rxMeasureOffset); });
    LMS7002M_LOG(
        ctx->rfsoc, lime_LogLevel_Verbose, "QEC phase: %g, (RSSI: %f dBFS)", lastPhase, la9310_get_rssi(ctx, rxMeasureOffset));

    iqcorr->SetImbalance(lastGain, lastPhase);

    const float rssi = la9310_get_rssi(ctx, rxMeasureOffset);
    LMS7002M_LOG(ctx->rfsoc, lime_LogLevel_Verbose, "QEC gain: %g db, phase: %g deg, (RSSI: %f dBFS)", lastGain, lastPhase, rssi);

    lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_LNA_RFE, 0);

    if (isTx)
        status = ctx->vspa->StartTxTone(false, txToneBin);
    else
        status = ctx->vspa->GenerateTxTone(false);
    return status;
}

static void SetupInternalLoopbackForRx(lms7002m_context* rfsoc)
{
    spi_batch_t batch;
    spi_batch_init(&batch);

    if (lms7002m_spi_read_csr(rfsoc, LMS7002M_SEL_PATH_RFE) == 2)
    {
        spi_batch_modify_csr(&batch, LMS7002M_PD_RLOOPB_2_RFE, 0);
        spi_batch_modify_csr(&batch, LMS7002M_EN_INSHSW_LB2_RFE, 0);
    }
    else
    {
        spi_batch_modify_csr(&batch, LMS7002M_PD_RLOOPB_1_RFE, 0);
        spi_batch_modify_csr(&batch, LMS7002M_EN_INSHSW_LB1_RFE, 0);
    }
    spi_batch_modify_csr(&batch, LMS7002M_EN_LOOPB_TXPAD_TRF, 1);
    lms7002m_spi_batch_flush(&batch, rfsoc);
}

OpStatus LimeSDR_Micro::CalibrateRx()
{

    CalibrationContext context;
    context.vspa = &la9310->vspa;
    context.sampleRate = GetSampleRate(0, TRXDir::Rx, 0, nullptr);
    context.vspa->Initialize();
    context.vspa->StopRx();
    context.rfsoc = mLMSChips[0]->mC_impl;
    lms7002m_context* rfsoc = context.rfsoc;
    const uint16_t x0020val = lms7002m_spi_read(rfsoc, 0x0020); //remember used channel
    const uint16_t channel = (x0020val & 0x3) == 1 ? 0 : 1;
    context.vspa->SelectRxChannel(channel == 0 ? 3 : 1);
    context.vspa->Setup(1, 0, context.sampleRate * 4);
    context.vspa->StopTx();
    context.phytimer = &la9310->phytimer;
    context.lo_diff = 0;

    context.vspa->SetupRx(0, 1024 * 1024, 16384 * sizeof(complex16_t) * 64);

    const bool dcOnly = false;
    const uint32_t bandwidthRF = 5e6;
    const double calibrationRF = bandwidthRF / calibUserBwDivider;

    LMS7002M_LOG(rfsoc,
        lime_LogLevel_Verbose,
        "Rx calibrate ch.%s @ %lu Hz, BW: %u Hz, RF input: %s, PGA: %i, LNA: %i, TIA: %i",
        (x0020val & 0x3) == 1 ? "A" : "B",
        lms7002m_get_frequency_sx(rfsoc, false),
        bandwidthRF,
        get_lna_name(lms7002m_spi_read_csr(rfsoc, LMS7002M_SEL_PATH_RFE)),
        lms7002m_spi_read_csr(rfsoc, LMS7002M_G_PGA_RBB),
        lms7002m_spi_read_csr(rfsoc, LMS7002M_G_LNA_RFE),
        lms7002m_spi_read_csr(rfsoc, LMS7002M_G_TIA_RFE));
    lms7002m_save_chip_state(rfsoc, false);

    OpStatus status = lms7002m_calibrate_rx_setup(&context, calibrationRF);
    if (status != OpStatus::Success)
        goto RxCalibrationEndStage;
    limesdrmicro_calibrate_rx_dc(&context, channel);

    if (dcOnly)
        goto RxCalibrationEndStage;

    SetupInternalLoopbackForRx(rfsoc);

    status = RaiseGainsToOptimalLevel(&context, calibrationRF);
    if (status != OpStatus::Success)
        goto RxCalibrationEndStage;

    status = la9310_calibrate_iq_imbalance(&context, false, calibrationRF);

RxCalibrationEndStage : {
    lms7002m_save_chip_state(rfsoc, true);
    lms7002m_spi_write(rfsoc, 0x0020, x0020val);
    if (status != OpStatus::Success)
    {
        LMS7002M_LOG(rfsoc, lime_LogLevel_Debug, "%s", "Rx calibration failed");
        return status;
    }
}
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_DCMODE, 1);
    if (x0020val & 0x1)
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_DCDAC_RXA, 0);
    else
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_DCDAC_RXB, 0);
    LMS7002M_LOG(rfsoc, lime_LogLevel_Verbose, "%s", "Rx calibration finished");
    return ResultToStatus(lime_Result_Success);
}

static OpStatus SetupInternalLoopbackForTx(lms7002m_context* rfsoc)
{
    // Select Rx LNA for loopback
    const uint8_t sel_band1_2_trf = lms7002m_spi_read_bits(rfsoc, 0x0103, 11, 10);
    if (sel_band1_2_trf != 0x1 && sel_band1_2_trf != 0x2) //BAND1
    {
        LMS7002M_LOG(rfsoc, lime_LogLevel_Error, "%s", "Tx Calibration: band not selected");
        return OpStatus::Error;
    }

    spi_batch_t batch;
    spi_batch_init(&batch);
    spi_batch_modify_csr(&batch, LMS7002M_SEL_PATH_RFE, sel_band1_2_trf + 1);
    spi_batch_modify(&batch, 0x010C, 6, 5, sel_band1_2_trf ^ 0x3);
    spi_batch_modify(&batch, 0x010D, 4, 3, sel_band1_2_trf ^ 0x3);
    spi_batch_modify_csr(&batch, LMS7002M_EN_LOOPB_TXPAD_TRF, 1);
    lms7002m_spi_batch_flush(&batch, rfsoc);

    return OpStatus::Success;
}

static OpStatus lms7002m_calibrate_tx_setup(CalibrationContext* ctx, uint32_t bandwidthRF)
{
    lms7002m_context* rfsoc = ctx->rfsoc;
    const uint16_t x0020val = lms7002m_spi_read(rfsoc, 0x0020);

    lms7002m_spi_modify_csr(rfsoc, LMS7002M_EN_DIR_TRF, 0);
    // use external ADC/DAC
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_OSW_PGA_RBB, 1);
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_TSTIN_TBB, 3);

    SetupInternalLoopbackForTx(rfsoc);

    lms7002m_spi_modify_csr(rfsoc, LMS7002M_MAC, 2);

    lime_Result status = lime_Result_Success;
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_LOCH_T2RBUF, 1);

    // SXT
    const double sxt_lo = lms7002m_get_frequency_sx(rfsoc, true);

    // SXR
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_MAC, 1);
    ctx->lo_diff = (ctx->sampleRate / 8);
    const double sxr_lo = sxt_lo - ctx->lo_diff;
    status = lms7002m_set_frequency_sx(rfsoc, false, sxr_lo);
    LMS7002M_LOG(rfsoc, lime_LogLevel_Debug, "RxLO@%g, TxLO@%g", sxr_lo, sxt_lo);

    lms7002m_spi_write(rfsoc, 0x0020, x0020val);
    if (status != lime_Result_Success)
        return OpStatus::Error;

    // modifications when calibrating channel B
    lms7002m_enable_mimo_buffers_if_necessary(rfsoc);
    lms7002m_enable_channel_power_controls(rfsoc);
    return OpStatus::Success;
}

static OpStatus RaiseRxGainToOptimalLevel(CalibrationContext* ctx, double txToneFreq)
{
    const int txToneBin = 256 * (2 * txToneFreq / ctx->sampleRate);
    const float rxMeasureOffset = ctx->lo_diff + txToneFreq;
    const float saturationLevel = -12.86; // dBFS
    lms7002m_context* rfsoc = ctx->rfsoc;

    lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_LNA_RFE, 1);

    OpStatus status = ctx->vspa->GenerateTxTone(true, txToneBin);
    if (status != OpStatus::Success)
        return status;

    uint8_t g_pga = lms7002m_spi_read_csr(rfsoc, LMS7002M_G_PGA_RBB);
    uint8_t g_rfe = lms7002m_spi_read_csr(rfsoc, LMS7002M_G_RXLOOPB_RFE);

    float rssi = la9310_get_rssi(ctx, rxMeasureOffset);
    LMS7002M_LOG(rfsoc, lime_LogLevel_Debug, "Receiver saturation search, target level: %s", rssi_to_string(saturationLevel));
    LMS7002M_LOG(rfsoc, lime_LogLevel_Debug, "initial  PGA: %2i, RXLOOPB: %2i, %s", g_pga, g_rfe, rssi_to_string(rssi));

    while (rssi < saturationLevel)
    {
        if (g_rfe < 15)
            ++g_rfe;
        else
            break;

        lms7002m_spi_modify_csr(rfsoc, LMS7002M_G_RXLOOPB_RFE, g_rfe);

        rssi = la9310_get_rssi(ctx, rxMeasureOffset);
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
            rssi = la9310_get_rssi(ctx, rxMeasureOffset);

            printf_dbg_log("g_pga: %i, rssi: %s\n", g_pga, rssi_to_string(rssi));

            if (rssi - rssi_prev < 0.9)
                break;

            rssi_prev = rssi;
        }
    }
    LMS7002M_LOG(rfsoc,
        lime_LogLevel_Debug,
        "adjusted PGA: %2i, RXLOOPB: %2i, %s",
        lms7002m_spi_read_csr(rfsoc, LMS7002M_G_PGA_RBB),
        g_rfe,
        rssi_to_string(rssi));

    ctx->vspa->GenerateTxTone(false);

    lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_LNA_RFE, 0);

    const float expectedRSSI_level = -30.0;
    const float failureRSSI_level = -50;
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
            return OpStatus::Error;
    }
    return OpStatus::Success;
}

static OpStatus limesdrmicro_calibrate_tx_dc(CalibrationContext* ctx, uint16_t channel, double txToneFreq)
{
    lms7002m_context* rfsoc = ctx->rfsoc;
    LMS7002M_LOG(rfsoc, lime_LogLevel_Debug, ">%s", __func__);

    spi_batch_t batch;
    spi_batch_init(&batch);
    spi_batch_modify_csr(&batch, LMS7002M_PD_LNA_RFE, 1);
    spi_batch_modify_csr(&batch, LMS7002M_EN_G_TRF, 1); // enable transmitter
    spi_batch_modify_csr(&batch, LMS7002M_DCMODE, 1); // Rx DC correctors selection
    lms7002m_spi_batch_flush(&batch, rfsoc);

    std::shared_ptr<IDCCorrector> digitalDC = ctx->vspa->GetTxDCCorrector();
    digitalDC->SetDCOffset(complex16_t(0, 0));

    std::unique_ptr<IDCCorrector> analogDC = std::make_unique<LMS7002M_DC>(rfsoc, TRXDir::Tx, channel);
    analogDC->Enabled(true);
    int16_t dci = 0, dcq = 0;
    analogDC->SetDCOffset(complex16_t(dci, dcq));

    const int txToneBin = 256 * (2 * txToneFreq / ctx->sampleRate);
    const float rxMeasureOffset = ctx->lo_diff;
    OpStatus status = ctx->vspa->GenerateTxTone(true, txToneBin);
    if (status != OpStatus::Success)
    {
        printf("Failed to generate tone\n");
        return status;
    }

    // analog
    for (int i = 0; i < 1; ++i)
    {
        dci = bisection_find_min_rssi2(
            analogDC->GetRange(),
            [&analogDC](float value) { analogDC->SetDCI(value); },
            [&ctx, rxMeasureOffset]() { return la9310_get_rssi(ctx, rxMeasureOffset); });

        dcq = bisection_find_min_rssi2(
            analogDC->GetRange(),
            [&analogDC](float value) { analogDC->SetDCQ(value); },
            [&ctx, rxMeasureOffset]() { return la9310_get_rssi(ctx, rxMeasureOffset); });
    }

    LMS7002M_LOG(rfsoc,
        lime_LogLevel_Verbose,
        "TxDC offset (analog) I:%3i, Q:%3i, %s",
        dci,
        dcq,
        rssi_to_string(la9310_get_rssi(ctx, rxMeasureOffset)));

    // // digital
    // {
    //     dci = bisection_find_min_rssi2(digitalDC->GetRange(),
    //         [&digitalDC](float value) {digitalDC->SetDCI(value); },
    //         [&ctx, rxMeasureOffset]() { return la9310_get_rssi(ctx, rxMeasureOffset); }
    //     );

    //     dcq = bisection_find_min_rssi2(digitalDC->GetRange(),
    //         [&digitalDC](float value) {digitalDC->SetDCQ(value); },
    //         [&ctx, rxMeasureOffset]() { return la9310_get_rssi(ctx, rxMeasureOffset); }
    //     );

    //     LMS7002M_LOG(rfsoc,
    //         lime_LogLevel_Verbose,
    //         "TxDC offset (digital) I:%3i, Q:%3i, %s",
    //         dci,
    //         dcq,
    //         rssi_to_string(la9310_get_rssi(ctx, rxMeasureOffset)));
    // }
    status = ctx->vspa->GenerateTxTone(false);
    spi_batch_modify_csr(&batch, LMS7002M_PD_LNA_RFE, 0);
    return OpStatus::Success;
}

OpStatus LimeSDR_Micro::CalibrateTx()
{
    CalibrationContext context;
    context.vspa = &la9310->vspa;
    context.sampleRate = GetSampleRate(0, TRXDir::Tx, 0, nullptr);
    context.vspa->Initialize();
    context.vspa->StopRx();
    context.vspa->Setup(1, 0, context.sampleRate * 4);
    context.vspa->StopTx();
    context.phytimer = &la9310->phytimer;
    context.rfsoc = mLMSChips[0]->mC_impl;
    lms7002m_context* rfsoc = context.rfsoc;
    const uint16_t x0020val = lms7002m_spi_read(rfsoc, 0x0020); //remember used channel
    const uint16_t channel = (x0020val & 0x3) == 1 ? 0 : 1;
    context.vspa->SelectRxChannel(channel == 0 ? 3 : 1);

    context.vspa->GetTxDCCorrector()->SetDCOffset(complex16_t(0, 0));
    context.vspa->GetTxQEC()->SetImbalance(0, 0);

    context.vspa->SetupRx(0, 1024 * 1024, 16384 * sizeof(complex16_t) * 64);

    const uint32_t bandwidthRF = 5e6;
    const double calibrationRF = bandwidthRF / calibUserBwDivider;

    LMS7002M_LOG(rfsoc,
        lime_LogLevel_Verbose,
        "Tx calibrate ch.%s , BW: %u Hz, RF output: %s, LOSS_MAIN:%i IAMP: %i",
        (x0020val & 3) == 0x1 ? "A" : "B",
        bandwidthRF,
        lms7002m_spi_read_csr(rfsoc, LMS7002M_SEL_BAND1_TRF) == 1 ? "BAND1" : "BAND2",
        lms7002m_spi_read_csr(rfsoc, LMS7002M_LOSS_MAIN_TXPAD_TRF),
        lms7002m_spi_read_csr(rfsoc, LMS7002M_CG_IAMP_TBB));

    lms7002m_save_chip_state(rfsoc, false);
    OpStatus status = lms7002m_calibrate_tx_setup(&context, calibrationRF);
    if (status != OpStatus::Success)
        goto TxCalibrationEnd; //go to ending stage to restore registers

    status = RaiseRxGainToOptimalLevel(&context, calibrationRF);
    if (status != OpStatus::Success)
        goto TxCalibrationEnd;

    limesdrmicro_calibrate_rx_dc(&context, channel);

    limesdrmicro_calibrate_tx_dc(&context, channel, calibrationRF);

    status = la9310_calibrate_iq_imbalance(&context, true, calibrationRF);
TxCalibrationEnd : {
    lms7002m_save_chip_state(rfsoc, true);
    lms7002m_spi_write(rfsoc, 0x0020, x0020val);
    if (status != OpStatus::Success)
    {
        LMS7002M_LOG(rfsoc, lime_LogLevel_Debug, "%s", "Tx calibration failed");
        return status;
    }

    lms7002m_spi_modify_csr(rfsoc, LMS7002M_DCMODE, 1);
    if ((x0020val & 1) == 1)
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_DCDAC_TXA, 0);
    else
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_PD_DCDAC_TXB, 0);
    return OpStatus::Success;
}
}

} // namespace lime