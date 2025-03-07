#include "pll.h"

#include <stdint.h>
#include <stddef.h>
#include "privates.h"

#include "limesuiteng/embedded/result.h"
#include "lms7002m_context.h"
#include "csr_data.h"
#include "spi.h"
#include "poly.h"
// #include "expect.h"
#include "../../src/gnuPlotPipe.h"
#include <chrono>
#include <sstream>

typedef enum {
    COMPARATORS_CSW_TOO_LOW = 0x0,
    COMPARATORS_INVALID = 0x1,
    COMPARATORS_CSW_GOOD = 0x2,
    COMPARATORS_CSW_TOO_HIGH = 0x3
} Comparators;

// SXR/SXT VCO data
static const char* const vcoNames[] = { "VCOL", "VCOM", "VCOH" };
static const uint64_t VCO_min_frequency[3] = { 3800000000, 4961000000, 6306000000 };
static const uint64_t VCO_max_frequency[3] = { 5222000000, 6754000000, 7714000000 };

lime_Result lms7002m_measure_vco_approximation(struct lms7002m_context* self, uint8_t sel_vco);

struct VCOStatistics {
    struct Samples {
        double freqHz;
        uint8_t cswRange[2];
    };
    double frequencyRange[2];
    double polynom[3];
    std::vector<Samples> measurements;
};

struct VCO_CSW_Data {
    uint16_t vcomin_MHz;
    uint16_t vcomax_MHz;
    double polynom[3];
};

// if expected VCO frequency is provided, guess the approximate CSW value
uint8_t ApproximateCSW_VCO(uint8_t sel_vco, uint64_t expectedVCO_Hz)
{
    // const float pCoeffs[3][3] = {
    //     {-1993, 0.78, -6.74e-5},
    //     {-2021, 0.591, -3.77e-5},
    //     {-2826, 0.662, -3.44e-5}
    //     };
    VCO_CSW_Data vcol[] = {
        { 3921, 4347, -1388.630127, 0.511491, -0.000037 },
        { 4349, 5098, -1724.790405, 0.673785, -0.000057 },
    };
    VCO_CSW_Data vcom[] = {
        { 5067, 5697, -1691.604492, 0.489033, -0.000030 },
        { 5700, 6647, -1789.207153, 0.527522, -0.000033 },
    };
    VCO_CSW_Data vcoh[] = {
        { 6420, 6929, -2161.940674, 0.467022, -0.000020 },
        { 6931, 7597, -839.999146, 0.120660, 0.000003 },
    };

    VCO_CSW_Data* ranges;
    int rangesCount = 0;

    if (sel_vco == 0)
    {
        ranges = vcol;
        rangesCount = 2;
    }
    else if (sel_vco == 1)
    {
        ranges = vcom;
        rangesCount = 2;
    }
    else
    {
        ranges = vcoh;
        rangesCount = 2;
    }

    // const float pCoeffs[3][3] = {
    //     {-1832.155762, 0.714695, -0.00006},
    //     {-437.669312, 0.162482, -0.00006},
    //     {-2826, 0.662, -3.44e-5}
    //     };

    float vcoMHz = (expectedVCO_Hz) / 1e6;
    for (int i = 0; i < rangesCount; ++i)
    {
        if (vcoMHz >= ranges[i].vcomin_MHz && vcoMHz <= ranges[i].vcomax_MHz)
        {
            int16_t csw = ranges[i].polynom[0] + (vcoMHz) * (ranges[i].polynom[1]) + pow(vcoMHz, 2) * ranges[i].polynom[2];
            return csw;
        }
    }
    // printf("can't aprox: %f\n", vcoMHz);
    return 128;
}

static Comparators ReadComparators(struct lms7002m_context* self, struct lms7002m_csr comparator)
{
    // comparators might need some settling time, so just read the same register multiple times, to create delay
    const uint32_t mosi[2] = { LMS7002M_VCO_CMPHO.address << 16, LMS7002M_VCO_CMPHO.address << 16 };
    uint32_t miso[2];
    self->cnt_spi_rd += 2;
    ++self->cnt_spi;
    lime_Result status = self->hooks.spi16_transact(mosi, miso, 2, self->hooks.spi16_userData);
    return csr_get_bits(miso[1], comparator); // use last value
}

/// Just finds the CSW_VCO tune value
/// @param vcoIndex Which VCO to tune
/// @param reg0121_value [Optional] Initial 0x0121 register value to avoid unnecessary read
lime_Result lms7002m_tune_vco_sx_fast(struct lms7002m_context* self, uint16_t* reg0121_value)
{
    // EXPECT(self, lms7002m_spi_read_csr(self, LMS7002M_PD_VCO) == 0); // ensure VCO is not powered down
    // EXPECT(self, lms7002m_spi_read_csr(self, LMS7002M_PD_VCO_COMP) == 0); // ensure VCO Comparators are not powered down

    const uint16_t addr = LMS7002M_CSW_VCO.address;

    // conjunction of LMS7002M_VCO_CMPHO and LMS7002M_VCO_CMPLO to treat as single value
    const struct lms7002m_csr VCO_CMP = { LMS7002M_VCO_CMPHO.address, 13, 12 };

    // only the CSW is being changed, read register once to avoid repeated reading overhead
    uint16_t reg0121_base;
    if (reg0121_value)
    {
        // register hint given, assume that is the actual value in the chip
        // search starts from the value already set.
        reg0121_base = *reg0121_value;
    }
    else
    {
        // no hint given, readback the register and do a full search
        reg0121_base = lms7002m_spi_read(self, addr);

        // Search could start with any currently preexisting CSW_VCO value, but just in case set it to known value to be deterministic
        reg0121_base = csr_set_bits(reg0121_base, LMS7002M_CSW_VCO, 0);
        lms7002m_spi_write(self, addr, reg0121_base);
    }

    Comparators cmp = ReadComparators(self, VCO_CMP); //lms7002m_spi_read_csr(self, VCO_CMP);
    if (cmp == COMPARATORS_CSW_GOOD)
    {
        // printf("Cache hit: \n");
        return lime_Result_Success;
    }

    uint8_t csw_high = 255;
    uint8_t csw_low = 0;
    // initial check tell which direction the search should go
    uint8_t csw_vco = csr_get_bits(reg0121_base, LMS7002M_CSW_VCO);
    if (cmp == COMPARATORS_CSW_TOO_LOW)
        csw_low = csw_vco;
    else
        csw_high = csw_vco;

    reg0121_base = csr_clear_bits(reg0121_base, LMS7002M_CSW_VCO);

    // search is done until any value is found that achieves PLL lock.
    // usually there is a range of values that achieve lock.
    // ideally one should be selected from middle of that range for better reliability, but extra measurements takes more time.

    // In theory binary search should be enough, but in reality bits are representing capacitor banks
    // so there is a chance that the capacitance of one big capacitor (i.e 0x80) is less than many small ones (i.e 0x7F)
    // creating situation where PLL lock is achieved with values in two ranges [56-62] and [64], the 63 is unable to lock, and 64 being on the edge/unstable.
    do
    {
        uint8_t csw_mid = csw_low + (csw_high - csw_low) / 2;
        lms7002m_spi_write(self, addr, csr_set_bits(reg0121_base, LMS7002M_CSW_VCO, csw_mid)); //
        cmp = ReadComparators(self, VCO_CMP); //lms7002m_spi_read_csr(self, VCO_CMP);
        // printf("CSW: %i - cmp:%i\n", csw_mid, cmp);
        if (cmp == COMPARATORS_CSW_GOOD)
        {
            return lime_Result_Success;
        }

        if (cmp == COMPARATORS_CSW_TOO_HIGH)
            csw_high = csw_mid - 1;
        else
            csw_low = csw_mid + 1;
    } while (csw_low < csw_high);

    return lime_Result_Error;
}

lime_Result lms7002m_calculate_pll_coefficients(struct pll_coefficients* pll, uint64_t refClk_Hz, uint64_t LO_freq_hz)
{
    bool canDeliverFrequency = false;
    uint64_t VCOfreq;

    // use VCOM last, it seems to have the narrowest PLL locking ranges
    const uint8_t preferred_vco_order[3] = { 2, 0, 1 };

    // check is any of the VCO can provide needed frequency
    for (uint8_t i = 0; i < 3 && !canDeliverFrequency; ++i)
    {
        const uint8_t sel_vco = preferred_vco_order[i];
        // div_loch value 7 is not allowed
        for (int8_t div_loch = 6; div_loch >= 0; --div_loch)
        {
            VCOfreq = (1 << (div_loch + 1)) * (uint64_t)LO_freq_hz;
            if ((VCOfreq >= VCO_min_frequency[sel_vco]) && (VCOfreq <= VCO_max_frequency[sel_vco]))
            {
                pll->div_loch = div_loch;
                pll->sel_vco = sel_vco;

                canDeliverFrequency = true;
                break;
            }
        }
    }

    if (!canDeliverFrequency)
        return lime_Result_Error;

    const uint64_t m_dThrF = 5500000000; // VCO frequency threshold to enable additional divider
    const uint8_t additional_divider = (VCOfreq > m_dThrF) ? 1 : 0;
    const uint64_t divider = refClk_Hz << additional_divider;

    uint16_t integerPart = VCOfreq / divider;
    // "Fixed point number" division, take only the fraction part
    uint32_t fractionalPart = ((VCOfreq - integerPart * divider) << 20) / divider;
    integerPart -= 4; // value written to register is value-4

    pll->integer = integerPart;
    pll->fractional = fractionalPart;
    pll->en_div2 = additional_divider;

    uint32_t approx_csw =
        (VCO_max_frequency[pll->sel_vco] - VCO_min_frequency[pll->sel_vco]) / (VCOfreq - VCO_min_frequency[pll->sel_vco]);
    approx_csw = ApproximateCSW_VCO(pll->sel_vco, VCOfreq);
    // printf("VCO %i Guess CSW :%i\n", pll->sel_vco, approx_csw);
    pll->csw_vco = approx_csw; // could be approximated

    return lime_Result_Success;
}

static lime_Result lms7002m_write_pll_registers(
    struct lms7002m_context* self, const struct pll_coefficients* pll, uint16_t* reg0121)
{
    uint32_t mosi[5];
    mosi[0] = LMS7002M_DIV_LOCH.address << 16;
    mosi[1] = LMS7002M_EN_DIV2_DIVPROG.address << 16;
    mosi[2] = LMS7002M_SEL_VCO.address << 16; // also contains CSW_VCO

    uint32_t miso[3];
    ++self->cnt_spi;
    lime_Result status = self->hooks.spi16_transact(mosi, miso, 3, self->hooks.spi16_userData);
    self->cnt_spi_rd += 2;
    if (status != lime_Result_Success)
        return status;

    // omit values that don't need to change
    uint8_t mosi_cnt = 0;
    if (csr_get_bits(miso[0], LMS7002M_DIV_LOCH) != pll->div_loch)
        mosi[mosi_cnt++] = (1 << 31) | (LMS7002M_DIV_LOCH.address) << 16 | csr_set_bits(miso[0], LMS7002M_DIV_LOCH, pll->div_loch);

    if (csr_get_bits(miso[1], LMS7002M_EN_DIV2_DIVPROG) != pll->en_div2)
        mosi[mosi_cnt++] =
            (1 << 31) | (LMS7002M_EN_DIV2_DIVPROG.address) << 16 | csr_set_bits(miso[1], LMS7002M_EN_DIV2_DIVPROG, pll->en_div2);

    mosi[mosi_cnt++] = 0x811E0000 | (pll->integer << 4) | ((pll->fractional >> 16) & 0xF);
    mosi[mosi_cnt++] = 0x811D0000 | (pll->fractional & 0xFFFF);

    mosi[mosi_cnt] = (1 << 31) | (LMS7002M_SEL_VCO.address << 16) | miso[2];
    mosi[mosi_cnt] = csr_set_bits(mosi[mosi_cnt], LMS7002M_SEL_VCO, pll->sel_vco);
    mosi[mosi_cnt] = csr_set_bits(mosi[mosi_cnt], LMS7002M_CSW_VCO, pll->csw_vco);

    if (reg0121)
        *reg0121 = mosi[mosi_cnt] & 0xFFFF;
    ++mosi_cnt;

    self->cnt_spi_wr += mosi_cnt;
    ++self->cnt_spi;
    return self->hooks.spi16_transact(mosi, NULL, mosi_cnt, self->hooks.spi16_userData) == 0 ? lime_Result_Success
                                                                                             : lime_Result_IOFailure;
}

lime_Result lms7002m_set_lo_sx(lms7002m_context* self, uint64_t LO_freq_hz)
{
    const uint32_t refClk_Hz = lms7002m_get_reference_clock(self);
    if (refClk_Hz == 0)
        return lime_Result_Error;

    // VCOStatistics stats;
    // for (int i=0; i<3; ++i)
    // int i = 0;
    // {
    //     lms7002m_spi_modify_csr(self, LMS7002M_SEL_VCO, i);
    //     lms7002m_measure_vco_approximation(self, i);
    // }
    // return -1;

    struct pll_coefficients pll;
    lime_Result status = lms7002m_calculate_pll_coefficients(&pll, refClk_Hz, LO_freq_hz);
    if (status != lime_Result_Success)
    {
        LMS7002M_LOG(self, lime_LogLevel_Error, "%s: LO(%lu Hz) - VCO cannot deliver frequency.", __func__, LO_freq_hz);
        return lime_Result_Error;
    }

    uint16_t reg0121 = 0;
    status = lms7002m_write_pll_registers(self, &pll, &reg0121);
    if (status != lime_Result_Success)
        return status;

    int16_t ict_vco = -1;
    do // if initial tune fails, attempt again with modified bias current
    {
        LMS7002M_LOG(self, lime_LogLevel_Debug, "Tuning %s (ICT_VCO:%d):", vcoNames[pll.sel_vco], ict_vco);
        status = lms7002m_tune_vco_sx_fast(self, &reg0121);
        if (status == lime_Result_Success)
            return status;

        if (ict_vco < 0) // read the initial value only once, if needed
            ict_vco = lms7002m_spi_read_csr(self, LMS7002M_ICT_VCO);

        if (ict_vco == 255)
            break;
        ict_vco = ict_vco + 32 > 255 ? 255 : ict_vco + 32; // retry with higher bias current
        lms7002m_spi_modify_csr(self, LMS7002M_ICT_VCO, ict_vco);
    } while (ict_vco <= 255);

    LMS7002M_LOG(self, lime_LogLevel_Error, "%s: LO(%lu Hz) - tune failed", __func__, LO_freq_hz);
    return lime_Result_Error;
}

static lime_Result ProduceApproximation(const uint64_t* freqs, const uint8_t* csw, size_t count, VCO_CSW_Data* approx)
{
    if (count == 0)
        return lime_Result_InvalidValue;

    std::vector<float> coeffs;
    const int order = 2;
    std::vector<float> freq(count);
    std::vector<float> csws(count);
    for (int i = 0; i < count; ++i)
    {
        freq[i] = freqs[i] / 1000000;
        csws[i] = csw[i];
    }

    PolynomialRegression<float> regression;
    if (regression.fitIt(freq, csws, order, coeffs))
    {
        approx->vcomin_MHz = freq[0];
        approx->vcomax_MHz = freq[count - 1];
        approx->polynom[0] = coeffs[0];
        approx->polynom[1] = coeffs[1];
        approx->polynom[2] = coeffs[2];
        return lime_Result_Success;
    }
    return lime_Result_Error;
}

lime_Result lms7002m_measure_calculate_csw_vco_approximation(
    struct lms7002m_context* self, int sel_vco, uint64_t vcomin, uint64_t vcomax)
{
    const double m_dThrF = 5500e6; //threshold to enable additional divider
    uint16_t addrCMP = LMS7002M_VCO_CMPHO.address;

    const int maxSteps = 512;
    const uint64_t freqStep = (vcomax - vcomin) / maxSteps;
    const uint64_t refClk_Hz = lms7002m_get_reference_clock(self);
    int stepsDone = 0;
    uint64_t freqs[maxSteps];
    uint8_t csw_avg[maxSteps];
    uint8_t acsw_max[maxSteps];
    uint8_t acsw_min[maxSteps];

    // VCO constrol has only 256 available values
    for (double freq = vcomin; freq <= vcomax; freq += freqStep)
    {
        // calculate params for desired VCO
        bool isValidFreq = false;
        int8_t div_loch;
        // div_loch value 7 is not allowed
        // for (div_loch = 6; div_loch >= 0; --div_loch)
        for (div_loch = 0; div_loch <= 6; ++div_loch)
        {
            double pllHz = freq / (1 << (div_loch + 1));
            if (pllHz > 30e6 && pllHz < 3.8e9)
            {
                printf("VCO: %f, LO: %f\n", freq, pllHz);
                isValidFreq = true;
                break;
            }
        }
        if (!isValidFreq)
            return lime_Result_Error;

        const float ratio = freq / (refClk_Hz * (1 + (freq > m_dThrF)));
        struct pll_coefficients pll;
        pll.div_loch = div_loch;
        pll.integer = (uint16_t)(ratio - 4);
        pll.fractional = (uint32_t)((ratio - (uint32_t)ratio) * 1048576);
        pll.en_div2 = (freq > m_dThrF);
        pll.sel_vco = sel_vco;
        lms7002m_write_pll_registers(self, &pll, NULL);

        lime_Result status = lms7002m_tune_vco_sx_fast(self, NULL);
        if (status == lime_Result_Success)
        {
            uint8_t csw = lms7002m_spi_read_csr(self, LMS7002M_CSW_VCO);
            int16_t csw_min = csw;
            int16_t csw_max = csw;
            uint8_t cmphl;
            for (; csw_min > 0; --csw_min)
            {
                lms7002m_spi_modify_csr(self, LMS7002M_CSW_VCO, csw_min - 1);
                cmphl = ReadComparators(self, { addrCMP, 13, 12 });
                cmphl = ReadComparators(self, { addrCMP, 13, 12 });
                if (cmphl != 2)
                    break;
            }
            for (; csw_max < 255; ++csw_max)
            {
                lms7002m_spi_modify_csr(self, LMS7002M_CSW_VCO, csw_max + 1);
                cmphl = ReadComparators(self, { addrCMP, 13, 12 });
                cmphl = ReadComparators(self, { addrCMP, 13, 12 });
                if (cmphl != 2)
                    break;
            }

            freqs[stepsDone] = freq;
            csw_avg[stepsDone] = (csw_min + csw_max) / 2;
            acsw_min[stepsDone] = csw_min;
            acsw_max[stepsDone] = csw_max;
            ++stepsDone;
            if (stepsDone >= maxSteps)
                break;
        }
    }

    // split approximation into multiple CSW_VCO ranges approximations
    // theres a large shift in CSW values at some frequencies, which would throw off approximation if it would be single polynom for the whole frequency range
    const int csw_break_points[2] = { 128, 256 };
    int bp = 0;
    int start = 0;
    VCO_CSW_Data approx[2];
    int ap = 0;
    while (start < stepsDone)
    {
        int end = start;
        while (end < stepsDone)
        {
            if (csw_avg[end] >= csw_break_points[bp])
            {
                ++bp;
                break;
            }
            ++end;
        }
        int slots = end - start;

        lime_Result status = ProduceApproximation(&freqs[start], &csw_avg[start], slots, &approx[ap]);
        printf("{ %i, %i, %f, %f, %f},\n",
            approx[ap].vcomin_MHz,
            approx[ap].vcomax_MHz,
            approx[ap].polynom[0],
            approx[ap].polynom[1],
            approx[ap].polynom[2]);
        ap++;

        start = end;
    }

    GNUPlotPipe gp;

    gp.writef("set title 'Board S:N/0x%08X VCO Freq/CSW correlation'\n", 0);
    gp.write("set xlabel 'VCO freq MHz'\n");
    gp.writef("set xrange [%f:%f]\n", freqs[0] / 1e6, freqs[stepsDone - 1] / 1e6);
    gp.write("set xtics 250\n");

    gp.write("set ylabel 'CSW'\n");
    gp.write("set yrange [0:255]\n");
    gp.write("set ytics 16\n");

    gp.write("set grid ytics xtics\n");
    gp.write("set key right bottom\n");

    //gp.write("set style fill pattern 2 border\n");
    //gp.write("set style data lines\n");

    gp.write("plot ");

    gp.write("'-' u 1:2:3 w filledcu t 'VCO 128',");
    //gp.write("'-' u 1:2 lt -1 notitle, '-' u 1:3 lt -1 notitle,");
    gp.write("'-' u 1:2 w l t 'CSW approx'\n");

    std::stringstream ss;
    for (int i = 0; i < stepsDone; ++i)
        gp.writef("%lf %lf %lf\n", freqs[i] / 1e6, (float)acsw_min[i], (float)acsw_max[i]);
    gp.write("e\n");

    // const auto &data2 = highstats[sel_vco].measurements;
    // for(int i=0; i < data2.size(); ++i)
    //     gp.writef("%lf %lf %lf\n", data2[i].freqHz/1e6, (float)data2[i].cswRange[0], (float)data2[i].cswRange[1]);
    // gp.write("e\n");

    // for(int i=0; i < data2.size(); ++i)
    //     gp.writef("%lf %lf %lf\n", data2[i].freqHz/1e6, (float)data2[i].cswRange[0], (float)data2[i].cswRange[1]);
    // gp.write("e\n");

    for (int i = 0; i < stepsDone; ++i)
    {
        VCO_CSW_Data* data = &approx[0];
        float x = freqs[i] / 1e6;
        gp.writef("%lf %lf\n", x, data->polynom[0] + data->polynom[1] * x + data->polynom[2] * pow(x, 2));
        //gp.writef("%lf %lf\n", x, (float)csw_avg[i]);
    }
    gp.write("e\n");

    return lime_Result_Success;
}

using namespace std;
lime_Result lms7002m_measure_vco_range(struct lms7002m_context* self, int sel_vco, uint64_t* vcomin, uint64_t* vcomax)
{
    const uint64_t m_dThrF = 5500e6; //threshold to enable additional divider

    // reduce VCO frequency until it fails to tune
    uint64_t refClk_Hz = lms7002m_get_reference_clock(self);
    uint64_t mid = *vcomin + (*vcomax - *vcomin) / 2;
    uint64_t frequencyRange[2] = { mid, mid };
    int64_t steps[2] = { -200000000, 200000000 };
    const uint8_t cswValues[2] = { 0, 255 };
    const uint8_t targetCMP[2] = { 0, 3 };

    for (int dir = 0; dir < 2; ++dir) // min, then max search
    {
        double VCOfreq = mid;
        lms7002m_spi_modify_csr(self, LMS7002M_CSW_VCO, cswValues[dir]);
        do
        {
            VCOfreq = VCOfreq + steps[dir];

            // calculate params for desired VCO
            bool isValidFreq = false;
            int8_t div_loch;
            // div_loch value 7 is not allowed
            for (div_loch = 6; div_loch >= 0; --div_loch)
            {
                double LO = VCOfreq / (1 << (div_loch + 1));
                if (LO > 30e6 && LO < 3.8e9)
                {
                    isValidFreq = true;
                    break;
                }
            }
            if (!isValidFreq)
                return lime_Result_Error;

            const float ratio = VCOfreq / (refClk_Hz * (1 + (VCOfreq > m_dThrF)));

            struct pll_coefficients pll;
            pll.div_loch = div_loch;
            pll.integer = (uint16_t)(ratio - 4);
            ;
            pll.fractional = (uint32_t)((ratio - (uint32_t)ratio) * 1048576);
            pll.en_div2 = (VCOfreq > m_dThrF);
            pll.sel_vco = sel_vco;
            lms7002m_write_pll_registers(self, &pll, NULL);

            lms7002m_tune_vco_sx_fast(self, NULL);
            uint8_t cmphl = ReadComparators(self, { LMS7002M_VCO_CMPHO.address, 13, 12 });
            if (cmphl == targetCMP[dir] || cmphl == 2) // 2 is still locking
                frequencyRange[dir] = VCOfreq;
            else
                steps[dir] /= 2;
        } while (abs(steps[dir]) >= 1e6);
    }
    // back off from the edges to be safe
    const uint64_t offset = 10e6;
    frequencyRange[0] += offset;
    frequencyRange[1] -= offset;
    if (vcomin)
        *vcomin = frequencyRange[0];
    if (vcomax)
        *vcomax = frequencyRange[1];
    return lime_Result_Success;
}

lime_Result lms7002m_measure_vco_approximation(struct lms7002m_context* self, uint8_t sel_vco)
{
    int8_t div_loch;
    bool canDeliverFrequency = false;
    uint16_t integerPart;
    uint32_t fractionalPart;
    int16_t csw_value;

    const uint64_t refClk_Hz = lms7002m_get_reference_clock(self);

    uint64_t vcomin = VCO_min_frequency[sel_vco];
    uint64_t vcomax = VCO_max_frequency[sel_vco];
    lime_Result status = lms7002m_measure_vco_range(self, sel_vco, &vcomin, &vcomax);
    if (status != lime_Result_Success)
        return status;

    printf("%s = [%g : %g] MHz\n", vcoNames[sel_vco], vcomin / 1e6, vcomax / 1e6);
    // highstats[sel_vco] = stats[sel_vco];

    status = lms7002m_measure_calculate_csw_vco_approximation(self, sel_vco, vcomin, vcomax);
}
