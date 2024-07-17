#define _USE_MATH_DEFINES
#include <cmath>
#include <assert.h>
#include <algorithm>
#include <cstdarg>
#include <vector>
#include <set>
#include <string.h>
#include <string>
#include <memory>

#include "INI.h"
#include "limesuiteng/types.h"
#include "limesuiteng/Logger.h"
#include "limesuiteng/LMS7002M.h"

#include "comms/ISPI.h"
#include "CrestFactorReduction.h"
#include "registers.h"

using namespace lime;
using namespace CFR;

static constexpr std::array<CSRegister, 36> CFRarray = { EN_RXTSP,
    EN_TXTSP,
    RX_DCCORR_BYP,
    RX_PHCORR_BYP,
    RX_GCORR_BYP,
    RX_EQU_BYP,
    RX_DCLOOP_BYP,
    RX_DCLOOP_AVG,
    TX_HB_BYP,
    TX_HB_DEL,
    SLEEP_CFR,
    BYPASS_CFR,
    ODD_CFR,
    BYPASSGAIN_CFR,
    SLEEP_FIR,
    BYPASS_FIR,
    ODD_FIR,
    TX_PHCORR_BYP,
    TX_GCORR_BYP,
    TX_DCCORR_BYP,
    TX_ISINC_BYP,
    TX_EQU_BYP,
    TX_INVERTQ,
    TX_GCORRQ,
    TX_GCORRI,
    TX_PHCORR,
    TX_DCCORRI,
    TX_DCCORRQ,
    thresholdSpin,
    thresholdGain,
    CFR_ORDER,
    RX_GCORRQ,
    RX_GCORRI,
    RX_PHCORR,
    cmbInsel,
    MAC };

static std::string stringFormat [[gnu::format(printf, 1, 2)]] (const char* format, ...)
{
    char buff[4096];
    va_list args;
    va_start(args, format);
    std::vsnprintf(buff, sizeof(buff), format, args);
    va_end(args);
    return std::string(buff);
}

CrestFactorReduction::Config::Config()
{
    memset(this, 0, sizeof(Config));
}

CrestFactorReduction::CrestFactorReduction(std::shared_ptr<ISPI> comms, LMS7002M* rfsoc)
    : m_Comms(comms)
    , lms2(rfsoc)
{
}

CrestFactorReduction::~CrestFactorReduction()
{
}

OpStatus CrestFactorReduction::WriteRegister(const Register& reg, uint16_t value)
{
    uint32_t mosi = reg.address;
    uint32_t miso = 0;
    OpStatus status = m_Comms->SPI(&mosi, &miso, 1);
    if (status != OpStatus::Success)
        return status;
    const uint16_t regMask = bitMask(reg.msb, reg.lsb);

    uint32_t regValue = (miso & ~regMask);
    regValue |= ((value << reg.lsb) & regMask);
    mosi = (1 << 31) | reg.address << 16 | regValue;
    return m_Comms->SPI(&mosi, nullptr, 1);
}

uint16_t CrestFactorReduction::ReadRegister(const Register& reg)
{
    uint32_t mosi = reg.address;
    uint32_t miso = 0;
    m_Comms->SPI(&mosi, &miso, 1);
    const uint16_t regMask = bitMask(reg.msb, reg.lsb);
    return (miso & regMask) >> reg.lsb;
}

void CrestFactorReduction::Configure(const CrestFactorReduction::Config& state)
{
    // TODO: batch writes
    for (uint8_t ch = 0; ch < 2; ++ch)
    {
        WriteRegister(MAC, ch + 1);
        WriteRegister(RX_EQU_BYP, state.bypassRxEqualizer[ch]);
        WriteRegister(TX_EQU_BYP, state.bypassTxEqualizer[ch]);

        const Config::CFR& cfr = state.cfr[ch];
        const Config::FIR& fir = state.fir[ch];

        const bool useOversample = std::min<uint8_t>(cfr.interpolation, 2) != 1;

        WriteRegister(TX_HB_BYP, !useOversample);
        WriteRegister(TX_HB_DEL, useOversample);
        WriteRegister(SLEEP_CFR, cfr.sleep);
        WriteRegister(BYPASS_CFR, cfr.bypass);
        WriteRegister(ODD_CFR, fir.coefficientsCount % 2);
        WriteRegister(BYPASSGAIN_CFR, cfr.bypassGain);

        assert(fir.coefficientsCount <= 32);
        SetFIRCoefficients(fir.coefficients, fir.coefficientsCount);
        // WriteRegister(ODD_FIR, fir.coefficientsCount % 2); // set in SetFIRCoefficients
        WriteRegister(SLEEP_FIR, fir.sleep);
        WriteRegister(BYPASS_FIR, fir.bypass);

        if (cfr.order >= 2 && cfr.order <= 32)
        {
            WriteRegister(CFR_ORDER, cfr.order);
            UpdateHannCoeff(cfr.order);
        }
        WriteRegister(thresholdSpin, cfr.threshold);
        WriteRegister(thresholdGain, cfr.thresholdGain);
    }
}

// Generates coefficients based on CFR order
void CrestFactorReduction::UpdateHannCoeff(uint16_t Filt_N)
{
    uint16_t msb, lsb = 0;
    uint16_t data = 0;
    uint16_t addr = 0;
    uint16_t i = 0;
    uint16_t j = 0;
    uint16_t offset = 0;
    uint16_t w[40];

    // CFR registers
    uint16_t maddressf0 = 0x000C; // 12*64=768
    uint16_t maddressf1 = 0x000D; // 13*64=832
    uint16_t NN = 15;
    const uint16_t MaxFilt_N = 32;

    Filt_N = std::min(Filt_N, MaxFilt_N);

    for (i = 0; i < Filt_N; ++i)
        w[i] = static_cast<uint16_t>(32768.0 * 0.25 * (1.0 - cos(2.0 * M_PI * i / (Filt_N - 1))));

    WriteRegister(SLEEP_CFR, 1);
    msb = lsb = 0;
    data = 0;
    i = 0;

    //WriteRegister(RESET_N, 0);
    //WriteRegister(RESET_N, 1);

    // TODO: figure out this address mess. For now just replaced SPI write functions
    std::vector<uint32_t> mosi;
    while (i < MaxFilt_N)
    {
        addr = (2 << 15) + (maddressf0 << 6) + (msb << 4) + lsb;
        mosi.push_back((1 << 31) | addr << 16 | data);
        addr = (2 << 15) + (maddressf1 << 6) + (msb << 4) + lsb;
        mosi.push_back((1 << 31) | addr << 16 | data);
        if (lsb >= NN) // 15
        {
            lsb = 0;
            msb++;
        }
        else
            lsb++;
        i++;
    }
    m_Comms->SPI(mosi.data(), nullptr, mosi.size());
    mosi.clear();

    msb = lsb = 0;
    i = j = 0;
    offset = 0;
    while (i <= static_cast<uint16_t>((Filt_N / 2) - 1))
    {
        addr = (2 << 15) + (maddressf1 << 6) + (msb << 4) + lsb;
        if (j >= offset)
            data = w[static_cast<uint16_t>((Filt_N + 1) / 2 + i)];
        else
            data = 0;
        mosi.push_back((1 << 31) | addr << 16 | data);
        if (lsb >= NN) // 15
        {
            lsb = 0;
            msb++;
        }
        else
            lsb++;

        if (j >= offset)
            i++;
        j++;
    }
    m_Comms->SPI(mosi.data(), nullptr, mosi.size());
    mosi.clear();

    msb = lsb = 0;
    i = j = 0;
    offset = (MaxFilt_N / 2) - (static_cast<uint16_t>((Filt_N + 1) / 2));
    while (i < Filt_N)
    {
        addr = (2 << 15) + (maddressf0 << 6) + (msb << 4) + lsb;

        if (j >= offset)
            data = w[i];
        else
            data = 0;

        mosi.push_back((1 << 31) | addr << 16 | data);
        if (lsb >= NN) // 3
        {
            lsb = 0;
            msb++;
        }
        else
            lsb++;

        if (j >= offset)
            i++;
        j++;
    }
    m_Comms->SPI(mosi.data(), nullptr, mosi.size());
    mosi.clear();

    WriteRegister(ODD_CFR, Filt_N % 2);
    WriteRegister(SLEEP_CFR, 0); // RELEASE SLEEP_CFR

    // software reset
    //WriteRegister(RESET_N, 0);
    //WriteRegister(RESET_N, 1);
}

void CrestFactorReduction::SetFIRCoefficients(const int16_t* coefficients, uint16_t count)
{
    const int maxCoefCount = 32;
    assert(count <= 32);

    // FIR registers
    uint16_t maddressf0 = 0x000E; // 14x64 = 896;
    uint16_t maddressf1 = 0x000F; // 15x64 = 960;

    const uint16_t Filt_N = 32;
    uint16_t NN = 0;
    uint16_t addr = 0;
    uint16_t data = 0;
    uint16_t msb = 0;
    uint16_t lsb = 0;
    uint16_t i = 0;

    NN = 15;

    // std::vector<double> coefficients;
    // coefficients.resize(Filt_N, 0);

    // for (i = 0; i < Filt_N; i++) // maxCoefCount
    //     coefficients[i] = 0.0;

    // read coeffs
    // msb = lsb = i = 0;
    // while (i <= (uint16_t)((Filt_N)-1))
    // {
    //     addr = (maddressf0 << 6) + (msb << 4) + lsb;
    //     uint32_t mosi = addr;
    //     uint32_t miso = 0;
    //     m_Comms->SPI(&mosi, &miso, 1);
    //     coefficients[i] = (double)(miso & 0xFFFF);
    //     if (lsb >= NN) // 15
    //     {
    //         lsb = 0;
    //         msb++;
    //     }
    //     else
    //         lsb++;
    //     i++;
    // }

    WriteRegister(SLEEP_FIR, 1);

    std::vector<uint32_t> mosi;
    for (i = 0; i < maxCoefCount; i++)
    {
        addr = (maddressf0 << 6) + i;
        mosi.push_back((1 << 31) | addr << 16 | 0);
        addr = (maddressf1 << 6) + i;
        mosi.push_back((1 << 31) | addr << 16 | 0);
    }
    m_Comms->SPI(mosi.data(), nullptr, mosi.size());
    mosi.clear();

    msb = lsb = i = 0;
    while (i <= static_cast<uint16_t>((Filt_N)-1))
    {
        addr = (maddressf0 << 6) + (msb << 4) + lsb;
        data = static_cast<uint16_t>(coefficients[i]);
        mosi.push_back((1 << 31) | addr << 16 | data);

        addr = (maddressf1 << 6) + (msb << 4) + lsb;
        data = static_cast<uint16_t>(coefficients[i]);
        mosi.push_back((1 << 31) | addr << 16 | data);
        if (lsb >= NN) // 15
        {
            lsb = 0;
            msb++;
        }
        else
            lsb++;
        i++;
    }
    m_Comms->SPI(mosi.data(), nullptr, mosi.size());
    mosi.clear();

    WriteRegister(ODD_FIR, Filt_N % 2);
    WriteRegister(SLEEP_FIR, 0);
}

void CrestFactorReduction::SetOversample(uint8_t oversample)
{
    // treat oversample 0 as "auto", maximum available oversample
    const uint8_t maxOversample = 2;
    oversample = std::min(oversample, maxOversample);

    const bool useOversample = oversample != 1;
    // TODO: batch writes
    for (uint8_t ch = 0; ch < 2; ++ch)
    {
        WriteRegister(MAC, ch + 1);
        WriteRegister(TX_HB_BYP, !useOversample);
        WriteRegister(TX_HB_DEL, useOversample);
    }
}

uint8_t CrestFactorReduction::GetOversample()
{
    const int ch = 0;
    WriteRegister(MAC, ch + 1);
    int bypass = ReadRegister(TX_HB_BYP);
    int delay = ReadRegister(TX_HB_DEL);
    // TODO: Warn if bypass and delay are incompatible
    return (delay && !bypass) ? 2 : 1;
}

void CrestFactorReduction::SaveRegisterRangesToIni(INI* m_options, uint8_t chipId, const std::vector<Range<uint16_t>>& ranges)
{
    uint16_t regValue;
    for (const auto& range : ranges)
    {
        for (size_t address = range.min; address <= range.max; address += range.step)
        {
            regValue = ReadRegister(address);
            std::string key = stringFormat("Ch%d_Reg:0x%04X", chipId, static_cast<uint16_t>(address));
            m_options->set(key, std::to_string(regValue));
        }
    }
}

void CrestFactorReduction::FDQIE_SaveEqualiser(int Ntaps, const std::string& m_sConfigFilename)
{
    INI m_options(m_sConfigFilename, true);
    m_options.select("LimeSDR-5GRadio");

    // const int maxCoefCount = 16;
    const std::vector<Range<uint16_t>> coeffsRanges = {
        { 0x2C0, 0x2CF }, // 704 - address of Equ Tx coeffs I
        { 0x2D0, 0x2DF }, // 720 Tx Q
        { 0x2E0, 0x2EF }, // 736  -address of Equ Rx coeffs I
        { 0x2F0, 0x2FF }, // 752 Rx Q
        { CFR::TX_GCORRI.address, CFR::TX_GCORRI.address },
        { CFR::TX_GCORRQ.address, CFR::TX_GCORRQ.address },
        { CFR::TX_PHCORR.address, CFR::TX_PHCORR.address },

        { CFR::RX_GCORRI.address, CFR::RX_GCORRI.address },
        { CFR::RX_GCORRQ.address, CFR::RX_GCORRQ.address },
        { CFR::RX_PHCORR.address, CFR::RX_PHCORR.address },
    };

    for (int chipId = 1; chipId <= 2; ++chipId)
    {
        if (WriteRegister(0xFFFF, chipId) != OpStatus::Success)
        {
            printf("[Warn] Device cannot be opened.\n");
            return;
        }

        SaveRegisterRangesToIni(&m_options, chipId, coeffsRanges);
    }

    m_options.save(m_sConfigFilename);
    printf("[Info] Equaliser configuration is saved in file %s.\n", m_sConfigFilename.c_str());

    if (WriteRegister(0xFFFF, 1) != OpStatus::Success)
    {
        printf("[Warn] Cannot write data to device.\n");
        return;
    }
}

void CrestFactorReduction::FDQIE_SaveDC(double RefClk, const std::string& m_sConfigFilename)
{
    INI m_options(m_sConfigFilename, true);
    m_options.select("LimeSDR-5GRadio");

    const std::vector<Range<uint16_t>> addrRanges = {
        { CFR::TX_DCCORRI.address, CFR::TX_DCCORRI.address },
        { CFR::TX_DCCORRQ.address, CFR::TX_DCCORRQ.address },
    };

    for (int chipId = 1; chipId <= 2; chipId++)
    {
        if (WriteRegister(0xFFFF, chipId) != OpStatus::Success)
        {
            printf("[Warn] Device cannot be opened.\n");
            return;
        }

        SaveRegisterRangesToIni(&m_options, chipId, addrRanges);
    }

    m_options.save(m_sConfigFilename);
    printf("[Info] DC calibration results are saved in file %s.\n", m_sConfigFilename.c_str());
}

void CrestFactorReduction::SaveCFRFIR(const std::string& m_sConfigFilename)
{
    INI m_options(m_sConfigFilename, true);
    m_options.select("LimeSDR-5GRadio");

    // FIR registers start address
    const int maxCoefCount = 32;
    const std::vector<Range<uint16_t>> addrRanges = {
        { 768, 768 + maxCoefCount - 1 }, // address of CFR
        { 832, 832 + maxCoefCount - 1 }, // CFR
        { 896, 896 + maxCoefCount - 1 }, // Post-CFR FIR filter coefficients
        { 960, 960 + maxCoefCount - 1 }, // FIR
    };

    std::set<uint16_t> CFRadresses;
    for (const auto& csr : CFRarray)
        CFRadresses.insert(csr.address);

    for (int kk = 1; kk <= 2; ++kk)
    {
        if (WriteRegister(0xFFFF, kk) != OpStatus::Success)
        {
            printf("[Warn] Device cannot be opened.\n");
            return;
        }

        // Save various board configuration
        for (const auto addr : CFRadresses)
        {
            uint16_t value = ReadRegister(Register(addr));
            m_options.set(stringFormat("Ch%d_Reg:0x%04X", kk, addr), std::to_string(value));
        }

        SaveRegisterRangesToIni(&m_options, kk, addrRanges);
    }
    m_options.save(m_sConfigFilename);
    printf("[Info] CFRFIR configuration is saved to file %s.\n", m_sConfigFilename.c_str());
}

void CrestFactorReduction::FDQIE_LoadEqualiser(const std::string& m_sConfigFilename)
{

    // load configuration from file
    uint16_t regValue = 0;
    const char* config_filename = m_sConfigFilename.c_str();
    INI m_options(m_sConfigFilename, true);
    m_options.select("LimeSDR-5GRadio");

    constexpr int maxCoefCount = 16;
    constexpr uint16_t maddressT_hI = 0x2C0; // address of TX EQU coeffs
    constexpr uint16_t maddressT_hQ = 0x2D0;
    constexpr uint16_t maddressR_hI = 0x2E0; // address of RX EQU coeffs
    constexpr uint16_t maddressR_hQ = 0x2F0;

    for (int kk = 1; kk <= 2; kk++)
    {

        if (WriteRegister(0xFFFF, kk) != OpStatus::Success)
        {
            printf("[Warn] Device cannot be opened.\n");
            return;
        }
        for (int i = 0; i < maxCoefCount; i++)
        {
            regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, maddressT_hI + i).c_str(), 0);
            if (WriteRegister(maddressT_hI + i, regValue) != OpStatus::Success)
                printf("[Warn] Cannot read data from file.\n");
            regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, maddressT_hQ + i).c_str(), 0);
            if (WriteRegister(maddressT_hQ + i, regValue) != OpStatus::Success)
                printf("[Warn] Cannot read data from file.\n");
            regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, maddressR_hI + i).c_str(), 0);
            if (WriteRegister(maddressR_hI + i, regValue) != OpStatus::Success)
                printf("[Warn] Cannot read data from file.\n");
            regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, maddressR_hQ + i).c_str(), 0);
            if (WriteRegister(maddressR_hQ + i, regValue) != OpStatus::Success)
                printf("[Warn] Cannot read data from file.\n");
        }
        // Static transmitter I/Q corrector
        {
            constexpr Register reg1{ CFR::TX_GCORRI };
            constexpr Register reg2{ CFR::TX_GCORRQ };
            constexpr Register reg3{ CFR::TX_PHCORR };

            regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, reg1.address).c_str(), 0);
            if (WriteRegister(reg1.address, regValue) != OpStatus::Success)
                printf("[Warn] Cannot write data to device.\n");
            regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, reg2.address).c_str(), 0);
            if (WriteRegister(reg2.address, regValue) != OpStatus::Success)
                printf("[Warn] Cannot write data to device.\n");
            regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, reg3.address).c_str(), 0);
            if (WriteRegister(reg3.address, regValue) != OpStatus::Success)
                printf("[Warn] Cannot write data to device.\n");
        }

        // Static RX I/Q corrector
        {
            constexpr Register reg1{ CFR::RX_GCORRI };
            constexpr Register reg2{ CFR::RX_GCORRQ };
            constexpr Register reg3{ CFR::RX_PHCORR };

            regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, reg1.address).c_str(), 0);
            if (WriteRegister(reg1.address, regValue) != OpStatus::Success)
                printf("[Warn] Cannot write data to device.\n");

            regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, reg2.address).c_str(), 0);
            if (WriteRegister(reg2.address, regValue) != OpStatus::Success)
                printf("[Warn] Cannot write data to device.\n");

            regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, reg3.address).c_str(), 0);
            if (WriteRegister(reg3.address, regValue) != OpStatus::Success)
                printf("[Warn] Cannot write data to device.\n");
        }
    }

    if (WriteRegister(0xFFFF, 1) != OpStatus::Success)
    {
        printf("[Warn] Cannot write data to device.\n");
        return;
    }
    printf("[Info] Equaliser configuration is loaded from file %s.\n", config_filename);
}

void CrestFactorReduction::FDQIE_LoadDC(const std::string& m_sConfigFilename)
{
    uint16_t regValue = 0;
    // Transmitter DC
    constexpr Register reg1{ CFR::TX_DCCORRI };
    constexpr Register reg2{ CFR::TX_DCCORRQ };

    const char* config_filename = m_sConfigFilename.c_str();
    INI m_options(m_sConfigFilename, true);
    m_options.select("LimeSDR-5GRadio");

    for (int kk = 1; kk <= 2; kk++)
    {

        if (WriteRegister(0xFFFF, kk) != OpStatus::Success)
        {
            printf("[Warn] Device cannot be opened.\n");
            return;
        }
        regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, reg1.address).c_str(), 0);
        if (WriteRegister(reg1.address, regValue) != OpStatus::Success)
            printf("[Warn] Cannot write data to device.\n");

        regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, reg2.address).c_str(), 0);
        if (WriteRegister(reg2.address, regValue) != OpStatus::Success)
            printf("[Warn] Cannot write data to device.\n");
    }
    printf("[Info] DC calibration results are loaded from file %s.\n", config_filename);
}

void CrestFactorReduction::LoadCFRFIR(const std::string& m_sConfigFilename)
{
    // load configuration from file
    uint16_t regValue = 0;
    const char* config_filename = m_sConfigFilename.c_str();
    INI m_options(m_sConfigFilename, true);
    m_options.select("LimeSDR-5GRadio");
    // std::map<wxObject*, CSRegister>::iterator iter;

    for (int kk = 1; kk <= 2; kk++)
    {
        if (WriteRegister(0xFFFF, kk) != OpStatus::Success)
        {
            lime::error("[Warn] Device cannot be opened.\n");
            return;
        }
        // for (iter = controlsPtr2Registers.begin(); iter != controlsPtr2Registers.end(); ++iter)
        // {
        //     Register reg = iter->second;
        //     regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, reg.address), 0); // read data from file
        //     WriteRegister(reg.address, regValue);
        // }
        // Load CFR FIR filter coefficients
        const int maxCoefCount = 32;
        uint16_t maddressf0 = 768; // CFR
        uint16_t maddressf1 = 832; // CFR
        uint16_t maddressf2 = 896; // FIR
        uint16_t maddressf3 = 960; // FIR
        for (int i = 0; i < maxCoefCount; i++)
        {
            regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, maddressf0 + i), 0);
            WriteRegister(maddressf0 + i, regValue);

            regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, maddressf1 + i), 0);
            WriteRegister(maddressf1 + i, regValue);

            regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, maddressf2 + i), 0);
            WriteRegister(maddressf2 + i, regValue);

            regValue = m_options.getAs(stringFormat("Ch%d_Reg:0x%04X", kk, maddressf3 + i), 0);
            WriteRegister(maddressf3 + i, regValue);
        }
    }

    if (WriteRegister(0xFFFF, 0x1) != OpStatus::Success)
    {
        lime::error("[Warn] Cannot write data to device.\n");
        return;
    }

    lime::info("[Info] CFR & CFRFIR configuration is loaded from file %s.\n", config_filename);
}

void CrestFactorReduction::FDQIE_ResetEqualiser(int kk)
{
    const int maxCoefCount = 16;
    uint16_t maddressT_hI = 0x2C0; // address of transmitter's coeffs
    uint16_t maddressT_hQ = 0x2D0;
    uint16_t maddressR_hI = 0x2E0; // address of receiver's coeffs
    uint16_t maddressR_hQ = 0x2F0;
    int16_t regValue = 0;

    if (WriteRegister(0xFFFF, kk) != OpStatus::Success)
    {
        printf("[Warn] Device cannot be opened.\n");
        return;
    }

    for (int i = 0; i < maxCoefCount; i++)
    {
        if (i == 4)
            regValue = 32767;
        else
            regValue = 0;

        if (WriteRegister(maddressT_hI + i, regValue) != OpStatus::Success)
            printf("[Warn] Cannot read data from file.\n");
        if (WriteRegister(maddressT_hQ + i, regValue) != OpStatus::Success)
            printf("[Warn] Cannot read data from file.\n");
        if (WriteRegister(maddressR_hI + i, regValue) != OpStatus::Success)
            printf("[Warn] Cannot read data from file.\n");
        if (WriteRegister(maddressR_hQ + i, regValue) != OpStatus::Success)
            printf("[Warn] Cannot read data from file.\n");
    }

    if (WriteRegister(CFR::TX_GCORRI, 2047) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");
    if (WriteRegister(CFR::TX_GCORRQ, 2047) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");
    if (WriteRegister(CFR::TX_PHCORR, 0) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");

    if (WriteRegister(CFR::RX_GCORRI, 2047) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");
    if (WriteRegister(CFR::RX_GCORRQ, 2047) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");
    if (WriteRegister(CFR::RX_PHCORR, 0) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");

    FDQIE_SetupTransmitterDC(0, 0, kk); // default
    FDQIE_SetupReceiverDC(lms2, 0, 0, kk); // default

    printf("[Info] Equaliser is in reset state.\n");
}

void CrestFactorReduction::FDQIE_SetupTransmitterDC(int16_t codeI, int16_t codeQ, int kk)
{
    if (WriteRegister(0xFFFF, kk) != OpStatus::Success)
    {
        printf("[Warn] Device cannot be opened.\n");
        return;
    }

    if (WriteRegister(CFR::TX_DCCORRI, codeI) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");

    if (WriteRegister(CFR::TX_DCCORRQ, codeQ) != OpStatus::Success)
        printf("[Warn] Cannot write data to device.\n");
}

void CrestFactorReduction::FDQIE_SetupReceiverDC(lime::LMS7002M* rfsoc, int16_t codeI, int16_t codeQ, int kk)
{
    rfsoc->Modify_SPI_Reg_bits(LMS7002MCSR::MAC, kk);
    rfsoc->Modify_SPI_Reg_bits(LMS7002MCSR::EN_DCOFF_RXFE_RFE, 1);

    uint16_t regValue = 0;

    if (codeI < 0)
        regValue = 64 - codeI;
    else
        regValue = codeI;

    rfsoc->Modify_SPI_Reg_bits(LMS7002MCSR::DCOFFI_RFE, regValue);

    if (codeQ < 0)
        regValue = 64 - codeQ;
    else
        regValue = codeQ;

    rfsoc->Modify_SPI_Reg_bits(LMS7002MCSR::DCOFFQ_RFE, regValue);
}
