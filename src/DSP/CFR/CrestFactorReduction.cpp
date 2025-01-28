#define _USE_MATH_DEFINES
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <memory>
#include <vector>

#include "comms/ISPI.h"
#include "CrestFactorReduction.h"
#include "registers.h"

using namespace lime;
using namespace CFR;

CrestFactorReduction::Config::Config()
{
    memset(this, 0, sizeof(Config));
}

CrestFactorReduction::CrestFactorReduction(std::shared_ptr<ISPI> comms)
    : m_Comms(comms)
{
}

CrestFactorReduction::~CrestFactorReduction()
{
}

void CrestFactorReduction::WriteRegister(const Register& reg, uint16_t value)
{
    uint32_t mosi = reg.address;
    uint32_t miso = 0;
    m_Comms->SPI(&mosi, &miso, 1);
    const uint16_t regMask = bitMask(reg.msb, reg.lsb);

    uint32_t regValue = (miso & ~regMask);
    regValue |= ((value << reg.lsb) & regMask);
    mosi = (1 << 31) | reg.address << 16 | regValue;
    m_Comms->SPI(&mosi, nullptr, 1);
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
