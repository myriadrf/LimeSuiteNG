/**
@file	Si5351C.cpp
@author	Lime Microsystems
@brief 	Configuring of Si5351C
*/

#include "Si5351C.h"
#include "limesuiteng/Logger.h"
#include <cmath>
#include <iomanip>
#include <fstream>
#include <set>
#include <map>
#include <cstring>
#include <cassert>
#include <algorithm>
#include <vector>
#include "comms/II2C.h"
#include <string_view>

using namespace std;
using namespace lime;
using namespace std::literals::string_view_literals;

static const uint8_t addrSi5351 = 0xC0;

/// Splits float into fraction integers A + B/C
void realToFrac(const float real, int& A, int& B, int& C)
{
    A = static_cast<int>(real);
    B = std::round((real - A) * 1048576);
    C = 1048576;

    int a = B;
    int b = C;
    int temp;
    while (b != 0) // greatest common divider
    {
        temp = a % b;
        a = b;
        b = temp;
    }
    B = B / a;
    C = C / a;
}

struct Si5351C_Memory {
    unsigned char index;
    unsigned char value;
};

/// Default configuration
static const std::vector<Si5351C_Memory> defaultConfiguration{
    { 2, 0x08 },
    { 15, 0x04 },
    { 16, 0x4F },
    { 17, 0x4F },
    { 18, 0x4F },
    { 19, 0x4F },
    { 20, 0x4F },
    { 21, 0x4F },
    { 22, 0x0F },
    { 23, 0x0F },
    { 27, 0x80 },
    { 29, 0x0B },
    { 30, 0x2F },
    { 43, 0x01 },
    { 45, 0x0D },
    { 51, 0x01 },
    { 53, 0x0D },
    { 59, 0x01 },
    { 61, 0x0D },
    { 67, 0x01 },
    { 69, 0x0D },
    { 75, 0x01 },
    { 77, 0x0D },
    { 83, 0x01 },
    { 85, 0x0D },
    { 90, 0x1E },
    { 91, 0x1E },
    { 181, 0x30 },
    { 183, 0xD2 },
    { 184, 0x60 },
    { 185, 0x60 },
    { 187, 0xC0 },
    { 221, 0x0D },
};

// ---------------------------------------------------------------------------
Si5351C::Si5351C(lime::II2C& i2c_comms)
    : comms(i2c_comms)
{
    Reset();
}
// ---------------------------------------------------------------------------

Si5351C::~Si5351C()
{
}

/** 
 * @brief Sends Configuration to Si5351C
 * @return The status code of the operation
*/
Si5351C::Status Si5351C::UploadConfiguration()
{
    std::vector<uint8_t> outBuffer;
    //Disable outputs
    outBuffer.push_back(3);
    outBuffer.push_back(0xFF);
    //Power down all output drivers
    for (int i = 0; i < 8; ++i)
    {
        outBuffer.push_back(16 + i);
        outBuffer.push_back(0x84);
    }
    //write new configuration
    for (int i = 15; i <= 92; ++i)
    {
        outBuffer.push_back(i);
        outBuffer.push_back(m_newConfiguration[i]);
    }
    for (int i = 149; i <= 170; ++i)
    {
        outBuffer.push_back(i);
        outBuffer.push_back(m_newConfiguration[i]);
    }
    //apply soft reset
    outBuffer.push_back(0XB1);
    outBuffer.push_back(0xAC);
    //Enabe desired outputs
    outBuffer.push_back(3);
    outBuffer.push_back(m_newConfiguration[3]);

    try
    {
        comms.I2CWrite(addrSi5351, outBuffer.data(), outBuffer.size());
        return Status::SUCCESS;
    } catch (std::runtime_error& e)
    {
        lime::error("Si5351C configuration failed "s + e.what());
        return Status::FAILED;
    }
}

/**
    @brief Loads register values for Si5356A from file
    @param FName input filename
*/
void Si5351C::LoadRegValuesFromFile(string FName)
{
    fstream fin;
    fin.open(FName, ios::in);

    const int len = 1024;
    char line[len];

    int addr;
    unsigned int value;

    while (!fin.eof())
    {
        fin.getline(line, len);
        if (std::string_view{ line } == "#END_PROFILE"sv)
            break;
        if (line[0] == '#')
            continue;
        sscanf(line, "%i,%x", &addr, &value);
        m_newConfiguration[addr] = value;
    }

    fin.close();
}

std::set<unsigned long> Si5351C::GenerateFrequencies(
    const unsigned long outputFrequency, const unsigned long Fmin, const unsigned long Fmax)
{
    std::set<unsigned long> returnSet;
    unsigned int mult = 6;
    unsigned long freq = outputFrequency;

    while (freq <= Fmax && mult <= 254)
    {
        freq = outputFrequency * mult;
        if (freq >= Fmin && freq <= Fmax)
        {
            returnSet.insert(freq);
        }
        mult += 2;
    }

    return returnSet;
}

unsigned long Si5351C::FindBestVCO(lime::Si5351_Channel* clocks, std::map<unsigned long, int>& availableFrequencies)
{
    int bestScore = 0; //score shows how many outputs have integer dividers
    //calculate scores for all available frequencies
    unsigned long bestVCO = 0;

    for (auto& freq : availableFrequencies)
    {
        for (int i = 0; i < 8; ++i)
        {
            if (clocks[i].outputFreqHz == 0 || !clocks[i].powered)
                continue;

            if ((freq.first % clocks[i].outputFreqHz) == 0)
            {
                freq.second = freq.second + 1;
            }
        }
        if (freq.second >= bestScore)
        {
            bestScore = freq.second;
            bestVCO = freq.first;
        }
    }

    return bestVCO;
}

/** @brief Calculates multisynth dividers and VCO frequencies
    @param clocks output clocks configuration
    @param plls plls configurations
    @param Fmin lowest VCO frequency
    @param Fmax highest VCO frequency
*/
void Si5351C::FindVCO(Si5351_Channel* clocks, Si5351_PLL* plls, const unsigned long Fmin, const unsigned long Fmax)
{
    int clockCount = 8;
    //reset output parameters
    for (int i = 0; i < clockCount; i++)
    {
        clocks[i].pllSource = 0;
        clocks[i].int_mode = 0;
        clocks[i].multisynthDivider = 8;
    }

    bool clk6satisfied = !clocks[6].powered;
    bool clk7satisfied = !clocks[7].powered;

    bool pllAused = false;
    bool pllBused = false;

    map<unsigned long, int> availableFrequenciesPLLA; //all available frequencies for VCO
    map<unsigned long, int> availableFrequenciesPLLB; //all available frequencies for VCO

    //if clk6 or clk7 is used make available frequencies according to them
    if (clocks[6].powered || clocks[7].powered)
    {
        std::set<unsigned long> clk6freqs;
        std::set<unsigned long> clk7freqs;
        if (!clk6satisfied)
        {
            clk6freqs = GenerateFrequencies(clocks[6].outputFreqHz, Fmin, Fmax);
        }

        if (!clk7satisfied)
        {
            clk7freqs = GenerateFrequencies(clocks[7].outputFreqHz, Fmin, Fmax);
        }

        std::set<unsigned long> sharedFreqs;
        //find if clk6 and clk7 can share the same pll
        std::set_intersection(clk6freqs.begin(),
            clk6freqs.end(),
            clk7freqs.begin(),
            clk7freqs.end(),
            std::inserter(sharedFreqs, sharedFreqs.begin()));
        bool canShare = !sharedFreqs.empty();

        if (canShare) //assign PLLA for both clocks
        {
            clocks[6].pllSource = 0;
            clocks[7].pllSource = 0;
            pllAused = true;

            for (const auto freq : sharedFreqs)
            {
                availableFrequenciesPLLA[freq] = 0;
            }
        }
        else //if clocks 6 and 7 can't share pll, assign pllA to clk6 and pllB to clk7
        {
            if (!clk6satisfied)
            {
                clocks[6].pllSource = 0;
                pllAused = true;
                for (const auto freq : clk6freqs)
                {
                    availableFrequenciesPLLA[freq] = 0;
                }
            }
            if (!clk7satisfied)
            {
                clocks[7].pllSource = 1;
                pllBused = true;
                for (const auto freq : clk7freqs)
                {
                    availableFrequenciesPLLB[freq] = 0;
                }
            }
        }
    }

    //PLLA stage, find  all clocks that have integer coefficients with PLLA
    //if pllA is not used by clk6 or clk7, fill available frequencies according to clk1-clk5 clocks
    if (availableFrequenciesPLLA.size() == 0 && !pllAused)
    {
        for (int i = 0; i < 6; ++i)
        {
            unsigned long freq =
                clocks[i].outputFreqHz > Fmin
                    ? clocks[i].outputFreqHz
                    : (clocks[i].outputFreqHz * ((Fmin / clocks[i].outputFreqHz) + ((Fmin % clocks[i].outputFreqHz) != 0)));
            while (freq >= Fmin && freq <= Fmax)
            {
                //add all output frequency multiples that are in VCO interval
                availableFrequenciesPLLA[freq] = 0;
                freq += clocks[i].outputFreqHz;
            }
        }
    }

    auto bestVCOA = FindBestVCO(clocks, availableFrequenciesPLLA);

    plls[0].VCO_Hz = bestVCOA;
    plls[0].feedbackDivider = static_cast<double>(bestVCOA) / plls[0].inputFreqHz;

    for (int i = 0; i < clockCount; ++i)
    {
        if (clocks[i].outputFreqHz == 0 || !clocks[i].powered)
            continue;

        if (bestVCOA % clocks[i].outputFreqHz == 0)
        {
            clocks[i].int_mode = true;
            clocks[i].multisynthDivider = bestVCOA / clocks[i].outputFreqHz;
        }
        else
        {
            clocks[i].int_mode = false;
            clocks[i].multisynthDivider = static_cast<double>(bestVCOA) / clocks[i].outputFreqHz;
        }
        clocks[i].pllSource = 0;
    }

    //PLLB stage, find  all clocks that have integer coefficients with PLLB
    //if pllB is not used by clk6 or clk7, fill available frequencies according to clk1-clk5 clocks, that don't have integer dividers
    if (availableFrequenciesPLLB.size() == 0 && !pllBused)
    {
        for (int i = 0; i < 6; ++i)
        {
            if (clocks[i].outputFreqHz == 0 || !clocks[i].powered)
                continue;

            if (clocks[i].int_mode) //skip clocks with integer dividers
                continue;
            unsigned long freq =
                clocks[i].outputFreqHz > Fmin
                    ? clocks[i].outputFreqHz
                    : (clocks[i].outputFreqHz * ((Fmin / clocks[i].outputFreqHz) + ((Fmin % clocks[i].outputFreqHz) != 0)));
            while (freq >= Fmin && freq <= Fmax)
            {
                availableFrequenciesPLLB[freq] = 0;
                freq += clocks[i].outputFreqHz;
            }
        }
    }

    auto bestVCOB = FindBestVCO(clocks, availableFrequenciesPLLB);

    if (bestVCOB == 0) //just in case if pllb is not used make it the same frequency as plla
        bestVCOB = bestVCOA;
    plls[1].VCO_Hz = bestVCOB;
    plls[1].feedbackDivider = static_cast<double>(bestVCOB) / plls[0].inputFreqHz;
    for (int i = 0; i < clockCount; ++i)
    {
        if (clocks[i].outputFreqHz == 0 || !clocks[i].powered)
            continue;

        if (clocks[i].int_mode)
            continue;

        if (bestVCOB % clocks[i].outputFreqHz == 0)
        {
            clocks[i].int_mode = true;
            clocks[i].multisynthDivider = bestVCOB / clocks[i].outputFreqHz;
        }
        else
        {
            clocks[i].int_mode = false;
            clocks[i].multisynthDivider = static_cast<double>(bestVCOB) / clocks[i].outputFreqHz;
        }
        clocks[i].pllSource = 1;
    }
}

/** @brief Modifies register map with clock settings
    @return true if success
*/
Si5351C::Status Si5351C::ConfigureClocks()
{
    FindVCO(CLK, PLL, 600000000, 900000000);
    int addr;
    m_newConfiguration[3] = 0;
    for (int i = 0; i < 8; ++i)
    {
        m_newConfiguration[3] |= (!CLK[i].powered) << i; //enabled
        m_newConfiguration[16 + i] = !CLK[i].powered << 7; // powered

        if (CLK[i].int_mode)
        {
            m_newConfiguration[16 + i] |= 1 << 6; //integer mode
        }

        m_newConfiguration[16 + i] |= CLK[i].pllSource << 5; //PLL source
        m_newConfiguration[16 + i] |= CLK[i].inverted << 4; // invert
        m_newConfiguration[16 + i] |= 0b1111;

        addr = 42 + i * 8;
        int DivA, DivB, DivC;
        realToFrac(CLK[i].multisynthDivider, DivA, DivB, DivC);

        lime::info("CLK%d fOut = %g MHz  Multisynth Divider %d %d/%d  R divider = %d source = %s",
            i,
            CLK[i].outputFreqHz / 1000000.0,
            DivA,
            DivB,
            DivC,
            CLK[i].outputDivider,
            (CLK[i].pllSource == 0 ? "PLLA" : "PLLB"));

        if (CLK[i].multisynthDivider < 8 || 900 < CLK[i].multisynthDivider)
        {
            lime::error("Si5351C - Output multisynth divider is outside [8;900] interval."s);
            return Status::FAILED;
        }

        if (i < 6)
        {
            if (CLK[i].outputFreqHz <= 150000000)
            {
                unsigned MSX_P1 = 128 * DivA + std::floor(128 * (static_cast<float>(DivB) / DivC)) - 512;
                unsigned MSX_P2 = 128 * DivB - DivC * std::floor(128 * DivB / DivC);
                unsigned MSX_P3 = DivC;

                m_newConfiguration[addr] = MSX_P3 >> 8;
                m_newConfiguration[addr + 1] = MSX_P3;

                m_newConfiguration[addr + 2] = (MSX_P1 >> 16) & 0x03;
                m_newConfiguration[addr + 3] = MSX_P1 >> 8;
                m_newConfiguration[addr + 4] = MSX_P1;

                m_newConfiguration[addr + 5] = (MSX_P2 >> 16) & 0x0F;
                m_newConfiguration[addr + 5] |= (MSX_P3 >> 16) << 4;

                m_newConfiguration[addr + 6] = MSX_P2;
                m_newConfiguration[addr + 7] = MSX_P2 >> 8;
            }
            else if (CLK[i].outputFreqHz <= 160000000) // AVAILABLE ONLY ON 0-5 MULTISYNTHS
            {
                lime::error("Si5351C - clock configuring for more than 150 MHz not implemented"s);
                return Status::FAILED;
            }
        }
        else // CLK6 and CLK7 only integer mode
        {
            if (CLK[i].outputFreqHz <= 150000000)
            {
                if (i == 6)
                {
                    m_newConfiguration[90] = DivA;
                    if (DivA % 2 != 0)
                    {
                        lime::error("Si5351C - CLK6 multisynth divider is not even integer"s);
                        return Status::FAILED;
                    }
                }
                else
                {
                    m_newConfiguration[91] = DivA;
                    if (DivA % 2 != 0)
                    {
                        lime::error("Si5351C - CLK7 multisynth divider is not even integer"s);
                        return Status::FAILED;
                    }
                }
            }
            else if (CLK[i].outputFreqHz <= 160000000) // AVAILABLE ONLY ON 0-5 MULTISYNTHS
            {
                lime::error("Si5351C - clock configuring for more than 150 MHz not implemented"s);
                return Status::FAILED;
            }
        }
    }

    //configure pll
    //set input clk source
    m_newConfiguration[15] &= 0xF3;
    m_newConfiguration[15] |= (PLL[0].CLK_SRC & 1) << 2;
    m_newConfiguration[15] |= (PLL[1].CLK_SRC & 1) << 3;
    for (int i = 0; i < 2; ++i)
    {
        addr = 26 + i * 8;
        if (PLL[i].feedbackDivider < 15 || PLL[i].feedbackDivider > 90)
        {
            lime::error("Si5351C - VCO frequency divider out of range [15:90]."s);
            return Status::FAILED;
        }
        if (PLL[i].VCO_Hz < 600000000 || PLL[i].VCO_Hz > 900000000)
        {
            lime::error("Si5351C - Can't calculate valid VCO frequency."s);
            return Status::FAILED;
        }

        //calculate MSNx_P1, MSNx_P2, MSNx_P3
        int MSNx_P1;
        int MSNx_P2;
        int MSNx_P3;

        int DivA;
        int DivB;
        int DivC;
        realToFrac(PLL[i].feedbackDivider, DivA, DivB, DivC);
        lime::info("Si5351C: VCO%s = %g MHz  Feedback Divider %d %d/%d",
            (i == 0 ? "A" : "B"),
            PLL[i].VCO_Hz / 1000000.0,
            DivA,
            DivB,
            DivC);

        MSNx_P1 = 128 * DivA + std::floor(128 * (static_cast<float>(DivB) / DivC)) - 512;
        MSNx_P2 = 128 * DivB - DivC * std::floor(128 * DivB / DivC);
        MSNx_P3 = DivC;

        m_newConfiguration[addr + 4] = MSNx_P1;
        m_newConfiguration[addr + 3] = MSNx_P1 >> 8;
        m_newConfiguration[addr + 2] = MSNx_P1 >> 16;

        m_newConfiguration[addr + 7] = MSNx_P2;
        m_newConfiguration[addr + 6] = MSNx_P2 >> 8;
        m_newConfiguration[addr + 5] = (MSNx_P2 >> 16) & 0x0F;

        m_newConfiguration[addr + 5] |= (MSNx_P3 >> 16) << 4;
        m_newConfiguration[addr + 1] |= MSNx_P3;
        m_newConfiguration[addr] |= MSNx_P3 >> 8;
    }
    return Status::SUCCESS;
}

/** @brief Sets output clock parameters
    @param id clock id 0-CLK0 1-CLK1 ...
    @param fOut_Hz output frequency in Hz
    @param enabled is this output powered
    @param inverted invert clock
*/
void Si5351C::SetClock(unsigned char id, unsigned long fOut_Hz, bool enabled, bool inverted)
{
    if (id >= 8)
    {
        return;
    }

    if (fOut_Hz < 8000 || fOut_Hz > 160000000)
    {
        lime::error("Si5351C - CLK%d output frequency must be between 8kHz and 160MHz. fOut_MHz = %g", id, fOut_Hz / 1000000.0);
        return;
    }
    CLK[id].powered = enabled;
    CLK[id].inverted = inverted;
    CLK[id].outputFreqHz = fOut_Hz;
}

/** @brief Sets PLL input frequency
    @param id PLL id 0-PLLA 1-PLLB
    @param CLKIN_Hz clock input in Hz
    @param CLK_SRC source of the clock
*/
void Si5351C::SetPLL(unsigned char id, unsigned long CLKIN_Hz, int CLK_SRC)
{
    if (id < 2)
    {
        PLL[id].inputFreqHz = CLKIN_Hz;
        PLL[id].CLK_SRC = CLK_SRC;
    }
}

/** @brief Resets configuration registers to default values
*/
void Si5351C::Reset()
{
    m_newConfiguration.fill(0);
    for (const auto& [address, value] : defaultConfiguration)
    {
        m_newConfiguration.at(address) = value;
    }
}

Si5351C::StatusBits Si5351C::GetStatusBits()
{
    StatusBits stat;
    std::vector<uint8_t> dataIo;
    dataIo.push_back(0);
    dataIo.push_back(1);

    try
    {
        comms.I2CRead(addrSi5351, dataIo.data(), 2);
    } catch (std::runtime_error& e)
    {
        return stat;
    }

    uint8_t reg0 = dataIo[0] & 0xFF;
    uint8_t reg1 = dataIo[1] & 0xFF;
    stat.sys_init = (reg0 >> 7);
    stat.lol_b = (reg0 >> 6) & 0x1;
    stat.lol_a = (reg0 >> 5) & 0x1;
    stat.los = (reg0 >> 4) & 0x1;
    stat.sys_init_stky = (reg1 >> 7);
    stat.lol_b_stky = (reg1 >> 6) & 0x1;
    stat.lol_a_stky = (reg1 >> 5) & 0x1;
    stat.los_stky = (reg1 >> 4) & 0x1;
    return stat;
}

Si5351C::Status Si5351C::ClearStatus()
{
    std::vector<uint8_t> dataWr;
    dataWr.push_back(1);
    dataWr.push_back(0x1);

    try
    {
        comms.I2CWrite(addrSi5351, dataWr.data(), dataWr.size());
        return Status::SUCCESS;
    } catch (std::runtime_error& e)
    {
        lime::error("Si5351C configuration failed "s + e.what());
        return Status::FAILED;
    }
}
