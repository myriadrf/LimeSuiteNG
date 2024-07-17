#pragma once

#include <stdint.h>
#include <memory>
#include <vector>

#include "registers.h"

#include "limesuiteng/config.h"
#include "limesuiteng/OpStatus.h"
#include "limesuiteng/types.h"

class INI;

namespace lime {

class ISPI;
class LMS7002M;

/// @brief Class for interfacing with the CrestFactorReduction module.
class LIME_API CrestFactorReduction
{
  public:
    /** @brief Structure containing the configuration of the CrestFactorReduction. */
    struct Config {
        /** @brief Crest Factor Reduction (CFR) controls. */
        struct CFR {
            bool bypass;
            bool sleep;
            bool bypassGain;
            uint8_t order;
            uint8_t interpolation;
            uint16_t threshold;
            uint16_t thresholdGain;
        } cfr[2]{};

        /** @brief Post-CFR Finite Impulse Response (FIR) information */
        struct FIR {
            bool sleep;
            bool bypass;
            int16_t coefficients[32];
            uint8_t coefficientsCount;
        } fir[2]{};
        bool bypassRxEqualizer[2]{};
        bool bypassTxEqualizer[2]{};

        Config();
    };

    CrestFactorReduction(std::shared_ptr<ISPI> comms, LMS7002M* rfsoc);
    ~CrestFactorReduction();
    void Configure(const CrestFactorReduction::Config& cfg);

    void SetOversample(uint8_t oversample);
    uint8_t GetOversample();

    lime::OpStatus WriteRegister(const Register& reg, uint16_t value);
    uint16_t ReadRegister(const Register& reg);

  public: // code ported from GUI
    void SaveRegisterRangesToIni(INI* m_options, uint8_t chipId, const std::vector<Range<uint16_t>>& ranges);
    void FDQIE_SaveEqualiser(int Ntaps, const std::string& m_sConfigFilename);
    void FDQIE_LoadEqualiser(const std::string& m_sConfigFilename);
    void FDQIE_ResetEqualiser(int kk);
    void UpdateHannCoeff(uint16_t Filt_N);

    //private:
    void LoadCFRFIR(const std::string& m_sConfigFilename);
    void SaveCFRFIR(const std::string& m_sConfigFilename);
    void FDQIE_SaveDC(double RefClk, const std::string& m_sConfigFilename);
    void FDQIE_LoadDC(const std::string& m_sConfigFilename);
    void FDQIE_SetupTransmitterDC(int16_t codeI, int16_t codeQ, int kk);
    void FDQIE_SetupReceiverDC(lime::LMS7002M* rfsoc, int16_t codeI, int16_t codeQ, int kk);

  private:
    std::shared_ptr<ISPI> m_Comms;
    lime::LMS7002M* lms2;

    void SetFIRCoefficients(const int16_t* coefficients, uint16_t count);
};

} // namespace lime
