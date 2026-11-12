#ifndef LIME_LMS7002M_MODULES_H
#define LIME_LMS7002M_MODULES_H

#include "limesuiteng/ILPF.h"
#include "limesuiteng/INCO.h"
#include "limesuiteng/IGFIR.h"
#include "limesuiteng/ILOMixer.h"
#include "limesuiteng/IRFChannel.h"
#include "limesuiteng/types.h"
#include "limesuiteng/IOversampler.h"
#include "limesuiteng/IQuadratureErrorCorrector.h"
#include "limesuiteng/IToneGenerator.h"
#include "limesuiteng/IRFPath.h"
#include "limesuiteng/IGainStage.h"
#include "limesuiteng/ISensor.h"

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace lime {

class LMS7002M;

/// @brief RFE input (Rx) or TRF band (Tx) selection of one LMS7002M channel.
class LMS7002M_RFPath : public IRFPath
{
  public:
    LMS7002M_RFPath(LMS7002M* chip, TRXDir dir, uint8_t channel);

    TRXDir GetDirection() const override { return mDir; }
    std::vector<PathDescriptor> GetPaths() const override;
    OpStatus SetPath(const std::string& name) override;
    std::string GetPath() override;

  private:
    LMS7002M* mChip;
    TRXDir mDir;
    uint8_t mChannel;
};

/// @brief RBB/TBB low pass filter of one LMS7002M channel.
class LMS7002M_LPF : public ILPF
{
  public:
    LMS7002M_LPF(LMS7002M* chip, TRXDir dir, uint8_t channel);

    TRXDir GetDirection() const override { return mDir; }
    OpStatus SetBandwidth(double bandwidth_Hz) override;
    double GetBandwidth() override;
    Range<double> GetBandwidthRange() const override;
    TriState IsEnabled() override;

  private:
    LMS7002M* mChip;
    TRXDir mDir;
    uint8_t mChannel;
    double mBandwidth_Hz; ///< The chip does not allow reading it back, so the last set value is kept.
};

/// @brief RxTSP/TxTSP CMIX NCO of one LMS7002M channel.
class LMS7002M_NCO : public INCO
{
  public:
    LMS7002M_NCO(LMS7002M* chip, TRXDir dir, uint8_t channel);

    TRXDir GetDirection() const override { return mDir; }
    OpStatus Enable(bool enable) override;
    TriState IsEnabled() override;

    uint8_t GetPresetCount() const override { return 16; }
    OpStatus SelectPreset(uint8_t index) override;
    uint8_t GetSelectedPreset() override;

    Range<double> GetFrequencyRange() override;
    OpStatus SetFrequency(uint8_t index, double frequency_Hz) override;
    double GetFrequency(uint8_t index) override;

    OpStatus SetPhaseOffset(uint8_t index, double phase_deg) override;
    double GetPhaseOffset(uint8_t index) override;

  private:
    LMS7002M* mChip;
    TRXDir mDir;
    uint8_t mChannel;
};

/// @brief One of the three RxTSP/TxTSP general purpose FIR filters of one LMS7002M channel.
class LMS7002M_GFIR : public IGFIR
{
  public:
    LMS7002M_GFIR(LMS7002M* chip, TRXDir dir, uint8_t channel, uint8_t index);

    TRXDir GetDirection() const override { return mDir; }
    OpStatus Enable(bool enable) override;
    TriState IsEnabled() override;

    std::size_t GetMaxCoefficientCount() override;
    OpStatus SetCoefficients(std::span<const double> coefficients) override;
    OpStatus GetCoefficients(std::vector<double>& coefficients) override;

  private:
    LMS7002M* mChip;
    TRXDir mDir;
    uint8_t mChannel;
    uint8_t mIndex;
};

/// @brief SXR/SXT synthesizer of the LMS7002M, shared by both channels.
class LMS7002M_LOMixer : public ILOMixer
{
  public:
    LMS7002M_LOMixer(LMS7002M* chip, TRXDir dir);

    TRXDir GetDirection() const override { return mDir; }
    Range<double> GetFrequencyRange() const override;
    OpStatus SetFrequency(double frequency_Hz) override;
    double GetFrequency() override;
    bool IsLocked() override;

  private:
    LMS7002M* mChip;
    TRXDir mDir;
};

/// @brief RxTSP HBD / TxTSP HBI decimation/interpolation of the LMS7002M.
/// The TSP clock divider is common to both channels, so one oversampler is shared by both channels of a direction.
class LMS7002M_Oversampler : public IOversampler
{
  public:
    LMS7002M_Oversampler(LMS7002M* chip, TRXDir dir);

    /// @param oversample_pow2 0 (bypass) to 5 (32x).
    OpStatus SetOversample(uint32_t ratio) override;
    uint32_t GetOversample() override;
    uint32_t GetMaxOversample() override;

  private:
    LMS7002M* mChip;
    TRXDir mDir;
};

/// @brief RxTSP/TxTSP IQ gain (GCORRI/GCORRQ) and phase (IQCORR) correction of one LMS7002M channel.
///
/// Gain correction in dB: positive values attenuate the I branch, negative values attenuate the Q branch.
/// Phase correction is in degrees, written to IQCORR.
class LMS7002M_QEC : public IQuadratureErrorCorrector
{
  public:
    LMS7002M_QEC(LMS7002M* chip, TRXDir dir, uint8_t channel);

    /// @brief Controls the GC_BYP and PH_BYP bypasses.
    OpStatus Enabled(bool enable) override;
    TriState IsEnabled() override;

    OpStatus SetImbalance(float iq_gain_imb, float phase_imb_deg) override;
    OpStatus SetPhaseCorrection(float phase_imb_deg) override;
    OpStatus SetGainCorrection(float iq_gain_imb) override;
    OpStatus GetImbalance(float& iq_gain_imb, float& phase_imb_deg) override;

    lime::Range<float> GetGainRange() override;
    lime::Range<float> GetPhaseRange() override;

  private:
    LMS7002M* mChip;
    TRXDir mDir;
    uint8_t mChannel;
};

/// @brief RxTSP/TxTSP test signal generator of one LMS7002M channel.
///
/// The hardware supports a limited set of tones:
/// - frequency (relative to the TSP sample rate): 0 (DC), +-1/8 or +-1/4. Negative frequencies swap I and Q.
/// - amplitude: full scale or -6 dB for tones (the nearest is used), any value in [-1; 1] for DC.
class LMS7002M_ToneGenerator : public IToneGenerator
{
  public:
    LMS7002M_ToneGenerator(LMS7002M* chip, TRXDir dir, uint8_t channel);

    OpStatus Enabled(bool enabled) override;
    OpStatus SetAmplitude(double amplitude) override;
    OpStatus SetFrequency(double freq_weight) override;

  private:
    OpStatus Apply();

    LMS7002M* mChip;
    TRXDir mDir;
    uint8_t mChannel;
    double mAmplitude;
    double mFrequencyWeight;
};

/// @brief A sensor whose value is read by calling the given function.
class LMS7002M_Sensor : public ISensor
{
  public:
    /// @brief The function that reads the sensor value.
    using ReadFunction = std::function<OpStatus(double& value)>;

    LMS7002M_Sensor(std::string name, Units units, ReadFunction read);

    const std::string& GetName() const override { return mName; }
    Units GetUnits() const override { return mUnits; }
    OpStatus GetValue(double& value) override { return mRead(value); }

  private:
    std::string mName;
    Units mUnits;
    ReadFunction mRead;
};

/// @brief One of the LMS7002M gain stages (LNA, TIA, PGA, PAD, ...) of a single channel.
class LMS7002M_GainStage : public IGainStage
{
  public:
    LMS7002M_GainStage(
        LMS7002M* chip, TRXDir dir, uint8_t channel, eGainTypes type, std::string name, Units units, Range<double> range);

    const std::string& GetName() const override { return mName; }
    Units GetUnits() const override { return mUnits; }
    Range<double> GetRange() const override { return mRange; }

    OpStatus SetValue(double value) override;
    OpStatus GetValue(double& value) override;

  private:
    LMS7002M* mChip;
    TRXDir mDir;
    uint8_t mChannel;
    std::string mName;
    Units mUnits;
    Range<double> mRange;
    eGainTypes mType;
};

/// @brief One Rx or Tx channel of the LMS7002M.
class LMS7002M_RFChannel : public IRFChannel
{
  public:
    static constexpr uint8_t GFIR_COUNT = 3;

    LMS7002M_RFChannel(
        LMS7002M* chip, TRXDir dir, uint8_t index, std::shared_ptr<ILOMixer> loMixer, std::shared_ptr<IOversampler> oversampler);
    ~LMS7002M_RFChannel() override;

    TRXDir GetDirection() const override { return mDir; }
    uint8_t GetIndex() const override { return mIndex; }
    TriState IsEnabled() override;

    IRFPath* GetRFPath() override { return mRFPath.get(); }
    ILPF* GetLPF() override { return mLPF.get(); }
    INCO* GetNCO() override { return mNCO.get(); }

    uint8_t GetGFIRCount() const override { return GFIR_COUNT; }
    IGFIR* GetGFIR(uint8_t index) override;

    uint8_t GetGainStageCount() const override { return static_cast<uint8_t>(mGainStages.size()); }
    IGainStage* GetGainStage(uint8_t index) override;

    ILOMixer* GetLOMixer() override { return mLOMixer.get(); }
    IDCCorrector* GetDCCorrector() override;
    IOversampler* GetOversampler() override { return mOversampler.get(); }
    IQuadratureErrorCorrector* GetQuadratureErrorCorrector() override { return mQEC.get(); }
    IToneGenerator* GetToneGenerator() override { return mToneGenerator.get(); }

  private:
    LMS7002M* mChip;
    TRXDir mDir;
    uint8_t mIndex;
    std::unique_ptr<LMS7002M_RFPath> mRFPath;
    std::unique_ptr<LMS7002M_LPF> mLPF;
    std::unique_ptr<LMS7002M_NCO> mNCO;
    std::array<std::unique_ptr<LMS7002M_GFIR>, GFIR_COUNT> mGFIR;
    std::vector<std::unique_ptr<LMS7002M_GainStage>> mGainStages;
    std::unique_ptr<IDCCorrector> mDC;
    std::unique_ptr<LMS7002M_QEC> mQEC;
    std::unique_ptr<LMS7002M_ToneGenerator> mToneGenerator;
    std::shared_ptr<ILOMixer> mLOMixer; ///< Shared by both channels
    std::shared_ptr<IOversampler> mOversampler; ///< Shared by both channels
};

} // namespace lime

#endif
