#ifndef LIMESUITENG_IQUADRATUREERRORCORRECTOR_H
#define LIMESUITENG_IQUADRATUREERRORCORRECTOR_H

#include "limesuiteng/OpStatus.h"
#include "limesuiteng/types.h"

namespace lime {

class IQuadratureErrorCorrector
{
  public:
    virtual ~IQuadratureErrorCorrector(){};

    /// @brief Enables or bypasses both the gain and the phase correction.
    virtual OpStatus Enabled(bool enable) = 0;
    /// @return Undetermined if only one of the gain and phase corrections is enabled.
    virtual TriState IsEnabled() = 0;

    virtual OpStatus SetImbalance(float iq_gain_imb, float phase_imb_deg) = 0;
    virtual OpStatus SetPhaseCorrection(float phase_imb_deg) = 0;
    virtual OpStatus SetGainCorrection(float phase_imb_deg) = 0;

    /// @brief Reads the current correction, in the same units as SetImbalance().
    /// @param iq_gain_imb Output: the IQ gain imbalance correction (dB).
    /// @param phase_imb_deg Output: the phase imbalance correction (degrees).
    /// @return The status of the operation. The outputs are left unchanged if it fails.
    virtual OpStatus GetImbalance(float& iq_gain_imb, float& phase_imb_deg) = 0;

    virtual lime::Range<float> GetGainRange() = 0;
    virtual lime::Range<float> GetPhaseRange() = 0;
};

} // namespace lime

#endif