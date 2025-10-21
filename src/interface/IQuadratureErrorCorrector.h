#ifndef LIME_QEC_H
#define LIME_QEC_H

#include "limesuiteng/OpStatus.h"

namespace lime {

class IQuadratureErrorCorrector
{
  public:
    virtual ~IQuadratureErrorCorrector(){};

    virtual OpStatus SetImbalance(float iq_gain_imb, float phase_imb_deg) = 0;
    virtual OpStatus SetPhaseCorrection(float phase_imb_deg) = 0;
    virtual OpStatus SetGainCorrection(float phase_imb_deg) = 0;
};

} // namespace lime

#endif