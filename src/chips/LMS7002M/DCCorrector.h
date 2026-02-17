#pragma once

#include "interface/IDCCorrector.h"
#include "limesuiteng/types.h"

#include <memory>

struct lms7002m_context;

namespace lime {

class LMS7002M;
class IDCCorrector;

class LMS7002M_DC : public IDCCorrector
{
  public:
    LMS7002M_DC(lms7002m_context* rfsoc, TRXDir dir, uint32_t channel);
    OpStatus Enabled(bool enable) override;

    OpStatus SetDCOffset(lime::complex16_t offset) override;
    OpStatus SetDCI(int16_t offset) override;
    OpStatus SetDCQ(int16_t offset) override;

    complex16_t GetDCOffset() override;

    lime::Range<float> GetRange() override;
    IDCCorrector::Type GetType() const override;

  private:
    lms7002m_context* rfsoc;
    uint16_t base_reg;
};

} // namespace lime