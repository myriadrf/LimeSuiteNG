#pragma once

#include "limesuiteng/PPSDO.h"

#include <memory>

namespace lime {

class LA9310_FW_Impl;

/// PPSDO implemented by the LA9310 M4 firmware, disciplines the on-board XO to the PPS input.
class PPSDO_LA9310 : public PPSDO
{
  public:
    PPSDO_LA9310(std::shared_ptr<LA9310_FW_Impl> fw);

    OpStatus Configure(const PPSDO::Config& config) override;
    OpStatus Enable(bool enable) override;
    OpStatus GetStatus(PPSDO::Status& status) override;

  private:
    std::shared_ptr<LA9310_FW_Impl> fw;
};

} // namespace lime
