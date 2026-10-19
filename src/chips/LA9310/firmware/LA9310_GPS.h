#pragma once

#include "limesuiteng/GPS.h"

#include <memory>

namespace lime {

class LA9310_FW_Impl;

/// GPS module attached to the LA9310 UART, accessed through the M4 firmware.
class LA9310_GPS : public GPS
{
  public:
    LA9310_GPS(std::shared_ptr<LA9310_FW_Impl> fw);

    OpStatus SetPPSConfig(GPS::PPSmode availability, std::chrono::milliseconds pulseWidth) override;
    OpStatus StandByMode(bool sleep) override;
    OpStatus GetStatus(GPS::Status& status) override;

  private:
    std::shared_ptr<LA9310_FW_Impl> fw;
};

} // namespace lime
