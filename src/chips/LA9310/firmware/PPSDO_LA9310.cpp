#include "PPSDO_LA9310.h"

#include "LA9310_FW_Impl.h"
#include "m4_commands.h"
#include "limesuiteng/Logger.h"

#include <cmath>
#include <limits>

namespace lime {

PPSDO_LA9310::PPSDO_LA9310(std::shared_ptr<LA9310_FW_Impl> fw)
    : fw(fw)
{
}

OpStatus PPSDO_LA9310::Configure(const PPSDO::Config& config)
{
    const double tolerance_ppb = std::round(config.tolerancePPM * 1000);
    if (tolerance_ppb < 0 || tolerance_ppb > std::numeric_limits<uint32_t>::max())
        return OpStatus::InvalidValue;

    // The firmware measures the PHY timer, its frequency is set by the firmware itself.
    if (config.clockFrequency != 0)
        lime::warning("PPSDO: clock frequency is determined by the M4 firmware, ignoring %g Hz", config.clockFrequency);

    return fw->ConfigurePPSDO(static_cast<uint32_t>(tolerance_ppb));
}

OpStatus PPSDO_LA9310::Enable(bool enable)
{
    return fw->EnablePPSDO(enable);
}

OpStatus PPSDO_LA9310::GetStatus(PPSDO::Status& status)
{
    ppsdo_status_payload payload{};
    OpStatus result = fw->GetPPSDOStatus(payload);
    if (result != OpStatus::Success)
        return result;

    status = PPSDO::Status{};
    status.enabled = payload.enabled;
    switch (payload.tune_state)
    {
    case PPSDO_TUNE_STATE_COARSE_MIN:
    case PPSDO_TUNE_STATE_COARSE_MAX:
    case PPSDO_TUNE_STATE_COARSE_DONE:
        status.state = TuneState::CoarseTune;
        break;
    case PPSDO_TUNE_STATE_FINE:
        status.state = TuneState::FineTune;
        break;
    case PPSDO_TUNE_STATE_IDLE:
        status.state = TuneState::Idle;
        break;
    default:
        status.state = TuneState::Unknown;
        break;
    }
    status.accuracy = payload.accuracy;
    status.ppsActive = payload.pps_active;
    status.error1s = payload.error_1s;
    status.error10s = payload.error_10s;
    status.error100s = payload.error_100s;
    status.dacValue = payload.xo_dac_value;
    return OpStatus::Success;
}

} // namespace lime
