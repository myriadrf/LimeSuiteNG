#ifndef LIME_IRFTIMING_H
#define LIME_IRFTIMING_H

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "limesuiteng/OpStatus.h"
#include "limesuiteng/types.h"

namespace lime {

/// @brief Timing of the RF front end control signals (DAC/ADC enable, PA enable, RF switches, ...) that the device
/// toggles around the start and the end of every timed burst.
///
/// Each control has two edges, one at the burst start and one at the burst end. An edge's offset is relative to the
/// burst's start/end timestamp, in device timer ticks (negative = before the burst edge), and its action is what the
/// control output does at that moment.
class IRFTiming
{
  public:
    static constexpr std::string_view DeviceTreeClass{ "RFTiming" };

    /// @brief What the control output does at an edge.
    enum class Action : uint8_t { NoChange, Low, High, Toggle };

    struct Edge {
        int32_t offset{ 0 };
        Action action{ Action::NoChange };
    };

    struct Control {
        Edge start;
        Edge end;
    };

    struct ControlDescriptor {
        std::string name; ///< Identifier used with Get/SetControl, e.g. "pa"
        std::string description;
        lime::Range<int32_t> offsetRange;
        bool sharedOffset; ///< start and end use the same offset, Set requires them to be equal
        bool configurableActions; ///< false: actions are fixed by the device, Set ignores them
    };

    virtual ~IRFTiming(){};

    /// @return The direction whose bursts this timing applies to.
    virtual TRXDir GetDirection() const = 0;

    virtual std::vector<ControlDescriptor> GetControls() const = 0;

    virtual OpStatus GetControl(const std::string& name, Control& control) = 0;
    virtual OpStatus SetControl(const std::string& name, const Control& control) = 0;
};

} // namespace lime

#endif
