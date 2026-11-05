#ifndef LIMESUITENG_IRFPATH_H
#define LIMESUITENG_IRFPATH_H

#include "limesuiteng/OpStatus.h"
#include "limesuiteng/types.h"

#include <cstdint>
#include <string>
#include <vector>

namespace lime {

/// @brief Interface for configuring RF switch and antenna paths
class IRFPath
{
  public:
    struct PathDescriptor {
        std::string name; ///< e.g. "LNAH", "Band1"
        Range<double> frequencyRange; ///< Recommended RF frequency range, min == max == 0 if not applicable (e.g. loopback).
    };

    virtual ~IRFPath() = default;

    /// @return The direction of the signal path.
    virtual TRXDir GetDirection() const = 0;

    /// @return The available paths.
    virtual std::vector<PathDescriptor> GetPaths() const = 0;

    /// @param name The name of one of the paths from GetPaths().
    /// @return OpStatus::InvalidValue if there is no path with such name.
    virtual OpStatus SetPath(const std::string& name) = 0;
    /// @return The name of the currently selected path, empty if it could not be determined.
    virtual std::string GetPath() = 0;
};

} // namespace lime

#endif
