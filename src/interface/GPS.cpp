#include "limesuiteng/GPS.h"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace lime {

static const char* LinkStateName(GPS::LinkState state)
{
    switch (state)
    {
    case GPS::LinkState::Idle:
        return "idle";
    case GPS::LinkState::Detecting:
        return "detecting baud rate";
    case GPS::LinkState::Fail:
        return "fail";
    case GPS::LinkState::Running:
        return "running";
    }
    return "unknown";
}

static const char* FixTypeName(uint8_t fixType)
{
    switch (fixType)
    {
    case 2:
        return "2D";
    case 3:
        return "3D";
    default:
        return "none";
    }
}

static const char* FixQualityName(uint8_t quality)
{
    switch (quality)
    {
    case 0:
        return "invalid";
    case 1:
        return "GPS";
    case 2:
        return "DGPS";
    case 3:
        return "PPS";
    case 4:
        return "RTK";
    case 5:
        return "float RTK";
    case 6:
        return "estimated";
    case 7:
        return "manual";
    case 8:
        return "simulation";
    default:
        return "unknown";
    }
}

std::string GPS::Status::ToString() const
{
    std::stringstream ss;
    ss << std::setfill('0');

    ss << "Link: " << LinkStateName(link);
    if (baudrate)
        ss << ", " << baudrate << " baud";
    ss << '\n';

    ss << "Fix: " << (active ? "valid" : "not valid") << ", type " << FixTypeName(fixType) << ", quality "
       << FixQualityName(fixQuality) << '\n';

    ss << "UTC: ";
    if (dateValid)
        ss << std::setw(4) << year << '-' << std::setw(2) << int(month) << '-' << std::setw(2) << int(day) << ' ';
    else
        ss << "----------- ";
    if (timeValid)
        ss << std::setw(2) << int(hour) << ':' << std::setw(2) << int(minute) << ':' << std::setw(2) << int(second) << '.'
           << std::setw(3) << millisecond;
    else
        ss << "--:--:--.---";
    ss << '\n';
    ss << std::setfill(' ');

    ss << std::fixed;
    ss << "Position: ";
    if (positionValid)
        ss << std::setprecision(7) << std::fabs(latitude) << (latitude < 0 ? " S, " : " N, ") << std::fabs(longitude)
           << (longitude < 0 ? " W" : " E");
    else
        ss << "unknown";
    ss << '\n';

    ss << "Altitude: ";
    if (altitudeValid)
        ss << std::setprecision(1) << altitude << " m (geoid separation " << geoidSeparation << " m)";
    else
        ss << "unknown";
    ss << '\n';

    ss << std::setprecision(2);
    ss << "Speed: ";
    if (speedValid)
        ss << speedKnots << " kn (" << speedKnots * 1.852 << " km/h)";
    else
        ss << "unknown";
    ss << ", course ";
    if (courseValid)
        ss << course << " deg";
    else
        ss << "unknown";
    ss << '\n';

    ss << "DOP: ";
    if (dopValid)
        ss << "P " << pdop << ", H " << hdop << ", V " << vdop;
    else
        ss << "unknown";
    ss << '\n';

    ss << "Satellites: " << int(satellitesUsed) << " used; in view GPS " << int(satellitesInView.gps) << ", GLONASS "
       << int(satellitesInView.glonass) << ", Galileo " << int(satellitesInView.galileo) << ", BeiDou "
       << int(satellitesInView.beidou) << ", QZSS " << int(satellitesInView.qzss) << '\n';

    ss << "Counters: rx " << counters.rxBytes << " B, sentences " << counters.sentencesOk << ", checksum errors "
       << counters.checksumErrors << ", framing errors " << counters.framingErrors << ", parse errors " << counters.parseErrors
       << ", UART line errors " << counters.rxLineErrors << ", rx overflows " << counters.rxBufferOverflows << '\n';
    return ss.str();
}

} // namespace lime
