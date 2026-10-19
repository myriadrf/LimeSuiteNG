#include "LA9310_GPS.h"

#include "LA9310_FW_Impl.h"
#include "m4_commands.h"
#include "gps_data.h"

#include <string>

namespace lime {

LA9310_GPS::LA9310_GPS(std::shared_ptr<LA9310_FW_Impl> fw)
    : fw(fw)
{
}

OpStatus LA9310_GPS::SetPPSConfig(GPS::PPSmode availability, std::chrono::milliseconds pulseWidth)
{
    const std::string body =
        "PMTK285," + std::to_string(static_cast<uint32_t>(availability)) + "," + std::to_string(pulseWidth.count());
    return fw->SendGPSSentence(body);
}

OpStatus LA9310_GPS::StandByMode(bool sleep)
{
    // Any UART activity wakes the module, the test command is harmless.
    return fw->SendGPSSentence(sleep ? "PMTK161,0" : "PMTK000");
}

OpStatus LA9310_GPS::GetStatus(GPS::Status& status)
{
    gps_status_payload payload{};
    OpStatus result = fw->GetGPSStatus(payload);
    if (result != OpStatus::Success)
        return result;

    status = GPS::Status{};
    switch (payload.link_state)
    {
    case GPS_LINK_DETECTING:
        status.link = LinkState::Detecting;
        break;
    case GPS_LINK_FAIL:
        status.link = LinkState::Fail;
        break;
    case GPS_LINK_RUNNING:
        status.link = LinkState::Running;
        break;
    default:
        status.link = LinkState::Idle;
        break;
    }
    status.baudrate = payload.baudrate;

    const gps_data_aggregate_t& data = payload.data;
    status.active = data.active;
    status.timeValid = data.valid & GPS_DATA_VALID_TIME;
    status.dateValid = data.valid & GPS_DATA_VALID_DATE;
    status.positionValid = data.valid & GPS_DATA_VALID_POSITION;
    status.altitudeValid = data.valid & GPS_DATA_VALID_ALTITUDE;
    status.speedValid = data.valid & GPS_DATA_VALID_SPEED;
    status.courseValid = data.valid & GPS_DATA_VALID_COURSE;
    status.dopValid = data.valid & GPS_DATA_VALID_DOP;

    status.year = data.date.year;
    status.month = data.date.month;
    status.day = data.date.day;
    status.hour = data.time.hour;
    status.minute = data.time.minute;
    status.second = data.time.second;
    status.millisecond = data.time.millisecond;

    status.latitude = data.position.latitude_1e7 / 1e7;
    status.longitude = data.position.longitude_1e7 / 1e7;
    status.altitude = data.altitude_mm / 1e3;
    status.geoidSeparation = data.geoid_separation_mm / 1e3;
    status.speedKnots = data.speed_knots_milli / 1e3;
    status.course = data.course_centideg / 1e2;
    status.pdop = data.pdop_centi / 1e2;
    status.hdop = data.hdop_centi / 1e2;
    status.vdop = data.vdop_centi / 1e2;

    status.fixQuality = data.fix_quality;
    status.fixType = data.fix_type;
    status.satellitesUsed = data.satellites_used;
    status.satellitesInView.gps = data.satellites_in_view[NMEA_SYSTEM_GPS];
    status.satellitesInView.glonass = data.satellites_in_view[NMEA_SYSTEM_GLONASS];
    status.satellitesInView.galileo = data.satellites_in_view[NMEA_SYSTEM_GALILEO];
    status.satellitesInView.beidou = data.satellites_in_view[NMEA_SYSTEM_BEIDOU];
    status.satellitesInView.qzss = data.satellites_in_view[NMEA_SYSTEM_QZSS];

    status.counters.rxBytes = payload.stats.rx_bytes;
    status.counters.rxLineErrors = payload.stats.rx_line_errors;
    status.counters.rxBufferOverflows = payload.stats.rx_buffer_overflows;
    status.counters.sentencesOk = payload.stats.sentences_ok;
    status.counters.checksumErrors = payload.stats.checksum_errors;
    status.counters.framingErrors = payload.stats.framing_errors;
    status.counters.parseErrors = payload.stats.parse_errors;
    return OpStatus::Success;
}

} // namespace lime
