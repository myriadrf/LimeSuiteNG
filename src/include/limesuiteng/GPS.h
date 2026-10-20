#pragma once

#include "limesuiteng/config.h"
#include "limesuiteng/OpStatus.h"

#include <chrono>
#include <cstdint>
#include <string>

namespace lime {

class LIME_API GPS
{
  public:
    virtual ~GPS(){};

    enum class PPSmode { Disable = 0, AfterFirstFix = 1, Only3DFix, Only2D3DFix, Always };
    virtual OpStatus SetPPSConfig(PPSmode availability, std::chrono::milliseconds pulseWidth) = 0;
    virtual OpStatus StandByMode(bool sleep) = 0;

    enum class LinkState { Idle, Detecting, Fail, Running };

    struct LIME_API Status {
        LinkState link{ LinkState::Idle };
        uint32_t baudrate{ 0 }; ///< Detected module UART baud rate, 0 if not detected.

        // UTC
        uint16_t year{ 0 };
        uint8_t month{ 0 };
        uint8_t day{ 0 };
        uint8_t hour{ 0 };
        uint8_t minute{ 0 };
        uint8_t second{ 0 };
        uint16_t millisecond{ 0 };

        double latitude{ 0 }; ///< Degrees, positive north.
        double longitude{ 0 }; ///< Degrees, positive east.
        double altitude{ 0 }; ///< Meters above mean sea level.
        double geoidSeparation{ 0 }; ///< Meters.
        double speedKnots{ 0 };
        double course{ 0 }; ///< Degrees, true.
        double pdop{ 0 };
        double hdop{ 0 };
        double vdop{ 0 };

        uint8_t fixQuality{ 0 }; ///< GGA: 0 invalid, 1 GPS, 2 DGPS, 6 estimated.
        uint8_t fixType{ 0 }; ///< GSA: 1 none, 2 2D, 3 3D.
        uint8_t satellitesUsed{ 0 };

        struct SatellitesInView {
            uint8_t gps{ 0 };
            uint8_t glonass{ 0 };
            uint8_t galileo{ 0 };
            uint8_t beidou{ 0 };
            uint8_t qzss{ 0 };
        } satellitesInView;

        struct Counters {
            uint32_t rxBytes{ 0 };
            uint32_t rxLineErrors{ 0 }; ///< UART framing, parity, overrun and break errors.
            uint32_t rxBufferOverflows{ 0 };
            uint32_t sentencesOk{ 0 };
            uint32_t checksumErrors{ 0 };
            uint32_t framingErrors{ 0 };
            uint32_t parseErrors{ 0 };
        } counters;

        bool active{ false }; ///< Receiver reports valid navigation data (RMC status 'A').

        // Whether the latest sentence carrying a section had its fields filled, the values are zero otherwise.
        bool timeValid{ false };
        bool dateValid{ false };
        bool positionValid{ false };
        bool altitudeValid{ false }; ///< Altitude and geoid separation.
        bool speedValid{ false };
        bool courseValid{ false };
        bool dopValid{ false };

        /// Multi-line human readable summary.
        std::string ToString() const;
    };
    virtual OpStatus GetStatus(Status& status) = 0;
};

} // namespace lime
