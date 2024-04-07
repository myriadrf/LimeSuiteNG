#ifndef LIMESUITENG_SDRCONFIG_H
#define LIMESUITENG_SDRCONFIG_H

#include "limesuiteng/commonTypes.h"
#include "limesuiteng/complex.h"
#include "limesuiteng/GainTypes.h"

#include <unordered_map>

namespace lime {

/// @brief Configuration of a single channel.
struct ChannelConfig {
    ChannelConfig()
        : rx()
        , tx()
    {
    }

    /// @brief Configuration for a direction in a channel.
    struct Direction {
        Direction()
            : centerFrequency(0)
            , NCOoffset(0)
            , sampleRate(0)
            , lpf(0)
            , path(0)
            , oversample(0)
            , gfir()
            , enabled(false)
            , calibrate(false)
            , testSignal{ false, false, TestSignal::Divide::Div8, TestSignal::Scale::Half }
        {
        }

        /// @brief Configuration of a general finite impulse response (FIR) filter.
        struct GFIRFilter {
            bool enabled; ///< Whether the filter is enabled or not.
            double bandwidth; ///< The bandwidth of the filter (in Hz).
        };

        /// @brief The structure holding the status of the test signal the device can produce.
        struct TestSignal {
            /// @brief The enumeration describing the divide mode of the test signal.
            enum class Divide : uint8_t { Div8, Div4 };

            /// @brief The enumeration describing the scale of the test signal.
            enum class Scale : uint8_t { Full, Half };

            complex16_t dcValue; ///< The value to use when in DC mode.
            Divide divide; ///< The current divide of the test signal.
            Scale scale; ///< The current scale of the test signal.
            bool enabled; ///< Denotes whether test mode is enabled or not.
            bool dcMode; ///< The DC mode of the test mode.

            /// @brief The constructor for the Test Signal storage class.
            /// @param enabled Whether the test signal is enabled or not.
            /// @param dcMode Whether the DC mode is enabled or not.
            /// @param divide The divide mode of the test signal.
            /// @param scale The scale of the dest signal.
            TestSignal(bool enabled = false, bool dcMode = false, Divide divide = Divide::Div8, Scale scale = Scale::Half)
                : dcValue(0, 0)
                , divide(divide)
                , scale(scale)
                , enabled(enabled)
                , dcMode(dcMode)
            {
            }
        };

        double centerFrequency; ///< The center frequency of the direction of this channel (in Hz).
        double NCOoffset; ///< The offset from the channel's numerically controlled oscillator (NCO) (in Hz).
        double sampleRate; ///< The sample rate of this direction of a channel (in Hz).
        std::unordered_map<eGainTypes, double> gain; ///< The gains and their current values for this direction.
        double lpf; ///< The bandwidth of the Low Pass Filter (LPF) (in Hz).
        uint8_t path; ///< The antenna being used for this direction.
        uint8_t oversample; ///< The oversample ratio of this direction.
        GFIRFilter gfir; ///< The general finite impulse response (FIR) filter settings of this direction.
        bool enabled; ///< Denotes whether this direction of a channel is enabled or not.
        bool calibrate; ///< Denotes whether the device will be calibrated or not.
        TestSignal testSignal; ///< Denotes whether the signal being sent is a test signal or not.
    };

    /// @brief Gets the reference to the direction settings.
    /// @param direction The direction to get it for.
    /// @return The reference to the direction.
    Direction& GetDirection(TRXDir direction)
    {
        switch (direction)
        {
        case TRXDir::Rx:
            return rx;
        case TRXDir::Tx:
            return tx;
        }
    }

    /// @brief Gets the const reference to the direction settings.
    /// @param direction The direction to get it for.
    /// @return The const reference to the direction.
    const Direction& GetDirection(TRXDir direction) const
    {
        switch (direction)
        {
        case TRXDir::Rx:
            return rx;
        case TRXDir::Tx:
            return tx;
        }
    }

    Direction rx; ///< Configuration settings for the Receive channel.
    Direction tx; ///< Configuration settings for the Transmit channel.
};

/// @brief Configuration of an SDR device.
struct SDRConfig {
    static constexpr uint8_t MAX_CHANNEL_COUNT = 16; ///< Maximum amount of channels an SDR Device can hold
    SDRConfig()
        : referenceClockFreq(0)
        , skipDefaults(false){};
    double referenceClockFreq; ///< The reference clock frequency of the device.
    ChannelConfig channel[MAX_CHANNEL_COUNT]; ///< The configuration settings for each of the channels.
    // Loopback setup?
    bool skipDefaults; ///< Skip default values initialization and write on top of current config.
};

} // namespace lime
#endif //LIMESUITENG_SDRCONFIG_H
