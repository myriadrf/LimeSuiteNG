#include "LMS7002M_validation.h"

#include <string>
#include <cstdarg>
#include "limesuiteng/SDRDevice.h"
#include "CommonFunctions.h"

namespace lime {

// Verify and configure given settings
// throw logic_error with description why the config is not possible
static inline bool InRange(double val, double min, double max)
{
    return val >= min ? val <= max : false;
}

bool LMS7002M_Validate(const SDRConfig& cfg, std::vector<std::string>& errors, const int lms7002_chCount)
{
    errors.clear();

    // channel count check
    for (int i = lms7002_chCount; i < SDRConfig::MAX_CHANNEL_COUNT; ++i)
    {
        if (cfg.channel[i].rx.enabled)
            errors.push_back(strFormat("Rx channel %i enabled, LMS7002M only has %i channels", i, lms7002_chCount));
        if (cfg.channel[i].tx.enabled)
            errors.push_back(strFormat("Tx channel %i enabled, LMS7002M only has %i channels", i, lms7002_chCount));
    }

    // MIMO necessary checks
    const ChannelConfig& chA = cfg.channel[0];
    const ChannelConfig& chB = cfg.channel[1];
    const bool rxMIMO = chA.rx.enabled && chB.rx.enabled;
    const bool txMIMO = chA.tx.enabled && chB.tx.enabled;
    if (rxMIMO || txMIMO)
    {
        // MIMO sample rates have to match
        if (rxMIMO && chA.rx.sampleRate != chB.rx.sampleRate)
            errors.push_back(
                strFormat("MIMO: Rx channels sample rates do not match A(%g), B(%g)", chA.rx.sampleRate, chB.rx.sampleRate));
        if (txMIMO && chA.tx.sampleRate != chB.tx.sampleRate)
            errors.push_back(
                strFormat("MIMO: Tx channels sample rates do not match A(%g), B(%g)", chA.tx.sampleRate, chB.tx.sampleRate));

        // LMS7002M MIMO A&B channels share LO, but can be offset by NCO
        // TODO: check if they are withing NCO range
        const double rxLOdiff = chA.rx.centerFrequency - chB.rx.centerFrequency;
        if (rxMIMO && rxLOdiff > 0)
            errors.push_back(
                strFormat("MIMO: Rx channels center frequency do not match A(%g), B(%g)", chA.rx.sampleRate, chB.rx.sampleRate));
        const double txLOdiff = chA.tx.centerFrequency - chB.tx.centerFrequency;
        if (txMIMO && txLOdiff > 0)
            errors.push_back(
                strFormat("MIMO: Tx channels center frequency do not match A(%g), B(%g)", chA.tx.sampleRate, chB.tx.sampleRate));
    }

    // individual channel validation
    const double minLO = 30e6; // LO can be lowest 30e6, 100e3 could be achieved using NCO
    const double maxLO = 3.8e9;
    for (int i = 0; i < 2; ++i)
    {
        const ChannelConfig& ch = cfg.channel[i];
        if (ch.rx.enabled && !InRange(ch.rx.centerFrequency, minLO, maxLO))
            errors.push_back(strFormat("Rx ch%i LO (%g) out of range [%g:%g]", i, ch.rx.centerFrequency, minLO, maxLO));
        if (ch.tx.enabled && !InRange(ch.tx.centerFrequency, minLO, maxLO))
            errors.push_back(strFormat("Tx ch%i LO (%g) out of range [%g:%g]", i, ch.tx.centerFrequency, minLO, maxLO));

        // TODO: make warning if rx loopback selection does not work with tx output
        if (ch.rx.enabled && !InRange(ch.rx.path, 0, 5))
            errors.push_back(strFormat("Rx ch%i invalid path(%i)", i, ch.rx.path));

        if (ch.tx.enabled && !InRange(ch.tx.path, 0, 2))
            errors.push_back(strFormat("Tx ch%i invalid path(%i)", i, ch.tx.path));

        if (std::abs(ch.rx.NCOoffset) > ch.rx.sampleRate / 2)
            errors.push_back(strFormat(
                "Rx ch%i NCO offset (%g) must be less than half of sample rate(%g)", i, ch.rx.NCOoffset, ch.rx.sampleRate));
        if (std::abs(ch.tx.NCOoffset) > ch.tx.sampleRate / 2)
            errors.push_back(strFormat(
                "Tx ch%i NCO offset (%g) must be less than half of sample rate(%g)", i, ch.tx.NCOoffset, ch.tx.sampleRate));
    }

    return errors.empty();
}

} // namespace lime
