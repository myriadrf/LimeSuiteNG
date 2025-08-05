#ifndef LIME_OEMTESTING_H
#define LIME_OEMTESTING_H

#include <string>
#include <map>
#include <vector>
#include "limesuiteng/OpStatus.h"
#include "limesuiteng/complex.h"

namespace lime {

class SDRDevice;

struct OEMTestData {
    OEMTestData(const std::string& title)
        : title(title)
        , status(OpStatus::InvalidValue){};
    virtual ~OEMTestData(){};
    std::string title;
    std::string output;
    OpStatus status;

    struct Measurement {
        std::string title;
        std::string value;
    };
    std::vector<Measurement> measurements;
};

class OEMTestReporter
{
  public:
    virtual ~OEMTestReporter(){};
    virtual void OnStart(OEMTestData& test, const std::string& testName = std::string()) = 0;
    virtual void OnStepUpdate(OEMTestData& test, const std::string& text = std::string()) = 0;
    virtual void OnSuccess(OEMTestData& test) = 0;
    virtual void OnFail(OEMTestData& test, const std::string& reasonText = std::string()) = 0;
    virtual void ReportColumn(const std::string& header, const std::string& value) = 0;
};

struct RFTestInput {
    std::string testName;
    float rfTestTolerance_dB;
    float rfTestTolerance_Hz;
    float sampleRate;
    float expectedPeakval_dBFS;
    float expectedPeakFrequency;
    int moduleIndex;
    int channelIndex;
};

struct RFTestOutput {
    float amplitude_dBFS;
    float frequency;
};

template<class T> bool IsWithinTolerance(T value, T expectedValue, T maxDeviation)
{
    return (std::abs(expectedValue - value) <= maxDeviation);
}

OpStatus CaptureRxSamples(
    SDRDevice& device, uint8_t chipIndex, uint8_t channelIndex, std::vector<lime::complex32f_t>& samples, size_t count);
void CalculateSignalPeak(const std::vector<lime::complex32f_t>& samples, double& peakAmplitude_dBFS, double& nyquistWeight);

} // namespace lime

#endif
