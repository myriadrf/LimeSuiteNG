#ifndef LIME_OEMTESTING_H
#define LIME_OEMTESTING_H

#include <string>
#include <map>
#include "limesuiteng/OpStatus.h"

namespace lime {

class SDRDevice;

struct OEMTestData {
    OEMTestData(const std::string& name)
        : name(name)
        , passed(false){};
    virtual ~OEMTestData(){};
    std::string name;
    bool passed;
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

OpStatus RunRFTest(SDRDevice& device, const RFTestInput& input, OEMTestReporter* reporter, RFTestOutput* output = nullptr);

} // namespace lime

#endif
