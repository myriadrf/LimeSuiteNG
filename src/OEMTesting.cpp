#include "OEMTesting.h"

#include "limesuiteng/OpStatus.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/StreamConfig.h"
#include "DSP/FFT/FFT.h"

namespace lime {

OpStatus RunRFTest(SDRDevice& device, const RFTestInput& input, OEMTestReporter* reporter, RFTestOutput* output)
{
    OEMTestData test(input.testName);
    reporter->OnStart(test);

    StreamConfig stream;
    stream.channels.at(TRXDir::Rx).push_back(input.channelIndex);
    stream.format = DataFormat::F32;
    stream.linkFormat = DataFormat::I16;

    device.StreamSetup(stream, input.moduleIndex);
    device.StreamStart(input.moduleIndex);

    //Receive samples
    const int fftSize = 8192;
    std::vector<lime::complex32f_t> samples(fftSize);
    lime::complex32f_t* dest[2] = { samples.data(), nullptr };

    // ignore first batch of samples, just in case there would be instability from digital DC corrector
    device.StreamRx(input.moduleIndex, reinterpret_cast<lime::complex32f_t**>(&dest), fftSize, nullptr);
    device.StreamRx(input.moduleIndex, reinterpret_cast<lime::complex32f_t**>(&dest), fftSize, nullptr);

    device.StreamStop(input.moduleIndex);

    std::vector<float> bins = FFT::Calc(samples);
    FFT::ConvertToDBFS(bins);

    float samplerate = input.sampleRate;
    float peakAmplitude = -1000, peakFrequency = 0;

    for (int i = 1; i < fftSize; ++i) // skip DC bin
    {
        if (bins[i] > peakAmplitude)
        {
            peakAmplitude = bins[i];
            peakFrequency = i * samplerate / fftSize;
        }
    }

    if (peakFrequency > samplerate / 2)
        peakFrequency = peakFrequency - samplerate;
    if (output)
    {
        output->amplitude_dBFS = peakAmplitude;
        output->frequency = peakFrequency;
    }

    if ((peakAmplitude > input.expectedPeakval_dBFS - input.rfTestTolerance_dB) &&
        (peakAmplitude < input.expectedPeakval_dBFS + input.rfTestTolerance_dB) &&
        (peakFrequency < input.expectedPeakFrequency + input.rfTestTolerance_Hz) &&
        (peakFrequency > input.expectedPeakFrequency - input.rfTestTolerance_Hz))
    {
        char ctemp[512];
        sprintf(ctemp,
            "RF OK, expected(%.2f dbFS @ %.3fMHz), got(%.2f dbFS @%.3fMHz)",
            input.expectedPeakval_dBFS,
            input.expectedPeakFrequency / 1e6,
            peakAmplitude,
            peakFrequency / 1e6);
        reporter->OnStepUpdate(test, ctemp);
        reporter->OnSuccess(test);
        return OpStatus::Success;
    }
    else
    {
        char ctemp[512];
        sprintf(ctemp,
            "RF FAILED, expected(%.2f dbFS @ %.3fMHz), got(%.2f dbFS @%.3fMHz)",
            input.expectedPeakval_dBFS,
            input.expectedPeakFrequency / 1e6,
            peakAmplitude,
            peakFrequency / 1e6);
        reporter->OnFail(test, ctemp);
        return OpStatus::Error;
    }
}

} // namespace lime