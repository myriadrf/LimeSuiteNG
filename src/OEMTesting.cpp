#include "OEMTesting.h"

#include "limesuiteng/OpStatus.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/StreamConfig.h"
#include "DSP/FFT/FFT.h"

namespace lime {

OpStatus CaptureRxSamples(
    SDRDevice& device, uint8_t chipIndex, uint8_t channelIndex, std::vector<complex32f_t>& samples, const size_t count)
{
    samples.resize(count);

    StreamConfig stream;
    stream.channels.at(TRXDir::Rx).push_back(channelIndex);
    stream.format = DataFormat::F32;
    stream.linkFormat = DataFormat::I12;

    OpStatus status = device.StreamSetup(stream, chipIndex);
    if (status != OpStatus::Success)
        return status;

    device.StreamStart(chipIndex);

    //Receive samples

    lime::complex32f_t* dest[2] = { samples.data(), nullptr };

    // ignore first batch of samples, just in case there would be stabilization period of digital DC corrector
    auto samplesGot = device.StreamRx(chipIndex, reinterpret_cast<lime::complex32f_t**>(&dest), count, nullptr);
    if (samplesGot != count)
        return OpStatus::Error;

    samplesGot = device.StreamRx(chipIndex, reinterpret_cast<lime::complex32f_t**>(&dest), count, nullptr);
    if (samplesGot != count)
        return OpStatus::Error;

    device.StreamStop(chipIndex);
    device.StreamDestroy(chipIndex);

    return OpStatus::Success;
}

void CalculateSignalPeak(const std::vector<complex32f_t>& samples, double& peakAmplitude_dBFS, double& samplerateWeight)
{
    std::vector<float> bins = FFT::Calc(samples);
    FFT::ConvertToDBFS(bins);

    peakAmplitude_dBFS = -1000;
    size_t peakBin = 0;

    for (size_t i = 1; i < bins.size(); ++i) // skip DC bin
    {
        if (bins[i] > peakAmplitude_dBFS)
        {
            peakAmplitude_dBFS = bins[i];
            peakBin = i;
        }
    }

    samplerateWeight = double(peakBin) / bins.size();
    if (peakBin > bins.size() / 2)
        samplerateWeight -= 1.0;
}

} // namespace lime