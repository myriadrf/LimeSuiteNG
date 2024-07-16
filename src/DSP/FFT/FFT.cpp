#include "FFT.h"

#include <cassert>
#include <cstring>
#include <chrono>

namespace lime {

FFT::FFT(uint8_t channelCount, uint32_t size, WindowFunctionType windowType)
    : samplesFIFO(channelCount)
    , channelCount(channelCount)
    , currentWindowType(windowType)
    , m_fftCalcPlan(kiss_fft_alloc(size, 0, nullptr, nullptr))
{
    m_fftCalcIn.resize(size);
    m_fftCalcOut.resize(size);

    std::for_each(samplesFIFO.begin(), samplesFIFO.end(), [&](auto& vec) { vec.Resize(size * 2); });

    doWork.store(true, std::memory_order_relaxed);
    mWorkerThread = std::thread(&FFT::ProcessLoop, this);

    GenerateWindowCoefficients(windowType, m_fftCalcIn.size(), mWindowCoeffs);
}

FFT::~FFT()
{
    doWork.store(false, std::memory_order_relaxed);
    inputAvailable.notify_all();
    if (mWorkerThread.joinable())
        mWorkerThread.join();
    kiss_fft_free(m_fftCalcPlan);
}

std::size_t FFT::PushSamples(const complex32f_t* const* const samples, std::size_t count, std::size_t samplesToSkip)
{
    std::size_t produced{ 0 };

    {
        std::unique_lock lk{ inputMutex };
        inputAvailable.wait(lk, [&]() { return samplesFIFO[0].Capacity() != samplesFIFO[0].Size() || !doWork.load(); });

        produced = samplesFIFO[0].Produce(samples[0] + samplesToSkip, count);
    }
    inputAvailable.notify_all();

    for (uint8_t ch = 1; ch < channelCount; ++ch)
    {
        std::size_t pushed{ 0 };
        // Ensures the same amount of samples gets pushed all across the channel range.
        while (pushed < produced)
        {
            {
                std::unique_lock lk{ inputMutex };
                inputAvailable.wait(lk, [&]() { return samplesFIFO[ch].Capacity() != samplesFIFO[ch].Size() || !doWork.load(); });

                pushed += samplesFIFO[ch].Produce(samples[ch] + samplesToSkip + pushed, produced - pushed);
            }
            inputAvailable.notify_all();
        }
    }

    return produced;
}

void FFT::SetResultsCallback(FFT::CallbackType fptr, void* userData)
{
    resultsCallback = fptr;
    mUserData = userData;
}

void FFT::SetWindowFunction(WindowFunctionType windowFunction)
{
    if (currentWindowType != windowFunction)
    {
        currentWindowType = windowFunction;
        GenerateWindowCoefficients(windowFunction, m_fftCalcIn.size(), mWindowCoeffs);
    }
}

void FFT::SetAverageCount(std::size_t count)
{
    avgCount = count;
}

std::vector<float> FFT::Calc(const std::vector<complex32f_t>& samples, WindowFunctionType window)
{
    const int fftSize = samples.size();
    std::vector<float> coeffs;
    GenerateWindowCoefficients(window, fftSize, coeffs);

    std::vector<kiss_fft_cpx> fftIn(fftSize);
    std::vector<kiss_fft_cpx> fftOut(fftSize);
    std::vector<float> bins(fftSize);
    kiss_fft_cfg plan = kiss_fft_alloc(fftSize, 0, nullptr, nullptr);

    for (int i = 0; i < fftSize; ++i)
    {
        fftIn[i].r = samples[i].real() * coeffs[i];
        fftIn[i].i = samples[i].imag() * coeffs[i];
    }
    kiss_fft(plan, fftIn.data(), fftOut.data());
    for (int i = 0; i < fftSize; ++i)
    {
        bins[i] = (fftOut[i].r * fftOut[i].r + fftOut[i].i * fftOut[i].i) / (fftSize * fftSize);
    }
    kiss_fft_free(plan);
    return bins;
}

void FFT::ConvertToDBFS(std::vector<float>& bins)
{
    for (float& amplitude : bins)
        amplitude = amplitude > 0 ? 10 * log10(amplitude) : -150;
}

template<typename T> void FFT::Calculate(const std::vector<std::vector<T>>& src, std::vector<std::vector<float>>& outputBins)
{
    constexpr double div = []() {
        if constexpr (std::is_same_v<T, complex16_t>)
        {
            return 32768.0;
        }
        return 1.0;
    }();

    outputBins.resize(src.size());

    for (std::size_t ch = 0; ch < src.size(); ++ch)
    {
        assert(src.at(ch).size() == m_fftCalcIn.size());
        for (std::size_t i = 0; i < src.at(ch).size(); ++i)
        {
            m_fftCalcIn.at(i).r = src.at(ch).at(i).real() / div * mWindowCoeffs.at(i);
            m_fftCalcIn.at(i).i = src.at(ch).at(i).imag() / div * mWindowCoeffs.at(i);
        }
        kiss_fft(m_fftCalcPlan, m_fftCalcIn.data(), m_fftCalcOut.data());
        outputBins.at(ch).resize(src.at(ch).size());

        std::size_t output_index = 0;
        for (std::size_t i = m_fftCalcIn.size() / 2 + 1; i < m_fftCalcIn.size(); ++i)
            outputBins.at(ch).at(output_index++) += m_fftCalcOut[i].r * m_fftCalcOut[i].r + m_fftCalcOut[i].i * m_fftCalcOut[i].i;

        for (std::size_t i = 0; i < m_fftCalcIn.size() / 2 + 1; ++i)
            outputBins.at(ch).at(output_index++) += m_fftCalcOut[i].r * m_fftCalcOut[i].r + m_fftCalcOut[i].i * m_fftCalcOut[i].i;
    }
}

void FFT::ProcessLoop()
{
    std::vector<std::vector<complex32f_t>> samples(channelCount);
    std::vector<std::vector<float>> fftBins;
    std::vector<std::vector<float>> avgOutput;
    std::vector<std::size_t> samplesReady(channelCount, 0);

    std::size_t resultsDone = 0;
    while (doWork.load(std::memory_order_relaxed) == true)
    {
        for (uint8_t ch = 0; ch < channelCount; ++ch)
        {
            samples.at(ch).resize(m_fftCalcIn.size());
            {
                std::unique_lock lk{ inputMutex };
                inputAvailable.wait(lk, [&]() { return samplesFIFO[ch].Size() != 0 || !doWork.load(); });

                int samplesNeeded = samples.at(ch).size() - samplesReady.at(ch);
                int ret = samplesFIFO[ch].Consume(samples.at(ch).data() + samplesReady.at(ch), samplesNeeded);
                samplesReady.at(ch) += ret;
            }
            inputAvailable.notify_all();
        }

        if (!std::all_of(samplesReady.begin(), samplesReady.end(), [&](const std::size_t& amount) {
                return amount == samples.at(0).size();
            }))
        {
            continue;
        }

        // auto t1 = std::chrono::high_resolution_clock::now();
        Calculate(samples, fftBins);
        avgOutput.resize(fftBins.size());
        ++resultsDone;

        for (std::size_t ch = 0; ch < fftBins.size(); ++ch)
        {
            samplesReady.at(ch) = 0;
            avgOutput.at(ch).resize(fftBins.at(ch).size());
            for (std::size_t i = 0; i < fftBins.at(ch).size(); ++i)
            {
                avgOutput.at(ch).at(i) += fftBins.at(ch).at(i) * fftBins.at(ch).at(i);
            }
        }

        if (resultsDone < avgCount)
        {
            continue;
        }

        if (resultsCallback)
        {
            for (std::size_t ch = 0; ch < fftBins.size(); ++ch)
            {
                const float div = static_cast<float>(resultsDone) * fftBins.at(ch).size() * fftBins.at(ch).size();
                for (std::size_t s = 0; s < fftBins.at(ch).size(); ++s)
                {
                    fftBins.at(ch).at(s) /= div;
                }
            }

            resultsCallback(fftBins, mUserData);
        }
        resultsDone = 0;

        for (auto& vec : avgOutput)
        {
            std::memset(vec.data(), 0, vec.size() * sizeof(float));
        }
        // auto t2 = std::chrono::high_resolution_clock::now();
        // auto timePeriod = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
        //printf("FFT update %lius\n", timePeriod);
    }
}

void FFT::GenerateWindowCoefficients(WindowFunctionType func, std::size_t coefCount, std::vector<float>& windowFcoefs)
{
    float amplitudeCorrection = 1;
    windowFcoefs.resize(coefCount);
    float a0 = 0.35875;
    float a1 = 0.48829;
    float a2 = 0.14128;
    float a3 = 0.01168;
    float PI = 3.14159265359;
    switch (func)
    {
    case WindowFunctionType::BLACKMAN_HARRIS:
        for (std::size_t i = 0; i < coefCount; ++i)
            windowFcoefs[i] = a0 - a1 * cos((2 * PI * i) / (coefCount - 1)) + a2 * cos((4 * PI * i) / (coefCount - 1)) -
                              a3 * cos((6 * PI * i) / (coefCount - 1));
        break;
    case WindowFunctionType::HAMMING:
        amplitudeCorrection = 0;
        a0 = 0.54;
        for (std::size_t i = 0; i < coefCount; ++i)
            windowFcoefs[i] = a0 - (1 - a0) * cos((2 * PI * i) / (coefCount));
        break;
    case WindowFunctionType::HANNING:
        amplitudeCorrection = 0;
        for (std::size_t i = 0; i < coefCount; ++i)
            windowFcoefs[i] = 0.5 * (1 - cos((2 * PI * i) / (coefCount)));
        break;
    case WindowFunctionType::NONE:
    default:
        for (std::size_t i = 0; i < coefCount; ++i)
            windowFcoefs[i] = 1;
        return;
    }
    for (std::size_t i = 0; i < coefCount; ++i)
        amplitudeCorrection += windowFcoefs[i];
    amplitudeCorrection = 1.0 / (amplitudeCorrection / coefCount);
    for (std::size_t i = 0; i < coefCount; ++i)
        windowFcoefs[i] *= amplitudeCorrection;
}

} // namespace lime
