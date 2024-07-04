#include "FFT.h"

#include <cassert>
#include <cstring>
#include <chrono>

namespace lime {

FFT::FFT(uint32_t size, WindowFunctionType windowType)
    : samplesFIFO(size * 128)
    , currentWindowType(windowType)
    , m_fftCalcPlan(kiss_fft_alloc(size, 0, nullptr, nullptr))
{
    m_fftCalcIn.resize(size);
    m_fftCalcOut.resize(size);

    doWork.store(true, std::memory_order_relaxed);
    mWorkerThread = std::thread(&FFT::ProcessLoop, this);

    GenerateWindowCoefficients(WindowFunctionType::BLACKMAN_HARRIS, m_fftCalcIn.size(), mWindowCoeffs);
}

FFT::~FFT()
{
    doWork.store(false, std::memory_order_relaxed);
    inputAvailable.notify_all();
    if (mWorkerThread.joinable())
        mWorkerThread.join();
    kiss_fft_free(m_fftCalcPlan);
}

int FFT::PushSamples(const complex32f_t* samples, uint32_t count)
{
    int produced = samplesFIFO.Produce(samples, count);
    inputAvailable.notify_all();
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

void FFT::SetAverageCount(int count)
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

void FFT::Calculate(const complex16_t* src, uint32_t count, std::vector<float>& outputBins)
{
    assert(count == m_fftCalcIn.size());
    for (uint32_t i = 0; i < count; ++i)
    {
        m_fftCalcIn[i].r = src[i].real() / 32768.0 * mWindowCoeffs[i];
        m_fftCalcIn[i].i = src[i].imag() / 32768.0 * mWindowCoeffs[i];
    }
    kiss_fft(m_fftCalcPlan, m_fftCalcIn.data(), m_fftCalcOut.data());
    outputBins.resize(count);
    for (uint32_t i = 0; i < count; ++i)
    {
        outputBins[i] = m_fftCalcOut[i].r * m_fftCalcOut[i].r + m_fftCalcOut[i].i * m_fftCalcOut[i].i;
    }
}

void FFT::Calculate(const complex32f_t* src, uint32_t count, std::vector<float>& outputBins)
{
    assert(count == m_fftCalcIn.size());
    for (uint32_t i = 0; i < count; ++i)
    {
        m_fftCalcIn[i].r = src[i].real() * mWindowCoeffs[i];
        m_fftCalcIn[i].i = src[i].imag() * mWindowCoeffs[i];
    }
    kiss_fft(m_fftCalcPlan, m_fftCalcIn.data(), m_fftCalcOut.data());
    outputBins.resize(count);

    std::size_t output_index = 0;
    for (std::size_t i = m_fftCalcIn.size() / 2 + 1; i < m_fftCalcIn.size(); ++i)
        outputBins[output_index++] += m_fftCalcOut[i].r * m_fftCalcOut[i].r + m_fftCalcOut[i].i * m_fftCalcOut[i].i;
    for (std::size_t i = 0; i < m_fftCalcIn.size() / 2 + 1; ++i)
        outputBins[output_index++] += m_fftCalcOut[i].r * m_fftCalcOut[i].r + m_fftCalcOut[i].i * m_fftCalcOut[i].i;
}

void FFT::ProcessLoop()
{
    std::vector<complex32f_t> samples(m_fftCalcIn.size());
    std::vector<float> fftBins(m_fftCalcOut.size());
    std::vector<float> avgOutput(m_fftCalcOut.size());
    uint32_t samplesReady = 0;

    std::unique_lock<std::mutex> lk(inputMutex);

    int resultsDone = 0;
    while (doWork.load(std::memory_order_relaxed) == true)
    {
        inputAvailable.wait(lk, [&]() { return samplesFIFO.Size() != 0 || !doWork.load(); });

        int samplesNeeded = samples.size() - samplesReady;
        int ret = samplesFIFO.Consume(samples.data() + samplesReady, samplesNeeded);
        samplesReady += ret;

        if (samplesReady != samples.size())
        {
            continue;
        }

        // auto t1 = std::chrono::high_resolution_clock::now();
        Calculate(samples.data(), samples.size(), fftBins);
        ++resultsDone;
        samplesReady = 0;

        for (uint32_t i = 0; i < fftBins.size(); ++i)
            avgOutput[i] += fftBins[i] * fftBins[i];

        if (resultsDone < avgCount)
        {
            continue;
        }

        if (resultsCallback)
        {
            for (std::size_t s = 0; s < fftBins.size(); ++s)
            {
                const float div = static_cast<float>(resultsDone) * fftBins.size() * fftBins.size();
                fftBins.at(s) /= div;
            }

            resultsCallback(fftBins, mUserData);
        }
        resultsDone = 0;
        std::memset(avgOutput.data(), 0, avgOutput.size() * sizeof(float));
        // auto t2 = std::chrono::high_resolution_clock::now();
        // auto timePeriod = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
        //printf("FFT update %lius\n", timePeriod);
    }
}

void FFT::GenerateWindowCoefficients(WindowFunctionType func, uint32_t coefCount, std::vector<float>& windowFcoefs)
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
        for (uint32_t i = 0; i < coefCount; ++i)
            windowFcoefs[i] = a0 - a1 * cos((2 * PI * i) / (coefCount - 1)) + a2 * cos((4 * PI * i) / (coefCount - 1)) -
                              a3 * cos((6 * PI * i) / (coefCount - 1));
        break;
    case WindowFunctionType::HAMMING:
        amplitudeCorrection = 0;
        a0 = 0.54;
        for (uint32_t i = 0; i < coefCount; ++i)
            windowFcoefs[i] = a0 - (1 - a0) * cos((2 * PI * i) / (coefCount));
        break;
    case WindowFunctionType::HANNING:
        amplitudeCorrection = 0;
        for (uint32_t i = 0; i < coefCount; ++i)
            windowFcoefs[i] = 0.5 * (1 - cos((2 * PI * i) / (coefCount)));
        break;
    case WindowFunctionType::NONE:
    default:
        for (uint32_t i = 0; i < coefCount; ++i)
            windowFcoefs[i] = 1;
        return;
    }
    for (uint32_t i = 0; i < coefCount; ++i)
        amplitudeCorrection += windowFcoefs[i];
    amplitudeCorrection = 1.0 / (amplitudeCorrection / coefCount);
    for (uint32_t i = 0; i < coefCount; ++i)
        windowFcoefs[i] *= amplitudeCorrection;
}

} // namespace lime
