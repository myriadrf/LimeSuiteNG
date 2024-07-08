#pragma once

#include "limesuiteng/complex.h"
#include "limesuiteng/config.h"
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include "RingBuffer.h"
#include "../external/kissFFT/kiss_fft.h"

namespace lime {

/// @brief Class for calculating Fast Fourier Transforms.
class LIME_API FFT
{
  public:
    /// @brief The type of the callback which gets called on a calculations update.
    typedef std::function<void(const std::vector<std::vector<float>>& bins, void* userData)> CallbackType;

    /// @brief Enumeration for selecting the window coefficient function to use
    enum class WindowFunctionType : uint8_t { NONE = 0, BLACKMAN_HARRIS, HAMMING, HANNING };

    /// @brief Constructs the FFT object.
    /// @param channelCount The amount of channels to build the FFT class for.
    /// @param size The amount of bins to use.
    /// @param windowType The Window function to initially initialize the FFT transform with.
    FFT(uint8_t channelCount, uint32_t size, WindowFunctionType windowType = WindowFunctionType::BLACKMAN_HARRIS);
    ~FFT();

    /// @brief Adds the given samples to the FFT calculation.
    /// @param samples The samples to add to the calculation.
    /// @param count The amount of samples to add to the calculation.
    /// @param samplesToSkip The amount of samples to skip from the beginning of the sample array.
    /// @return The amount of samples actually added to the buffer.
    std::size_t PushSamples(const complex32f_t* const* const samples, std::size_t count, std::size_t samplesToSkip);

    /// @brief Sets the function to call when a calculations update happens.
    /// @param fptr The pointer to the function to call.
    /// @param userData The data to pass to the function.
    void SetResultsCallback(FFT::CallbackType fptr, void* userData);

    /// @brief Sets the window function to use in the next calculations.
    /// @param windowType The window type to use for the FFT calculations.
    void SetWindowFunction(WindowFunctionType windowType);

    /// @brief Sets the amount of samples sampled and averaged before giving the bin values.
    /// @param count The amount of samples to average out with.
    void SetAverageCount(std::size_t count);

    /// @brief Generates the coefficients for a given window function
    /// @param type The type of the window function to generate the coefficients for.
    /// @param coefCount The amount of coefficients to generate.
    /// @param coeffs The buffer to which to store the coefficients.
    static void GenerateWindowCoefficients(WindowFunctionType type, std::size_t coefCount, std::vector<float>& coeffs);

    /// @brief Calculates the FFT bins from the provided samples.
    /// @param samples The samples to calculate from.
    /// @param window The window function to apply to the samples before calculating the bins.
    /// @return The array of computed bins, the same width as the amount of samples.
    static std::vector<float> Calc(const std::vector<complex32f_t>& samples, WindowFunctionType window = WindowFunctionType::NONE);

    /// @brief Convert the amplitude in the bins to a Decibels relative to full scale.
    /// @param bins The bins to convert.
    static void ConvertToDBFS(std::vector<float>& bins);

  private:
    template<typename T> void Calculate(const std::vector<std::vector<T>>& src, std::vector<std::vector<float>>& outputBins);
    void ProcessLoop();

    std::vector<RingBuffer<complex32f_t>> samplesFIFO;
    uint8_t channelCount;

    std::thread mWorkerThread;

    WindowFunctionType currentWindowType;
    std::vector<float> mWindowCoeffs;

    kiss_fft_cfg m_fftCalcPlan;
    std::vector<kiss_fft_cpx> m_fftCalcIn;
    std::vector<kiss_fft_cpx> m_fftCalcOut;

    std::atomic<bool> doWork{};
    std::condition_variable inputAvailable;
    std::mutex inputMutex;

    CallbackType resultsCallback{};
    void* mUserData{};

    std::size_t avgCount = 100;
};

} // namespace lime
