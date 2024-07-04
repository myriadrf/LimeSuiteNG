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
    typedef std::function<void(const std::vector<float>& bins, void* userData)> CallbackType;

        /// @brief Enumeration for selecting the window coefficient function to use
    enum class WindowFunctionType : uint8_t { NONE = 0, BLACKMAN_HARRIS, HAMMING, HANNING };

    /// @brief Constructs the FFT object.
    /// @param size The amount of bins to use.
    /// @param windowType The Window function to initially initialize the FFT transform with.
    FFT(uint32_t size, WindowFunctionType windowType = WindowFunctionType::BLACKMAN_HARRIS);
    ~FFT();

    /// @brief Adds the given samples to the FFT calculation.
    /// @param samples The samples to add to the calculation.
    /// @param count The amount of samples to add to the calculation.
    /// @return The amount of samples actually added to the buffer.
    int PushSamples(const complex32f_t* samples, uint32_t count);

    /// @brief Sets the function to call when a calculations update happens.
    /// @param fptr The pointer to the function to call.
    /// @param userData The data to pass to the function.
    void SetResultsCallback(FFT::CallbackType fptr, void* userData);

    void SetWindowFunction(WindowFunctionType windowType);
    void SetAverageCount(int count);

    /// @brief Generates the coefficients for a given window function
    /// @param type The type of the window function to generate the coefficients for.
    /// @param coefCount The amount of coefficients to generate.
    /// @param coeffs The buffer to which to store the coefficients.
    static void GenerateWindowCoefficients(WindowFunctionType type, uint32_t coefCount, std::vector<float>& coeffs);

    /// @brief Calculates the FFT bins from the provided samples.
    /// @param samples The samples to calculate from.
    /// @param window The window function to apply to the samples before calculating the bins.
    /// @return The array of computed bins, the same width as the amount of samples.
    static std::vector<float> Calc(const std::vector<complex32f_t>& samples, WindowFunctionType window = WindowFunctionType::NONE);

    /// @brief Convert the amplitude in the bins to a Decibels relative to full scale.
    /// @param bins The bins to convert.
    static void ConvertToDBFS(std::vector<float>& bins);

  private:
    void Calculate(const complex16_t* src, uint32_t count, std::vector<float>& outputBins);
    void Calculate(const complex32f_t* src, uint32_t count, std::vector<float>& outputBins);
    void ProcessLoop();

    RingBuffer<complex32f_t> samplesFIFO;

    std::thread mWorkerThread;

    WindowFunctionType currentWindowType;

    kiss_fft_cfg m_fftCalcPlan;
    std::vector<float> mWindowCoeffs;
    std::vector<kiss_fft_cpx> m_fftCalcIn;
    std::vector<kiss_fft_cpx> m_fftCalcOut;

    std::atomic<bool> doWork{};
    std::condition_variable inputAvailable;
    std::mutex inputMutex;

    CallbackType resultsCallback{};
    void* mUserData{};

    int avgCount = 100;
};

} // namespace lime
