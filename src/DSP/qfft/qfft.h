// --------------------------------------------------------------------------------------------
// Complex i.e. quadrature version of FFT Monitor
// --------------------------------------------------------------------------------------------

#include <string>
#include <complex>

namespace lime {

template<class T> class qfft
{
    T xi, xq; // Input signal = xi + j*xq
  public:
    // Constructors
    qfft();
    qfft(int, int, double, int, double, double, bool, const std::string&);
    ~qfft();

    // Parameters
    int ssize; // Internal data storage size is 1<<ssize
    int skip; // How many cycles to ignore input data
    double Amax; // To normalise amplitude with,
        // set it to <= 0 to get the spectrum in dBc
    int Rb; // Resolution bandwidth in fft points (1<<Rb)
    double fs; // Sampling frequency
    double Nfloor; // Noise floor
    bool Win; // Apply Hann window if true
    std::string fname; // Output file name

    void init(); // Standard SC_THREAD functions
    void always(T xi, T xq);
    void finish();
    void getData(double f, double* f1, double* ampl, double* phase);

  private:
    int samples; // Number of collected samples
    std::complex<double>* data; // Time domain data
    int skiping; // Used in skiping the initial data

    // FFT results
  public:
    int Np; // Number of points in the arrays below
    double* a; // Amplitude
    double* s; // Amplitude in log scale
    double* p; // Phase
    double* f; // Frequency

    double am(double fc); // Amplitude at frequency fc
    double amlog(double fc); // Log amplitude at frequency fc
    double ph(double fc); // Phase at frequency fc
    void acpr(double fc, double cbw, double cs, double* acp);
};

} // namespace lime