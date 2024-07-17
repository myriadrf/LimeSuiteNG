/* --------------------------------------------------------------------------------------------
FILE:		fft.cpp
DESCRIPTION:	Implementation of fft module
CONTENT:
AUTHOR:		Lime Microsystems LTD
		LONGDENE HOUSE
		HEDGEHOG LANE
		HASLEMERE GU27 2PH
DATE:		Mar 25, 2005
REVISIONS:	May 12, 2005: If fname = "", results will not be writen into a file.
   -------------------------------------------------------------------------------------------- */
#include "fft.h"
#include "fourier.h"
#include <iostream>
#include <complex>

using namespace std;
#define VERBOSEPRINT

namespace lime {

// Constructors
// --------------------------------------------------------------------------------------------

// Only name is specifiied
template<class T> fft<T>::fft()
{
    // Default parameter values
    ssize = 20; // Internal data storage size iz 1<<20
    skip = 0; // How many cycles to ignore input data
    Amax = 1.0; // To normalise amplitude with
    Rb = 0; // Resolution bandwidth in fft points is 1
    fs = 1.0; // Sampling frequency
    Nfloor = -140.0; // Noise floor
    Win = false; // Apply Hann window
    fname = "module.fft"; // Output file name is module_name.fft
    a = s = p = f = data = 0; // Not allocated
    X = 0;
    Np = 0;
}

// Name + parameters
template<class T>
fft<T>::fft(int Ssize, int Skip, double AMax, int RB, double Fs, double NFloor, bool WIN, const std::string& Fname)
{

    // Default parameter values
    ssize = Ssize;
    skip = Skip;
    Amax = AMax;
    Rb = RB;
    fs = Fs;
    Nfloor = NFloor;
    Win = WIN;
    fname = Fname;
    a = s = p = f = data = 0; // Not allocated, new
    X = 0;
    Np = 0;
}

// Destructor
// --------------------------------------------------------------------------------------------
template<class T> fft<T>::~fft()
{
    if (data)
        delete[] data;
    if (a)
        delete[] a;
    if (s)
        delete[] s;
    if (p)
        delete[] p;
    if (f)
        delete[] f;

    if (X)
        delete[] X; // new
}

// --------------------------------------------------------------------------------------------
template<class T> void fft<T>::init()
{
    // Set array pointers to NULL
    // If memory is allocated, release it
    if (a)
        delete[] a;
    if (s)
        delete[] s;
    if (p)
        delete[] p;
    if (f)
        delete[] f;

    if (X)
        delete[] X; // new

    a = s = p = f = 0;
    X = 0;

    Np = 0; // Flag that FFT is not done yet

    // Data storage
    if (data)
        delete[] data;
    data = new double[(1 << ssize) + 1];
    samples = 0;

    // Initialise skiping process
    skiping = skip;
}

// --------------------------------------------------------------------------------------------
template<class T> void fft<T>::always(T x)
{
    if (skiping > 0)
    {
        skiping--;
    }
    else
    {
        if (samples < (1 << ssize))
            data[samples++] = (double)x;
    }
}

// --------------------------------------------------------------------------------------------
template<class T> void fft<T>::finish()
{

    //	sc_string ftime = fname+".out";
    //	FILE *fp = fopen(ftime.c_str(), "w");
    //	for(int i=0; i<samples; i++) fprintf(fp, "%lg\n", (double) data[i]);
    //	fclose(fp);

    int N; // Number of fft points

    if (samples < 4)
    {
        std::cout << "FFT model: "
                  << " : Not enough points for fft...\n";
        // You can calculate it manualy :-)
        return;
    }

    // Find the nearest power of two to samples
    for (N = (1 << ssize); N > samples; N /= 2)
        ;

    // Allocate memory for FFT results
    Np = N / (1 << Rb) / 2;
    a = new double[Np];
    s = new double[Np];
    p = new double[Np];
    f = new double[Np];

    X = new complex<double>[Np + 1];

    //printf("Exit\n");

    // Ready for FFT
    Fourier::spectrum(N, (1 << Rb), data, a, s, p, Amax, Win, Nfloor, X);

    // Construct frequency scale
    for (int i = 0; i < Np; i++)
        f[i] = 0.5 * fs * (double)(i) / (double)(Np);

#ifdef VERBOSEPRINT
    if (fname.length() > 0)
    {
        FILE* fp = fopen(fname.c_str(), "w");
        if (fp == NULL)
        {
            std::cout << "FFT model: "
                      << " : Could not open output file " << fname << " ...\n";
            return;
        }
        // Write data to file
        fprintf(fp, "# File format: 'frequency' 'amplitude' 'log amplitude' 'phase'\n");
        fprintf(fp, "# ------------------------------------------------------------\n");
        for (int i = 0; i < Np; i++)
            fprintf(fp, "%lg %lg %lg %lg %lg %lg\n", f[i], a[i], s[i], p[i], real(X[i]) / Np, imag(X[i]) / Np);
        fclose(fp);
    }
#endif
}

// --------------------------------------------------------------------------------------------
template<class T> double fft<T>::am(double fc)
{
    if (Np == 0)
        return (0.0); // FFT is not done yet

    int index = 0;
    double deltaf = fabs(f[0] - fc);

    for (int i = 1; i < Np; i++)
    {
        if (fabs(f[i] - fc) < deltaf)
        {
            deltaf = fabs(f[i] - fc);
            index = i;
        }
    }

    return (a[index]);
}

template<class T> double fft<T>::amlog(double fc)
{
    if (Np == 0)
        return (0.0); // FFT is not done yet

    int index = 0;
    double deltaf = fabs(f[0] - fc);

    for (int i = 1; i < Np; i++)
    {
        if (fabs(f[i] - fc) < deltaf)
        {
            deltaf = fabs(f[i] - fc);
            index = i;
        }
    }

    return (s[index]);
}

template<class T> double fft<T>::ph(double fc)
{
    if (Np == 0)
        return (0.0); // FFT is not done yet

    int index = 0;
    double deltaf = fabs(f[0] - fc);

    for (int i = 1; i < Np; i++)
    {
        if (fabs(f[i] - fc) < deltaf)
        {
            deltaf = fabs(f[i] - fc);
            index = i;
        }
    }

    return (p[index]);
}

template<class T> void fft<T>::getData(double freq, double* f1, double* ampl, double* phase, double* Re, double* Im)
{

    int i = (int)(freq / (fs / 2.0) * Np);
    //i--;

    if (fabs(f[i + 1] - freq) < fabs(f[i] - freq))
        i++;

    *ampl = a[i];
    *phase = p[i];
    *f1 = f[i];

    *Re = real(X[i]) / Np;
    *Im = imag(X[i]) / Np;
}

template class fft<int>;
template class fft<double>;

} // namespace lime
