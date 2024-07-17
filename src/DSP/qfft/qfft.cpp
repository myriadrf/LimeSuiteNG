/* --------------------------------------------------------------------------------------------
FILE:		qfft.cpp
DESCRIPTION:	Implementation of qfft module
CONTENT:
AUTHOR:		Lime Microsystems LTD
DATE:		Mar 25, 2005
REVISIONS:	May 12, 2005: If fname = "", results will not be writen into a file.
		Nov 20, 2015: Original fft.cpp modified to implement quadratute (copmplex) FFT.
   -------------------------------------------------------------------------------------------- */

#include "qfft.h"
#include "fourier.h"
#include <iostream>
using namespace std;

namespace lime {
// Constructors
// --------------------------------------------------------------------------------------------

// Only name is specified
template<class T> qfft<T>::qfft()
{
    // Default parameter values
    ssize = 20; // Internal data storage size iz 1<<20
    skip = 0; // How many cycles to ignore input data
    Amax = 1.0; // To normalise amplitude with
    Rb = 0; // Resolution bandwidth in fft points is 1
    fs = 1.0; // Sampling frequency
    Nfloor = -140.0; // Noise floor
    Win = true; // Apply Hann window
    fname = "module.fft"; // Output file name is module_name.fft
    a = s = p = f = 0; // Not allocated
    data = 0;
    Np = 0;
}

// Name + parameters
template<class T>
qfft<T>::qfft(int Ssize, int Skip, double AMax, int RB, double Fs, double NFloor, bool WIN, const std::string& Fname)
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
    a = s = p = f = 0; // Not allocated
    data = 0;
    Np = 0;
}

// Destructor
// --------------------------------------------------------------------------------------------
template<class T> qfft<T>::~qfft()
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
}

// --------------------------------------------------------------------------------------------
template<class T> void qfft<T>::init()
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
    a = s = p = f = 0;

    Np = 0; // Flag that FFT is not done yet

    // Data storage
    if (data)
        delete[] data;
    data = new complex<double>[(1 << ssize) + 1];
    samples = 0;

    // Initialise skiping process
    skiping = skip;
}

// --------------------------------------------------------------------------------------------
template<class T> void qfft<T>::always(T xi, T xq)
{
    if (skiping > 0)
    {
        skiping--;
    }
    else
    {
        if (samples < (1 << ssize))
            data[samples++] = complex<double>((double)xi, (double)xq);
    }
}

// --------------------------------------------------------------------------------------------
template<class T> void qfft<T>::finish()
{

    //	sc_string ftime = fname+".out";
    //	FILE *fp = fopen(ftime.c_str(), "w");
    //	for(int i=0; i<samples; i++) fprintf(fp, "%lg\n", (double) data[i]);
    //	fclose(fp);

    int N; // Number of fft points

    if (samples < 4)
    {
        cerr << "QFFT model: "
             << " : Not enough points for fft...\n";
        // You can calculate it manualy :-)
        return;
    }

    // Find the nearest power of two to samples
    for (N = (1 << ssize); N > samples; N /= 2)
        ;

    // Allocate memory for FFT results
    Np = N / (1 << Rb);
    a = new double[Np];
    s = new double[Np];
    p = new double[Np];
    f = new double[Np];

    // Ready for FFT
    Fourier::qspectrum(N, (1 << Rb), data, a, s, p, Amax, Win, Nfloor);

    // Construct frequency scale
    // It goes from -fclk/2+fbin up to fclk/2 to match qspectrum()
    for (int i = 0; i < Np; i++)
        f[i] = fs * (double)(i - Np / 2 + 1) / (double)(Np);

    if (fname.length() > 0)
    {
        FILE* fp = fopen(fname.c_str(), "w");
        if (fp == NULL)
        {
            cerr << "QFFT model: "
                 << " : Could not open output file " << fname << " ...\n";
            return;
        }
        // Write data to file
        fprintf(fp, "# File format: 'frequency' 'amplitude' 'log amplitude' 'phase'\n");
        fprintf(fp, "# ------------------------------------------------------------\n");
        for (int i = 0; i < Np; i++)
            fprintf(fp, "%lg %lg %lg %lg\n", f[i], a[i], s[i], p[i]);
        fclose(fp);
    }
}

// --------------------------------------------------------------------------------------------
template<class T> double qfft<T>::am(double fc)
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

template<class T> double qfft<T>::amlog(double fc)
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

template<class T> double qfft<T>::ph(double fc)
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

template<class T> void qfft<T>::getData(double freq, double* f1, double* ampl, double* phase)
{

    int i = (int)(Np / 2 - 1 + freq / (fs / 2.0) * Np / 2);

    if (i <= 0)
        i = 0;
    else if (i >= Np)
        i = Np - 1;
    //i--;
    *ampl = a[i];
    *phase = p[i];
    *f1 = f[i];
}

template<class T> void qfft<T>::acpr(double fc, double cbw, double cs, double* acp)
{
    double p[5]; // Channel power
    double l[5], h[5]; // Chanel border

    for (int i = 0; i < 5; i++)
        p[i] = 0.0;

    l[0] = fc - 2.0 * cs - cbw / 2.0;
    h[0] = fc - 2.0 * cs + cbw / 2.0;
    for (int i = 1; i < 5; i++)
    {
        l[i] = l[i - 1] + cs;
        h[i] = h[i - 1] + cs;
    }

    for (int i = 0; i < Np; i++)
        for (int j = 0; j < 5; j++)
            if ((f[i] >= l[j]) && (f[i] <= h[j]))
                p[j] += a[i] * a[i];

    if (p[2] == 0.0)
    {
        std::cout << "ACPR: Main channel power is zero...\n";
        for (int i = 0; i < 4; i++)
            acp[i] = 0.0;
        return;
    }

    acp[0] = p[0] / p[2];
    acp[1] = p[1] / p[2];
    acp[2] = p[3] / p[2];
    acp[3] = p[4] / p[2];

    for (int i = 0; i < 4; i++)
    {
        if (acp[i] != 0.0)
            acp[i] = 10.0 * log10(acp[i]);
        if (acp[i] < Nfloor)
            acp[i] = Nfloor;
    }
}

template class qfft<int>;
template class qfft<double>;

} // namespace lime