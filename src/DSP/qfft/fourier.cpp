/* --------------------------------------------------------------------------------------------
FILE:		fourier.cpp
DESCRIPTION:	Some useful FFT functions.
CONTENT:		
AUTHOR:		Lime Microsystems LTD
DATE:		Mar 25, 2005
REVISIONS:	Nov 20, 2015: qspectrum() added.	
   -------------------------------------------------------------------------------------------- */

#include <math.h>
#include "fourier.h"
using namespace std;

#ifdef _WINDOWS
    #include "winmath.h"
#endif

#define WINDOW Hann

// --------------------------------------------------------------------------------------------
// Calculate the spectrum
//
//	N		- number of points in time domain
//	Rb		- resolution bandwidth in frequency points
//	x[N]		- real data in time domain
//	a[N/2/Rb]	- amplitude
//	s[N/2/Rb]	- amplitude in dB
//	p[N/2/Rb]	- phase in PI radians
//	Amax		- amplitude to normalise spectrum with
//	Win		- apply Hann window if true
//	Nfloor		- noise floor
// --------------------------------------------------------------------------------------------
void Fourier::spectrum(
    int N, int Rb, double* x, double* a, double* s, double* p, double Amax, bool Win, double Nfloor, complex<double>* X)
{
    // Pack real data into complex array of half size
    //complex<double> *X = new complex<double>[(N / 2) + 1];

    if (Win)
    {
        for (int n = 0; n < N / 2; n++)
            X[n] = complex<double>(x[2 * n] * WINDOW(2 * n, N), x[2 * n + 1] * WINDOW(2 * n + 1, N));
    }
    else
    {
        for (int n = 0; n < N / 2; n++)
            X[n] = complex<double>(x[2 * n], x[2 * n + 1]);
    }

    // Do FFT of real function
    fftr(X, N);

    // Calculate amplitude and phase
    for (int n = 0; n < N / 2 / Rb; n++)
    {
        a[n] = 0.0;
        p[n] = 0.0;
        double xx;
        for (int i = 0; i < Rb; i++)
        {
            xx = real(X[n * Rb + i]) * real(X[n * Rb + i]) + imag(X[n * Rb + i]) * imag(X[n * Rb + i]);
            if ((n * Rb + i) != 0)
                xx *= 4.0; // We double the amplitude for nonzero frequencies
            // since there are negative frequencies as well
            if (xx > a[n])
                a[n] = xx;
            p[n] += atan2(imag(X[n * Rb + i]), real(X[n * Rb + i]) + 1.0e-12);
        }
        a[n] = sqrt(a[n]) / (double)(N);
        //		a[n] = a[n]/(double)(N);
        p[n] *= 1.0 / M_PI / (double)(Rb);
    }

    // Scale the amplitude. If Amax<=0, find max a[]
    double Max = Amax;
    if (Max <= 0.0)
        for (int n = 0; n < N / 2 / Rb; n++)
            if (a[n] > Max)
                Max = a[n];
    if (Max > 0.0)
        for (int n = 0; n < N / 2 / Rb; n++)
            a[n] /= Max;

    // Calculate log scale amplitude
    for (int n = 0; n < N / 2 / Rb; n++)
    {
        s[n] = (a[n] > 1.0e-16) ? 20.0 * log10(a[n]) : -320;
        //		s[n] = (a[n] > 1.0e-32) ? 10.0*log10(a[n]) : -320;
        if (s[n] < Nfloor)
            s[n] = Nfloor;
    }

    //delete[] X;
}

// --------------------------------------------------------------------------------------------
// Calculate quadrature (complex) spectrum
//
//	N		- number of points in time domain
//	Rb		- resolution bandwidth in frequency points
//	x[N]		- real data in time domain
//	a[N/Rb]		- amplitude
//	s[N/Rb]	- 	- amplitude in dB
//	p[N/Rb]	- 	- phase in PI radians
//	Amax		- amplitude to normalise spectrum with
//	Win		- apply Hann window if true
//	Nfloor		- noise floor
// --------------------------------------------------------------------------------------------
void Fourier::qspectrum(int N, int Rb, complex<double>* x, double* a, double* s, double* p, double Amax, bool Win, double Nfloor)
{
    complex<double>* X = new complex<double>[N];

    // Apply window
    if (Win)
    {
        for (int n = 0; n < N; n++)
            X[n] = x[n] * WINDOW(n, N);
    }
    else
    {
        for (int n = 0; n < N; n++)
            X[n] = x[n];
    }

    // Do FFT of complex function
    fftc(X, N, -1);

    // Reorder FFT result to start with negative frequencies and to end with positive
    // fftc() returns:
    //	X[0]=DC, X[1]=fbin, ..., X[N/2-1]=fclk/2-fbin, X[N/2]=fclk/2, X[N/2+1]= -fclk/2+fbin, ..., X[N-1]=-fbin
    // while we want it in the following order:
    //	x[0]=-fclk/2+fbin, x[1]=-fclk/2+2fbin, ..., x[N/2-2]=-fbin, x[N/2-1]=DC, x[N/2]= fbin, ..., x[N-1]=fclk/2
    // Hence:
    //	x[N/2-1+i]=X[i], i=0, N/2
    //	x[i] = X[N/2+1+i], i=0, N/2-2
    for (int i = 0; i <= N / 2; i++)
        x[N / 2 - 1 + i] = X[i];
    for (int i = 0; i <= N / 2 - 2; i++)
        x[i] = X[N / 2 + 1 + i];

    // Calculate amplitude and phase
    for (int n = 0; n < N / Rb; n++)
    {
        a[n] = 0.0;
        p[n] = 0.0;
        double xx;
        for (int i = 0; i < Rb; i++)
        {
            xx = real(x[n * Rb + i]) * real(x[n * Rb + i]) + imag(x[n * Rb + i]) * imag(x[n * Rb + i]);
            if (xx > a[n])
                a[n] = xx;
            p[n] += atan2(imag(x[n * Rb + i]), real(x[n * Rb + i]) + 1.0e-12);
        }
        a[n] = sqrt(a[n]) / (double)(N);
        p[n] *= 1.0 / M_PI / (double)(Rb);
    }

    // Scale the amplitude. If Amax<=0, find max a[]
    double Max = Amax;
    if (Max <= 0.0)
        for (int n = 0; n < N / Rb; n++)
            if (a[n] > Max)
                Max = a[n];
    if (Max > 0.0)
        for (int n = 0; n < N / Rb; n++)
            a[n] /= Max;

    // Calculate log scale amplitude
    for (int n = 0; n < N / Rb; n++)
    {
        s[n] = (a[n] > 1.0e-16) ? 20.0 * log10(a[n]) : -320;
        if (s[n] < Nfloor)
            s[n] = Nfloor;
    }

    delete[] X;
}

// --------------------------------------------------------------------------------------------
// FFT of a real function
// --------------------------------------------------------------------------------------------
void Fourier::fftr(complex<double>* x, int N)
{
    int n;
    double ang;
    complex<double> W, Xep, Xop;

    fftc(x, N / 2, -1);

    x[N / 2] = complex<double>(real(x[0]) - imag(x[0]), 0);
    x[0] = complex<double>(real(x[0]) + imag(x[0]), 0);

    for (n = 1; n <= N / 4; n++)
    {
        ang = (2.0 * M_PI * n) / N;
        W = complex<double>(sin(ang), cos(ang));
        Xep = (x[n] + conj(x[N / 2 - n])) / 2.0;
        Xop = W * ((x[n] - conj(x[N / 2 - n])) / 2.0);
        x[n] = Xep - Xop;
        x[N / 2 - n] = conj(Xep + Xop);
    }
}

// --------------------------------------------------------------------------------------------
// FFT of complex function
// --------------------------------------------------------------------------------------------
void Fourier::fftc(complex<double>* x, int N, int sign)
{
    int n, M, nbitr = 0;
    int i, j, nmax, step;
    double ang;
    complex<double> w, kernel, temp;

    /* Arrange x in bit reverse order */
    for (n = 1; n < N - 1; n++)
    {

        /* Compute nbitr, the bit reversed value of n. */
        for (M = N / 2; M >= N - nbitr;)
            M /= 2;
        nbitr = (nbitr % M) + M;

        /* Swap, but only once for each bit reverse pair */
        if (n < nbitr)
        {
            temp = x[n];
            x[n] = x[nbitr];
            x[nbitr] = temp;
        }
    }

    /* Compute butterflies */
    for (nmax = 1; nmax < N; nmax *= 2)
    {
        kernel = complex<double>(1, 0);
        ang = sign * M_PI / (double)nmax;
        w = complex<double>(cos(ang), sin(ang));
        step = 2 * nmax;
        for (n = 0; n < nmax; n++)
        {
            for (i = n; i < N; i += step)
            {
                j = i + nmax;
                temp = x[j] * kernel;
                x[j] = x[i] - temp;
                x[i] += temp;
            }
            kernel *= w;
        }
    }

    /* Scale by N if inverse FFT */
    if (sign == 1)
        for (n = 0; n < N; n++)
            x[n] /= (double)N;
}

// --------------------------------------------------------------------------------------------
// Window functions
// --------------------------------------------------------------------------------------------
double Fourier::Nuttall(int k, int N)
{
    const double a0 = 0.338946, a1 = 0.481973, a2 = 0.161054, a3 = 0.018027;
    double scale = 1.031 * 4.0 / sqrt(2 * a0 * a0 + a1 * a1 + a2 * a2 + a3 * a3) / (double)(N);
    double k_n = 2.0 * M_PI * (double)(k) / (double)(N);
    return (scale * (a0 - a1 * cos(k_n) + a2 * cos(2.0 * k_n) - a3 * cos(3.0 * k_n)));
}

double Fourier::Hann(int k, int N)
{
    return (1.0 - cos(2.0 * M_PI * (double)(k) / (double)(N)));
}
