/* --------------------------------------------------------------------------------------------
FILE:		fft.h
DESCRIPTION:	Namespace which collects all fft functions.
CONTENT:
AUTHOR:		Lime Microsystems LTD
DATE:		Mar 25, 2005
REVISIONS:	Apr 18, 2005:	Nuttall window function added
		Nov 20, 2015:	qspectrum() added 
   -------------------------------------------------------------------------------------------- */
#include <complex>

namespace Fourier {

void spectrum(
    int N, int Rb, double* x, double* a, double* s, double* p, double Amax, bool Win, double Nfloor, std::complex<double>* X);
void qspectrum(int N, int Rb, std::complex<double>* x, double* a, double* s, double* p, double Amax, bool Win, double Nfloor);
void fftr(std::complex<double>* x, int N);
void fftc(std::complex<double>* x, int N, int sign);
double Nuttall(int k, int N);
double Hann(int k, int N);

} // namespace Fourier
