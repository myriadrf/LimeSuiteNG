/* ************************************************************************
   FILE:		amphgd.c
   COMMENT:		Calculate amplitude, phase and group delay of
			the resulting filter.
   CONTENT:
   AUTHOR:		MUMEC Design Team
   DATE:		Aug 14, 2001
   REVISION:
   ************************************************************************ */

#include <math.h>

namespace lime {
namespace nrc {

/* Import the variables we need here */
extern int N; /* Number of FIR filtering stages */
extern int P; /* Number of points on a frequency greed */
extern int Ps; /* Number of out of band frequency points */
extern double *x; /* Normalised frequency greed */
extern double *a; /* Amplitude response */
extern double *phi; /* Phase response */
extern double *tau; /* Group delay */
extern double *h; /* h[0]-h[N-1] filter coefficients */
/* h[N] = beta, delay constant */

#define EPSILON 1.0e-6

/* ************************************************************************
   ************************************************************************ */
void amphgd(int mode)
{
    double re, rek, im, imk, d, beta;
    int k, l;

    /* If mode=1, set new frequency greed to cover the whole Nyquist band */
    if (mode == 1)
    {
        d = 0.5 / (double)(P - 1);
        for (l = 0; l < P; l++)
            x[l] = d * (double)(l);
    }

    for (l = 0; l < P; l++)
    {
        re = rek = im = imk = 0.0;
        for (k = 0; k < N; k++)
        {
            d = h[k] * cos(2.0 * M_PI * k * x[l]);
            re += d;
            rek += k * d;
            d = h[k] * sin(2.0 * M_PI * k * x[l]);
            im += d;
            imk += k * d;
        }

        a[l] = sqrt(re * re + im * im);
        phi[l] = -1.0 * atan(im / (re + EPSILON));
        tau[l] = 1.0 * (re * rek + im * imk) / (a[l] * a[l] + EPSILON);
    }

    /* Out of band amplitude response */
    if (mode == 0)
    {
        for (l = P; l < (P + Ps); l++)
        {
            re = im = 0.0;
            for (k = 0; k < N; k++)
            {
                re += h[k] * cos(2.0 * M_PI * k * x[l]);
                im += h[k] * sin(2.0 * M_PI * k * x[l]);
            }

            a[l] = sqrt(re * re + im * im);
        }
    }

    /* Eliminate atan(x) function discontinuities */
    for (l = 1; l < P; l++)
    {
        d = phi[l] - phi[l - 1];
        if (fabs(d) > 0.9 * M_PI)
            for (k = l; k < P; k++)
                phi[k] += ((d > 0) ? -1.0 : 1.0) * M_PI;
    }

    /* Remove constant delay from phi[] */
    beta = h[N];
    for (l = 0; l < P; l++)
        phi[l] += 2.0 * M_PI * beta * x[l];
}

} // namespace nrc
} // namespace lime