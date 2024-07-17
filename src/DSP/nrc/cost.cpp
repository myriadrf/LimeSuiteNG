/* ************************************************************************
   FILE:		cost.c
   COMMENT:		This functions calculate cost function and its
			gradient which are required to perform
			nonlinear optimisation.
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
extern int P; /* Number of points on a frequency greed (in band) */
extern int Ps; /* Number of out of band frequency points */
extern double* x; /* Normalised frequency greed */
extern double* ad; /* Desired amplitude response */
extern double* taud; /* Desired group delay */
extern double* aw; /* Amplitude response weighting function */
extern double* tauw; /* Group delay weighting function */
extern double lambda; /* Amplitude/group delay weighting coefficient */

/* ************************************************************************
   ************************************************************************ */
double cost(double* p)
{
    double *h, re, rek, im, imk, a, tau, err, d, aerr, tauerr, beta;
    int k, l;

    h = p + 1;
    err = 0.0;
    beta = h[N];

    /* In band error function */
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

        a = sqrt(re * re + im * im);
        aerr = a - ad[l];
        tau = 1.0 * (re * rek + im * imk) / (a * a);
        tauerr = tau - taud[l] - beta;

        err += 0.5 * (1.0 - lambda) * aw[l] * aerr * aerr;
        err += 0.5 * lambda * tauw[l] * tauerr * tauerr;
    }

    /* Out of band error function */
    for (l = P; l < (P + Ps); l++)
    {

        re = im = 0.0;
        for (k = 0; k < N; k++)
        {
            re += h[k] * cos(2.0 * M_PI * k * x[l]);
            im += h[k] * sin(2.0 * M_PI * k * x[l]);
        }

        a = sqrt(re * re + im * im);
        aerr = a - ad[l];
        err += 0.5 * (1.0 - lambda) * aw[l] * aerr * aerr;
    }
    return (err);
}
/* ************************************************************************
   ************************************************************************ */
void grad(double* p, double* g)
{
    double *h, re, rek, im, imk, a, tau, d, aerr, tauerr, beta;
    double *errg, rei, reki, imi, imki, ai, taui;
    int i, k, l;

    h = p + 1;
    errg = g + 1;
    beta = h[N];
    for (i = 0; i <= N; i++)
        errg[i] = 0.0;

    /* In band gradient calculation */
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

        a = sqrt(re * re + im * im);
        aerr = a - ad[l];
        tau = 1.0 * (re * rek + im * imk) / (a * a);
        tauerr = tau - taud[l] - beta;

        for (i = 0; i < N; i++)
        {
            rei = cos(2.0 * M_PI * i * x[l]);
            reki = i * rei;
            imi = sin(2.0 * M_PI * i * x[l]);
            imki = i * imi;
            ai = (re * rei + im * imi) / a;
            taui = 1.0 * (rei * rek + re * reki + imi * imk + im * imki) / (a * a);
            taui += 2.0 * tau * ai / a;

            errg[i] += (1.0 - lambda) * aw[l] * aerr * ai;
            errg[i] += lambda * tauw[l] * tauerr * taui;
        }
        // errg[N] -= lambda*tauw[l]*tauerr;
        // Do not optimize beta, keep it integer (N-1)/2 as set initially
    }

    /* Out of band gradient calculation */
    for (l = P; l < (P + Ps); l++)
    {

        re = im = 0.0;
        for (k = 0; k < N; k++)
        {
            re += h[k] * cos(2.0 * M_PI * k * x[l]);
            im += h[k] * sin(2.0 * M_PI * k * x[l]);
        }

        a = sqrt(re * re + im * im);
        aerr = a - ad[l];

        for (i = 0; i < N; i++)
        {
            rei = cos(2.0 * M_PI * i * x[l]);
            imi = sin(2.0 * M_PI * i * x[l]);
            ai = (re * rei + im * imi) / a;

            errg[i] += (1.0 - lambda) * aw[l] * aerr * ai;
        }
    }
}

} // namespace nrc
} // namespace lime
