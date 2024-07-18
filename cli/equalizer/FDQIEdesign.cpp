/* ************************************************************************
   FILE:	FDQIEdesign.c
   COMMENT:	Frequency Dependent Quadrature Imbalance Equiliser design.
		Cleaned up version of the original program for designing
		FIR filters as phase equiliser. The design algorithm optimises
		the deired amplitude and phase responses simultaneously.
   CONTENT:
   AUTHOR:	Lime Microsystems
   DATE:	May 17, 2020
   REVISION:	May 31, 2020:
			Made less dependent on how three points are measured
		Jul 02, 2020: Changes for new algorithms
   ************************************************************************ */

// LMS7 registers length
#define CPREC ((1 << 15) - 1) // 15-bit magnitude + sign (16-bit) coefficients precision
#define GPREC ((1 << 11) - 1) // GainI and GainQ corrector parameters precision,
// 11-bit  unsingned integer
#define PPREC (1 << 11) // 12-bit signed integer precision of tan(alpha/2)
// codeAlpha = PPREC corresponds to alpha/2=45deg

/* Include section */
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "nrc.h"

using namespace lime::nrc;

// variables declared as extern in NRC library
namespace lime {
namespace nrc {
// ==============================================================================================
// Top level variables used in optimisation
// ==============================================================================================
int N; // Number of FIR filtering stages
int P = 500; // Number of points on the frequency grid (in band)
int Ps = 500; // Number of out of band frequency points
double *x; // Normalised frequency grid

// Pointer and arrays
double *ad, *adI, *adQ; // Desired amplitude response
double *phid, *phidI, *phidQ; // Desired phase response
double *taud, *taudI, *taudQ; // Desired group delay

// Shared during the optimization
double *a; // Amplitude response
double *phi; // Phase response
double *tau; // Group delay
double *aw; // Amplitude response weighting function
double *tauw; // Group delay weighting function

double *h, *hhI, *hhQ; // Pointer and filter coefficients
    // h[0]-h[N-1] unknown filter coefficients
    // h[N] = beta, delay constant

double lambda = 0.0001; // Amplitude/group delay weighting coefficient

} // namespace nrc
} // namespace lime

// ==============================================================================================
// Function declarations
// ==============================================================================================
void FDQImodel(double *w, double *amp, double *gerr, double *perr, double *aamp, double *agerr, double *aperr, int MeasPoints);

#define P_ORDER 4
#define SYMMETRICRESPONSE

double matxdet(double A[3][3]);
void solve(double A[3][3], double x[3]);
void init(double wp, double *aamp, double *agerr, double *aperr, bool tx);
double Cgain(double w, double *a);
double Cphase(double w, double *a);
double Cgdelay(double w, double *a);
void freemem();

void gaussEliminationLS(int m, int n, double a[P_ORDER + 1][P_ORDER + 2], double *x);
void FDQImodel2(double *w,
    double *amp,
    double *gerr,
    double *perr, // xx[MeasPoints] Measured pints
    double *aamp,
    double *agerr,
    double *aperr,
    int MeasPoints); // xx[P_ORDER+1] Polynomial coefficients

// ==============================================================================================
// Returns manually measured data
// ==============================================================================================
void FDQImeasure(
    double *f, double *r_amp, double *r_gerr, double *r_perr, double *t_amp, double *t_gerr, double *t_perr, int MeasPoints)
{
#ifdef VERBOSE
    printf("FDQI MEASURED DATA:\n");
    for (int i = 0; i < MeasPoints; i++)
        printf("    [%1d] f[] = %+5.1f MHz, r_amp[] = %.5f, r_gerr[] = %.5f, r_perr[] = %+2.5f [deg]\n",
            i,
            f[i],
            r_amp[i],
            r_gerr[i],
            r_perr[i]);
    printf("\n");
    for (int i = 0; i < MeasPoints; i++)
        printf("    [%1d] f[] = %+5.1f MHz, t_amp[] = %.5f, t_gerr[] = %.5f, t_perr[] = %+2.5f [deg]\n",
            i,
            f[i],
            t_amp[i],
            t_gerr[i],
            t_perr[i]);
#endif
}

// ==============================================================================================
// Preprocess measured data before eualizers design
// ==============================================================================================
void FDQIpreproc(double *f,
    double *r_amp1,
    double *r_gerr1,
    double *r_perr1,
    double *t_amp,
    double *t_gerr,
    double *t_perr,
    double *r_amp,
    double *r_gerr,
    double *r_perr,
    double r_clk,
    double t_clk,
    double *r_w,
    double *t_w,
    int *incRXGain,
    int *incTxGain,
    int MeasPoints)
{
    // Find max amplitudes
    double r_max1 = 0.0, t_max = 0.0, r_max = 0.0;
    double t_min = 0.0, r_min = 0.0;
    int temp = 0;

    for (int i = 0; i < MeasPoints; i++)
    {
        t_amp[i] /= r_amp1[i]; // Normalize TX again and deembed phase 1 RX gain roll-off from it
            // Normalize TX again and deembed phase 1 RX gain roll-off from it
    }

    for (int i = 0; i < MeasPoints; i++)
    {
        if (t_max < t_amp[i])
            t_max = t_amp[i];
        if (r_max < r_amp[i])
            r_max = r_amp[i];
    }

    for (int i = 0; i < MeasPoints; i++)
    {

        t_amp[i] /= t_max; // B.J. (OK)
        r_amp[i] /= r_max;

        r_w[i] = f[i] / r_clk; // Normalize frequencies
        t_w[i] = f[i] / t_clk;
        r_perr[i] *= M_PI / 180.0; // Convert phase into radians
        t_perr[i] *= M_PI / 180.0;
    }

    t_max = 0.0;
    r_max = 0.0;
    t_min = 100.0;
    r_min = 100.0;

    for (int i = 0; i < MeasPoints; i++)
    {
        if (t_max < t_amp[i])
            t_max = t_amp[i];
        if (t_min > t_amp[i])
            t_min = t_amp[i];

        if (r_max < r_amp[i])
            r_max = r_amp[i];
        if (r_min > r_amp[i])
            r_min = r_amp[i];
    }

    temp = (int)(20.0 * log10(r_max / r_min) - 0.5);
    if (temp < 0)
        temp = 0;
    if (temp > 9)
        temp = 9;
    *incRXGain = temp;

#ifdef VERBOSE
    printf("[Info] incRXGain = %d, ", temp);
#endif

    temp = (int)(20.0 * log10(t_max / t_min) - 0.5);
    if (temp < 0)
        temp = 0;
    if (temp > 9)
        temp = 9;
    *incTxGain = temp;

#ifdef VERBOSE
    printf("incTXGain = %d\n", temp);
#endif

#ifdef VERBOSE
    printf("FDQI PRE-PROCESSED DATA:\n");
    for (int i = 0; i < MeasPoints; i++)
        printf("    [%1d] r_w[] = %+.5f, r_amp[] = %8.5f, r_gerr[] = %.5f, r_perr[] = %+2.5f [rad]\n",
            i,
            r_w[i],
            r_amp[i],
            r_gerr[i],
            r_perr[i]);
    printf("\n");
    for (int i = 0; i < MeasPoints; i++)
        printf("    [%1d] t_w[] = %+.5f, t_amp[] = %8.5f, t_gerr[] = %.5f, t_perr[] = %+2.5f [rad]\n",
            i,
            t_w[i],
            t_amp[i],
            t_gerr[i],
            t_perr[i]);
#endif
}

// ==============================================================================================
// INPUT:
//	wp 	Equilizer pass band (normalized and all other frequencies are)
//	w[6]	Frequencies IQ imbalance has been measured at. Recomandation is:
//			-wp <= w[] <= wp
//	amp[6], gerr[6], perr[6]
//		Measured amplitude and IQ imbalance parameters for w[] points
//	Ntaps,	Number of taps of the FD part of the equilizer (FIRs)
//
// RETURN:
//	codeI, codeQ, codeAlpha
//		Codes to configure static IQ corrector.
//		In fact this is an estimation of the parameters at w=0=DC.
//	hI[Ntaps], hQ[Ntaps]
//		FIR coefficients of I and Q equilizers.
// ==============================================================================================
void FDQIEdesign(double wp,
    double *w,
    double *amp,
    double *gerr,
    double *perr,
    int *codeI,
    int *codeQ,
    int *codeAlpha,
    int Ntaps,
    int *hI,
    int *hQ,
    bool tx,
    int slaveDevice,
    int MeasPoints)
{
    int iter;
    double fret; // used by dfpmin()
    double aamp[P_ORDER + 1], agerr[P_ORDER + 1], aperr[P_ORDER + 1];
    double aamp1[P_ORDER + 1], agerr1[P_ORDER + 1], aperr1[P_ORDER + 1];
    // Mathematical model of FDQI

    N = Ntaps; // make it globally available for other functions

    // Calculate math model parameters
    // B.J.

#ifdef SYMMETRICRESPONSE
    FDQImodel(w, amp, gerr, perr, aamp, agerr, aperr, MeasPoints);
#endif
#ifndef SYMMETRICRESPONSE
    FDQImodel2(w, amp, gerr, perr, aamp, agerr, aperr, MeasPoints);
#endif

    FDQImodel2(w, amp, gerr, perr, aamp1, agerr1, aperr1, MeasPoints);

    // Initialise the equiliser optimisation
    init(wp, aamp, agerr, aperr, tx);

    double inierr, finierr, fclk;
    char design[100];

    // Do the optimization, I channel
    h = hhI;
    ad = adI;
    phid = phidI;
    taud = taudI;
    inierr = cost(h - 1);
    dfpmin(h - 1, N + 1, 1.0e-7, &iter, &fret, cost, grad);
    finierr = cost(h - 1);
    fclk = 245.76;

    if (slaveDevice == 1)
        sprintf(design, "%sEquI_S", tx ? "TX" : "RX");
    else
        sprintf(design, "%sEquI_M", tx ? "TX" : "RX");
// sprintf(design, "%sEquI", tx ? "TX" : "RX");
#include "report.c"

    // Do the optimization, Q channel
    h = hhQ;
    ad = adQ;
    phid = phidQ;
    taud = taudQ;
    inierr = cost(h - 1);
    dfpmin(h - 1, N + 1, 1.0e-7, &iter, &fret, cost, grad);
    finierr = cost(h - 1);
    fclk = 245.76 / 2.0;

    if (slaveDevice == 1)
        sprintf(design, "%sEquQ_S", tx ? "TX" : "RX");
    else
        sprintf(design, "%sEquQ_M", tx ? "TX" : "RX");
// sprintf(design, "%sEquQ", tx ? "TX" : "RX");
#include "report.c"

    // Round the coefficients to integers
    for (int i = 0; i < N; i++)
    {
        hhI[i] = round(hhI[i] * CPREC) / (double)(CPREC);
        hhQ[i] = round(hhQ[i] * CPREC) / (double)(CPREC);
    }

    // Set hardware parameters to return
    *codeI = GPREC;
    *codeQ = GPREC;
    *codeAlpha = (int)round(PPREC * tan(aperr[0] / 2.0));
    for (int i = 0; i < Ntaps; i++)
    {
        hI[i] = (int)(hhI[i] * CPREC);
        hQ[i] = (int)(hhQ[i] * CPREC);
    }

    /* Release allocated memory and return */
    freemem();
    return;
}

// ==============================================================================================
// Extract math model parameters from FDQI measured points
// ==============================================================================================
void FDQImodel(double *w,
    double *amp,
    double *gerr,
    double *perr, // xx[MeasPoints] Measured points
    double *aamp,
    double *agerr,
    double *aperr,
    int MeasPoints) // xx[3] Polynomial coefficients
{

    for (int i = 0; i < 3; i++)
        aamp[i] = agerr[i] = aperr[i] = 0.0;

    double x1, x2, x3, x4, x6, x8;
    double s0, s1, s2, s3, s4, s6, s8;

    s0 = s1 = s2 = s3 = s4 = s6 = s8 = 0.0;
    for (int i = 0; i < MeasPoints; i++)
    {
        x1 = w[i];
        x2 = x1 * x1;
        x3 = x1 * x2;
        x4 = x2 * x2;
        x6 = x3 * x3;
        x8 = x4 * x4;
        s0 += 1.0;
        s1 += x1;
        s2 += x2;
        s3 += x3;
        s4 += x4;
        s6 += x6;
        s8 += x8;
        aamp[0] += amp[i];
        aamp[1] += amp[i] * x2;
        aamp[2] += amp[i] * x4;
        agerr[0] += gerr[i];
        agerr[1] += gerr[i] * x2;
        agerr[2] += gerr[i] * x4;
        aperr[0] += perr[i];
        aperr[1] += perr[i] * x1;
        aperr[2] += perr[i] * x3;
    }

    double gD[3][3], pD[3][3]; // Gain and phase determinants

    gD[0][0] = s0;
    gD[1][1] = s4;
    gD[2][2] = s8;
    gD[0][1] = gD[1][0] = s2;
    gD[0][2] = gD[2][0] = s4;
    gD[1][2] = gD[2][1] = s6;

    pD[0][0] = s0;
    pD[1][1] = s2;
    pD[2][2] = s6;
    pD[0][1] = pD[1][0] = s1;
    pD[0][2] = pD[2][0] = s3;
    pD[1][2] = pD[2][1] = s4;

    // Solve the equations
    solve(gD, aamp);
    solve(gD, agerr);
    solve(pD, aperr);

#ifdef VERBOSE
    printf("FDQI MATH MODEL:\n");
    printf("    Amplitude coeffs      aamp[i]:      %+10.5f %+10.5f %+10.5f\n", aamp[0], aamp[1], aamp[2]);
    printf("    Gain error coeffs    agerr[i]:      %+10.5f %+10.5f %+10.5f\n", agerr[0], agerr[1], agerr[2]);
    printf("    Phase error coeffs   aperr[i]:      %+10.5f %+10.5f %+10.5f\n", aperr[0], aperr[1], aperr[2]);
    //printf("    Constant phase error aperr[0]:      %+10.5f [rad], %+10.5f [deg]\n", aperr[0], aperr[0] * 180.0 / M_PI);
    //printf("    Phase coefficient    aperr[1]:      %+10.5f [rad], %+10.5f [deg]\n", aperr[1], aperr[1] * 180.0 / M_PI);
    //printf("    Phase coefficient    aperr[2]:      %+10.5f [rad], %+10.5f [deg]\n", aperr[2], aperr[2] * 180.0 / M_PI);
#endif
}

// ==============================================================================================
// Matrix determinant calculation
// ==============================================================================================
double matxdet(double A[3][3])
{
    return (A[0][0] * A[1][1] * A[2][2] + A[0][1] * A[1][2] * A[2][0] + A[1][0] * A[2][1] * A[0][2] - A[2][0] * A[1][1] * A[0][2] -
            A[1][0] * A[0][1] * A[2][2] - A[0][0] * A[2][1] * A[1][2]);
}

// ==============================================================================================
// Solve 3x3 system of liner equations A[][]*x[] = b[]. At the input x[]=b[], return solution in x[]
// ==============================================================================================
void solve(double A[3][3], double x[3])
{
    double D0[3][3], D1[3][3], D2[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            D0[i][j] = D1[i][j] = D2[i][j] = A[i][j];
    for (int i = 0; i < 3; i++)
    {
        D0[i][0] = x[i];
        D1[i][1] = x[i];
        D2[i][2] = x[i];
    }

    double d = matxdet(A);
    x[0] = matxdet(D0) / d;
    x[1] = matxdet(D1) / d;
    x[2] = matxdet(D2) / d;
}

// ==============================================================================================
// Allocate memory, calculate desired equiliser responses, generate initial solution, ...
// ==============================================================================================
void init(double wp, double *aamp, double *agerr, double *aperr, bool tx)
{
    /* Allocate memory for the variables */
    x = (double *)calloc(P + Ps, sizeof(double));

    adI = (double *)calloc(P + Ps, sizeof(double));
    phidI = (double *)calloc(P, sizeof(double));
    taudI = (double *)calloc(P, sizeof(double));

    adQ = (double *)calloc(P + Ps, sizeof(double));
    phidQ = (double *)calloc(P, sizeof(double));
    taudQ = (double *)calloc(P, sizeof(double));

    a = (double *)calloc(P + Ps, sizeof(double));
    phi = (double *)calloc(P, sizeof(double));
    tau = (double *)calloc(P, sizeof(double));
    aw = (double *)calloc(P + Ps, sizeof(double));
    tauw = (double *)calloc(P, sizeof(double));

    hhI = (double *)calloc(N + 1, sizeof(double));
    hhQ = (double *)calloc(N + 1, sizeof(double));

    /* Calculate desired amplitude response, phase response and group delay */
    /* In band */

    double coeff;
    // B.J.
    if (tx)
        coeff = -1.0;
    else
        coeff = 1.0;

    coeff = -1.0; // should be

    double deltax = wp / (double)(P - 1);
    double amax = 0.0;
    for (int i = 0; i < P; i++)
    {
        x[i] = (double)(i)*deltax;
        adI[i] = 1.0 / Cgain(x[i], aamp); // Amplitude flatness on both
        adQ[i] = Cgain(x[i], agerr) / Cgain(x[i], aamp); // Gain correction on Q
        phidI[i] = coeff * Cphase(x[i], aperr); // Phase correction on I
        taudI[i] = coeff * Cgdelay(x[i], aperr);
        phidQ[i] = 0.0; // No phase correction on Q
        taudQ[i] = 0.0;
        aw[i] = 1.0;
        tauw[i] = 1.0;
        if (amax < adI[i])
            amax = adI[i];
        if (amax < adQ[i])
            amax = adQ[i];
    }

    for (int i = 0; i < P; i++)
    { // Normalize in band gains to <= 1
        adI[i] /= amax;
        adQ[i] /= amax;
    }

    /* Define out of band desired amplitude response  */
    deltax = (0.5 - wp) / (double)(Ps);
    for (int i = 0; i < Ps; i++)
    {
        x[P + i] = wp + (double)(i + 1) * deltax;
        adI[P + i] = adI[P - 1]; // Cgain(x[P+i], wg); //0.0;
        adQ[P + i] = adQ[P - 1];
        // aw[P+i] = 0.00001;
        aw[P + i] = 0.000001;
    }

    // Construct initial filter coefficients and beta
    // All pass FIR is used as initial solution
    for (int i = 0; i < N; i++)
        hhI[i] = hhQ[i] = 0.0;
    hhI[(N - 1) / 2] = hhQ[(N - 1) / 2] = 1.0;

    // beta fixed to (N-1)/2 and is not being optimized
    hhI[N] = hhQ[N] = (double)((N - 1) / 2);
}

// ==============================================================================================
// Free memory blobks allocated in init()
void freemem()
{
    free(x);

    free(adI);
    free(phidI);
    free(taudI);

    free(adQ);
    free(phidQ);
    free(taudQ);

    free(a);
    free(phi);
    free(tau);
    free(aw);
    free(tauw);

    free(hhI);
    free(hhQ);
}

// ==============================================================================================
// Corrector gain
// ==============================================================================================
double Cgain(double w, double *a)
{
#ifdef SYMMETRICRESPONSE
    return (a[0] + a[1] * w * w + a[2] * w * w * w * w);
#endif
#ifndef SYMMETRICRESPONSE
    return ((((((a[4] * w) + a[3]) * w + a[2]) * w) + a[1]) * w + a[0]);
#endif
}

// ==============================================================================================
// Corrector phase [rad], without constant phase error
// ==============================================================================================
double Cphase(double w, double *a)
{
#ifdef SYMMETRICRESPONSE
    return (a[1] * w + a[2] * w * w * w);
#endif
#ifndef SYMMETRICRESPONSE
    return ((((((a[4] * w) + a[3]) * w + a[2]) * w) + a[1]) * w);
#endif
}

// ==============================================================================================
// Corrector group delay [Tclk]
// ==============================================================================================
double Cgdelay(double w, double *a)
{
#ifdef SYMMETRICRESPONSE
    return (-(a[1] + 3.0 * a[2] * w * w) / 2.0 / M_PI);
#endif
#ifndef SYMMETRICRESPONSE
    double temp = ((((4.0 * a[4] * w + 3.0 * a[3]) * w) + 2.0 * a[2]) * w + a[1]);
    temp = (-1.0) * temp / 2.0 / M_PI;
    return temp;
#endif
}

void gaussEliminationLS(int m, int n, double a[P_ORDER + 1][P_ORDER + 2], double *x)
{
    int i, j, k;
    for (i = 0; i < m - 1; i++)
    {
        // Partial Pivoting
        for (k = i + 1; k < m; k++)
        {
            // If diagonal element(absolute vallue) is smaller than any of the terms below it
            if (fabs(a[i][i]) < fabs(a[k][i]))
            {
                // Swap the rows
                for (j = 0; j < n; j++)
                {
                    double temp;
                    temp = a[i][j];
                    a[i][j] = a[k][j];
                    a[k][j] = temp;
                }
            }
        }
        // Begin Gauss Elimination
        for (k = i + 1; k < m; k++)
        {
            double term = a[k][i] / a[i][i];
            for (j = 0; j < n; j++)
            {
                a[k][j] = a[k][j] - term * a[i][j];
            }
        }
    }
    // Begin Back-substitution
    for (i = m - 1; i >= 0; i--)
    {
        x[i] = a[i][n - 1];
        for (j = i + 1; j < n - 1; j++)
        {
            x[i] = x[i] - a[i][j] * x[j];
        }
        x[i] = x[i] / a[i][i];
    }
}

// x[N], y[N] input vectors, N = MeasPoints
// coefficients stored in A[P_ORDER + 1], P_ORDER = 4

void polyFitting(int N, double *x, double *y, double *A)
{

    // an array of size 2*P_ORDER+1 for storing
    // N, Sig xi, Sig xi^2, ...., etc. which are the independent components of the normal matrix
    double X[2 * P_ORDER + 1];
    int i, j = 0;

    for (i = 0; i <= 2 * P_ORDER; i++)
    {
        X[i] = 0;
        for (j = 0; j < N; j++)
        {
            X[i] = X[i] + pow(x[j], i);
        }
    }
    // the normal augmented matrix
    double B[P_ORDER + 1][P_ORDER + 2];
    // rhs
    double Y[P_ORDER + 1];
    for (i = 0; i <= P_ORDER; i++)
    {
        Y[i] = 0;
        for (j = 0; j < N; j++)
        {
            Y[i] = Y[i] + pow(x[j], i) * y[j];
        }
    }
    for (i = 0; i <= P_ORDER; i++)
    {
        for (j = 0; j <= P_ORDER; j++)
        {
            B[i][j] = X[i + j];
        }
    }
    for (i = 0; i <= P_ORDER + 1; i++)
    {
        B[i][P_ORDER + 1] = Y[i];
    }

    // printf("The polynomial fit is given by the equation:\n");
    // printMatrix(n + 1, n + 2, B);
    gaussEliminationLS(P_ORDER + 1, P_ORDER + 2, B, A);

    for (i = 0; i < P_ORDER; i++)
    {
        printf(" (%lf)x^%d +", A[i], i);
    }
    printf(" (%lf)x^%d", A[P_ORDER], P_ORDER);
}

void FDQImodel2(double *w,
    double *amp,
    double *gerr,
    double *perr, // xx[MeasPoints] Measured pints
    double *aamp,
    double *agerr,
    double *aperr,
    int MeasPoints) // xx[5] Polynomial coefficients
{

    for (int i = 0; i <= P_ORDER; i++)
        aamp[i] = agerr[i] = aperr[i] = 0.0;

    printf("[Info] aamp(x) =");
    polyFitting(MeasPoints, w, amp, aamp);
    printf("\n");
    printf("[Info] agerr(x) =");
    polyFitting(MeasPoints, w, gerr, agerr);
    printf("\n");
    printf("[Info] aperr(x) =");
    polyFitting(MeasPoints, w, perr, aperr);
    printf("\n");

#ifdef VERBOSE
    printf("FDQI MATH MODEL:\n");
    printf("    Amplitude coeffs      aamp[i]:      %+10.5f %+10.5f %+10.5f %+10.5f %+10.5f\n",
        aamp[0],
        aamp[1],
        aamp[2],
        aamp[3],
        aamp[4]);
    printf("    Gain error coeffs    agerr[i]:      %+10.5f %+10.5f %+10.5f %+10.5f %+10.5f\n",
        agerr[0],
        agerr[1],
        agerr[2],
        agerr[3],
        agerr[4]);
    printf("    Phase error coeffs   aperr[i]:      %+10.5f %+10.5f %+10.5f %+10.5f %+10.5f\n",
        aperr[0],
        aperr[1],
        aperr[2],
        aperr[3],
        aperr[4]);
    printf("    Constant phase error aperr[0]:      %+10.5f [rad], %+10.5f [deg]\n", aperr[0], aperr[0] * 180.0 / M_PI);
#endif
}
