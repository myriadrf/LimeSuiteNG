/* --------------------------------------------------------------------------------------------
FILE:		nrc.c
DESCRIPTION:	Linear system solver by LU decomposition.

CONTAINTS:	
DATE:		
AUTHOR:		From "Numerical Recipes in C"
REVISIONS:	February 01, 1994: File created.
		Febryary 24, 2000:
			This file is taken out from my PhD sources, Srdjan.
			Unnecessary functions are removed.
			All floats are converted to doubles.
		Apr 28, 2005: Functions collected into nrc namespace  
-------------------------------------------------------------------------------------------- */

#include <cstdio>
#include <cstdlib>
#include <cmath>

#include "nrc.h"

namespace lime {
namespace nrc {

/*************************************************/
/*** Numerical Recipes standard error handler. ***/
/*************************************************/
void nrerror(const char* error_text)
{
    printf("Numerical Recipes run-time error...\n");
    printf("%s\n", error_text);
    printf("Forced to exit.\n");
    //exit(1);
}

/* ******************************************************************** */
/* LU Decomposition of a matrix						*/
/*	a[1:n][1:n]		system matrix,				*/
/*	n			problem dimension,			*/
/*	indx[1:n]		temporary storage,			*/
/*	d			temporary storage.			*/
/* ******************************************************************** */
int ludcmp(double** a, int n, int* indx, double* d1)
{
    int i, imax, j, k;
    double big, dum, sum, temp;
    double* vv;

    double d = 0;
    int exit = 0;
    *d1 = 1.0;

    vv = vector(1, n);
    *d1 = 1.0;
    for (i = 1; i <= n; i++)
    {
        big = 0.0;
        for (j = 1; j <= n; j++)
            if ((temp = fabs(a[i][j])) > big)
                big = temp;
        if (!big)
        {
            nrerror("Singular matrix in routine LUDCMP");
            exit = 1;
        }
        vv[i] = 1.0 / big;
    }

    if (exit == 1)
        return -1; // Singular matrix

    for (j = 1; j <= n; j++)
    {
        for (i = 1; i < j; i++)
        {
            sum = a[i][j];
            for (k = 1; k < i; k++)
                sum -= a[i][k] * a[k][j];
            a[i][j] = sum;
        }
        big = 0.0;
        for (i = j; i <= n; i++)
        {
            sum = a[i][j];
            for (k = 1; k < j; k++)
                sum -= a[i][k] * a[k][j];
            a[i][j] = sum;
            if ((dum = vv[i] * fabs(sum)) >= big)
            {
                big = dum;
                imax = i;
            }
        }
        if (j != imax)
        {
            for (k = 1; k <= n; k++)
            {
                dum = a[imax][k];
                a[imax][k] = a[j][k];
                a[j][k] = dum;
            }
            (*d1) = -(*d1);
            d = -d;
            vv[imax] = vv[j];
        }
        indx[j] = imax;
        if (fabs(a[j][j]) <= TINY)
        {
            a[j][j] = TINY;
        }
        if (j != n)
        {
            dum = 1.0 / (a[j][j]);
            for (i = j + 1; i <= n; i++)
                a[i][j] *= dum;
        }
    }

    free_vector(vv, 1, n);
    return (0);

} /* end of ludcmp() */

/* ******************************************************************** */
/* Solve system of equation						*/
/* ******************************************************************** */
void lubksb(double** a, int n, int* indx, double* b)
{
    int i, ii = 0, ip, j;
    double sum;

    for (i = 1; i <= n; i++)
    {
        ip = indx[i];
        sum = b[ip];
        b[ip] = b[i];
        if (ii)
        {
            for (j = ii; j <= i - 1; j++)
                sum -= a[i][j] * b[j];
        }
        else if (sum)
        {
            ii = i;
        }
        b[i] = sum;
    }
    for (i = n; i >= 1; i--)
    {
        sum = b[i];
        for (j = i + 1; j <= n; j++)
            sum -= a[i][j] * b[j];
        b[i] = sum / a[i][i];
    }

} /*  end of lubksb() */

/* ******************************************************************** */
/* Using gradient descent to solve linear equations			*/
/* ******************************************************************** */
#define ITMAX 100
#define ITMIN 10
#define TOL 1.0e-7
void lgrad(double** a, double* b, double* x, int n, double alpha)
{
    //	double *g = new double[n+1];
    double e, xx;

    int iter;
    for (iter = 0; iter < ITMAX; iter++)
    {
        e = 0.0;

        // Calculate gradient
        for (int i = 1; i <= n; i++)
        {
            xx = x[i] + alpha * b[i];
            for (int j = 1; j <= n; j++)
                xx -= alpha * a[i][j] * x[j];
            e += fabs(x[i] - xx);
            x[i] = xx;
        }

        if ((e < TOL) && (iter > ITMIN))
            break;
    }
    //	printf("LGrad: iter=%d e=%lg\n", iter, e);
}

/* ******************************************************************** */
/* Gauss-Seidel iterative solution of linear equations			*/
/* ******************************************************************** */
void gauss_seidel(double** a, double* b, double* x, int n)
{
    double e, xx;

    int iter;
    for (iter = 0; iter < ITMAX; iter++)
    {
        e = 0.0;

        for (int i = 1; i <= n; i++)
        {
            xx = b[i];
            for (int j = 1; j <= n; j++)
                if (j != i)
                    xx -= a[i][j] * x[j];

            xx /= a[i][i];
            e += fabs(x[i] - xx);
            x[i] = xx;
        }

        if ((e < TOL) && (iter > ITMIN))
            break;
    }
    //	printf("Gauss-Seidel: iter=%d e=%lg\n", iter, e);
}

} // namespace nrc
} // namespace lime
