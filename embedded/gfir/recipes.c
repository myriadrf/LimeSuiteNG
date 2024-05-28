/* *******************************************************************
FILE:		recipes.c
DESCRIPTION:	Routines for vector and matrix allocation/free.
		Linear system solving by LU decomposition.
		All are from "Numerical Recipes in C".

CONTAINTS:	
		float  *vector(nl,nh)
		int    nl,nh;

		int  *ivector(nl,nh)
		int  nl,nh;

		void free_vector(v,nl,nh)
		float *v;
		int   nl,nh;

		void free_ivector(v,nl,nh)
		int *v;
		int   nl,nh;

		float  **matrix(nrl,nrh,ncl,nch)
		int  nrl,nrh,ncl,nch;

		void free_matrix(m,nrl,nrh,ncl,nch)
		float **m;
		int   nrl,nrh,ncl,nch;

		nrerror(error_text)
		char error_text[];

		void ludcmp(a,n,indx,d)
		int n, *indx;
		float **a,*d;

		void lubksb(a,n,indx,b)
		float **a,b[];
		int n,indx[];

DATE:		
AUTHOR:		From "Numerical Recipes in C"
REVISIONS:	February 01, 1994: File created.
		Febryary 24, 2000:
			This file is taken out from my PhD sources, Srdjan.
			Unnecessary functions are removed.
			All floats are converted to doubles.
  ********************************************************************* */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "recipes.h"

#define TINY 1.0e-20

void nrerror(char* error_text);

/*****************************************************/
/*** Allocates a float vector with range [nl..nh]. ***/
/*****************************************************/
float* vector(int nl, int nh)
{
    float* const v = (float*)malloc((unsigned)(nh - nl + 1) * sizeof(float));
    if (!v)
        nrerror("Allocation failure in vector().\n");
    return (v - nl);
}

/*****************************************************/
/*** Allocates an int vector with range [nl..nh].  ***/
/*****************************************************/
int* ivector(int nl, int nh)
{
    int* const v = (int*)calloc((unsigned)(nh - nl + 1), sizeof(int));
    if (!v)
        nrerror("Allocation failure in vector().\n");
    return (v - nl);
}

/****************************************************/
/*** Frees a float vector allocated by vector().  ***/
/****************************************************/
void free_vector(float* v, int nl, int nh)
{
    free((char*)(v + nl));
}

/****************************************************/
/*** Frees an int vector allocated by ivector().  ***/
/****************************************************/
void free_ivector(int* v, int nl, int nh)
{
    free((char*)(v + nl));
}

/*******************************************/
/****  Allocates a float matrix with    ****/
/****  range [nrl..nrh] [ncl..nch].     ****/
/*******************************************/
float** matrix(int nrl, int nrh, int ncl, int nch)
{
    /*** Allocate pointers to rows. ***/
    float** m = (float**)calloc((unsigned)(nrh - nrl + 1), sizeof(float*));
    if (!m)
        nrerror("Allocation failure #1 in matrix().");
    m -= nrl;

    /*** Allocate rows and pointers to them. ***/
    for (int i = nrl; i <= nrh; i++)
    {
        m[i] = (float*)malloc((unsigned)(nch - ncl + 1) * sizeof(float));
        if (!m[i])
            nrerror("Allocation failure #2 in matrix().");
        m[i] -= ncl;
    }
    return (m);
}

/******************************************/
/****   Frees a matrix allocated by ... ***/
/****   matrix().                       ***/
/******************************************/
void free_matrix(float** m, int nrl, int nrh, int ncl, int nch)
{
    for (int i = nrh; i >= nrl; i--)
        free((char*)(m[i] + ncl));
    free((char*)(m + nrl));
}

/*************************************************/
/*** Numerical Recipes standard error handler. ***/
/*************************************************/
void nrerror(char* error_text)
{
    void exit();

    fprintf(stderr, "Numerical Recipes run-time error...\n");
    fprintf(stderr, "%s\n", error_text);
    fprintf(stderr, "Forced to exit.\n");
    exit(1);
}

/* ******************************************************************** */
/* LU Decomposition of a matrix						*/
/*	a[1:n][1:n]		system matrix,				*/
/*	n			problem dimension,			*/
/*	indx[1:n]		temporary storage,			*/
/*	d			temporary storage.			*/
/* ******************************************************************** */
int ludcmp(float** a, int n, int* indx, float* d)
{
    float* const vv = vector(1, n);
    for (int i = 1; i <= n; i++)
    {
        float big = 0.0;
        for (int j = 1; j <= n; j++)
        {
            const float temp = fabs(a[i][j]);
            if (temp > big)
                big = temp;
        }
        if (!big)
            nrerror("Singular matrix in routine LUDCMP");
        vv[i] = 1.0 / big;
    }

    int imax = 0;
    (*d) = 1.0;
    for (int j = 1; j <= n; j++)
    {
        for (int i = 1; i < j; i++)
        {
            float sum = a[i][j];
            for (int k = 1; k < i; k++)
                sum -= a[i][k] * a[k][j];
            a[i][j] = sum;
        }
        float big = 0.0;
        for (int i = j; i <= n; i++)
        {
            float sum = a[i][j];
            for (int k = 1; k < j; k++)
                sum -= a[i][k] * a[k][j];
            a[i][j] = sum;
            const float dum = vv[i] * fabs(sum);
            if (dum >= big)
            {
                big = dum;
                imax = i;
            }
        }
        if (j != imax)
        {
            for (int k = 1; k <= n; k++)
            {
                const float dum = a[imax][k];
                a[imax][k] = a[j][k];
                a[j][k] = dum;
            }
            (*d) = -(*d);
            vv[imax] = vv[j];
        }
        indx[j] = imax;
        if (fpclassify(a[j][j]) == FP_ZERO)
        {
            a[j][j] = TINY;
        }
        if (j != n)
        {
            const float dum = 1.0 / (a[j][j]);
            for (int i = j + 1; i <= n; i++)
                a[i][j] *= dum;
        }
    }
    free_vector(vv, 1, n);
    return (0);

} /* end of ludcmp() */

/* ******************************************************************** */
/* Solve system of equation						*/
/* ******************************************************************** */
void lubksb(float** a, int n, int* indx, float* b)
{
    int ii = 0;
    for (int i = 1; i <= n; i++)
    {
        const int ip = indx[i];
        float sum = b[ip];
        b[ip] = b[i];
        if (ii)
        {
            for (int j = ii; j <= i - 1; j++)
                sum -= a[i][j] * b[j];
        }
        else if (sum)
        {
            ii = i;
        }
        b[i] = sum;
    }
    for (int i = n; i >= 1; i--)
    {
        float sum = b[i];
        for (int j = i + 1; j <= n; j++)
            sum -= a[i][j] * b[j];
        b[i] = sum / a[i][i];
    }

} /*  end of lubksb() */
