/* *******************************************************************
FILE:		nrecutils.c
DESCRIPTION:	Utility functions required by numerical receipes
		routines.

CONTAINTS:	

DATE:		
AUTHOR:		
REVISIONS:
********************************************************************* */

#include <cstdlib>
#include "vector_matrix.h"
#include "nrc.h"

#define TINY 1.0e-20

namespace lime {
namespace nrc {

/*****************************************************/
/*** Allocates a double vector with range [nl..nh]. ***/
/*****************************************************/
double* vector(int nl, int nh)
{
    double* v;

    v = (double*)malloc((unsigned)(nh - nl + 1) * sizeof(double));
    if (!v)
        nrerror((char*)"Allocation failure in vector().\n");
    return (v - nl);
}

/*****************************************************/
/*** Allocates an int vector with range [nl..nh].  ***/
/*****************************************************/
int* ivector(int nl, int nh)
{
    int* v;

    v = (int*)calloc((unsigned)(nh - nl + 1), sizeof(int));
    if (!v)
        nrerror((char*)"Allocation failure in vector().\n");
    return (v - nl);
}

/****************************************************/
/*** Frees a double vector allocated by vector().  ***/
/****************************************************/
void free_vector(double* v, int nl, int nh)
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
/****  Allocates a double matrix with    ****/
/****  range [nrl..nrh] [ncl..nch].     ****/
/*******************************************/
double** matrix(int nrl, int nrh, int ncl, int nch)
{
    int i;
    double** m;

    /*** Allocate pointers to rows.   ****/
    m = (double**)calloc((unsigned)(nrh - nrl + 1), sizeof(double*));
    if (!m)
        nrerror((char*)"Allocation failure #1 in matrix().");
    m -= nrl;

    /***  Allocate rows and pointers to them.  ***/
    for (i = nrl; i <= nrh; i++)
    {
        m[i] = (double*)malloc((unsigned)(nch - ncl + 1) * sizeof(double));
        if (!m[i])
            nrerror((char*)"Allocation failure #2 in matrix().");
        m[i] -= ncl;
    }
    return (m);
}

/******************************************/
/****   Frees a matrix allocated by ... ***/
/****   matrix().                       ***/
/******************************************/
void free_matrix(double** m, int nrl, int nrh, int ncl, int nch)
{
    int i;

    for (i = nrh; i >= nrl; i--)
        free((char*)(m[i] + ncl));
    free((char*)(m + nrl));
}

} // namespace nrc
} // namespace lime
