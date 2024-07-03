/* ************************************************************************ 
   FILE:		gfir_lms.c
   COMMENT:		Calculate the coefficients of general FIR filter
			using LMS optimisation. Filter is defined by
			pass band and stop band frequencies (w1, w2, a1, a2).
   CONTENT:

   AUTHOR:		Lime Microsystems
   DATE:		Feb 11, 2000
   REVISION:		Feb 25, 2000: Adapted for LMS
			Nov 30, 2007: Tiger project
			Apr 21, 2008: Asymmetrical number of points in pass/stop bands
   ************************************************************************ */

#include <stdlib.h>
#include <stdio.h>
#include "lms.h"
#include "dfilter.h"

enum {
    CPREC = 16, /* Coefficients precision */
    CSDPREC = 16 /* CSD Coefficients precision */
};

/* *********************************************************************** */
int gfir_lms(struct dfilter* hr,
    struct dfilter* hi,
    struct dfilter* hcsd,
    int n,
    float w1,
    float w2,
    float a1,
    float a2,
    int cprec,
    int csdprec,
    float (*correction)(float))
{
    /* Points on a frequency grid */
    const int points = LMS_POINTS / 2;

    /* Allocate memory */
    float* const weights = (float*)calloc(2 * points, sizeof(float));
    float* const desired = (float*)calloc(2 * points, sizeof(float));
    float* const w = (float*)calloc(2 * points, sizeof(float));

    int** const bincode = (int**)calloc(n, sizeof(int*));
    for (int i = 0; i < n; i++)
        bincode[i] = (int*)calloc(cprec + 1, sizeof(int));

    int** const csdcode = (int**)calloc(n, sizeof(int*));
    for (int i = 0; i < n; i++)
        csdcode[i] = (int*)calloc(cprec + 1, sizeof(int));

    int** const csdcoder = (int**)calloc(n, sizeof(int*));
    for (int i = 0; i < n; i++)
        csdcoder[i] = (int*)calloc(cprec + 1, sizeof(int));

    int** const xpx = (int**)calloc(n, sizeof(int*));
    for (int i = 0; i < n; i++)
        xpx[i] = (int*)calloc(cprec + 1, sizeof(int));

    int** const xmx = (int**)calloc(n, sizeof(int*));
    for (int i = 0; i < n; i++)
        xmx[i] = (int*)calloc(cprec + 1, sizeof(int));

    int** const x = (int**)calloc(n, sizeof(int*));
    for (int i = 0; i < n; i++)
        x[i] = (int*)calloc(cprec + 1, sizeof(int));

    /* Configure the filter with infinite precision coefficients */
    hr->m = n - 1;
    hr->n = 0;
    hr->a = (float*)calloc(n, sizeof(float));
    hr->b = (float*)calloc(1, sizeof(float));
    hr->b[0] = 1.0;
    hr->nw = 2 * points;
    hr->w = w;

    /* Configure the filter with integer coefficients */
    hi->m = n - 1;
    hi->n = 0;
    hi->a = (float*)calloc(n, sizeof(float));
    hi->b = (float*)calloc(1, sizeof(float));
    hi->b[0] = 1.0;
    hi->nw = 2 * points;
    hi->w = w;

    /* Configure the filter with CSD coefficients */
    hcsd->m = n - 1;
    hcsd->n = 0;
    hcsd->a = (float*)calloc(n, sizeof(float));
    hcsd->b = (float*)calloc(1, sizeof(float));
    hcsd->b[0] = 1.0;
    hcsd->nw = 2 * points;
    hcsd->w = w;

    /* Construct grid, desired response and weighting function */
    /* 0-w1 band */
    const int p1 = points / 4;
    float deltaw = w1 / (float)(p1 - 1);
    for (int i = 0; i < p1; i++)
    {
        w[i] = (float)(i)*deltaw;
        desired[i] = a1 * (correction)(w[i]);
        weights[i] = 1.0; //0.003;
    }
    /* w2-0.5 band */
    const int p2 = 2 * points - p1;
    deltaw = (0.5 - w2) / (float)(p2 - 1);
    for (int i = 0; i < p2; i++)
    {
        w[i + p1] = w2 + (float)(i)*deltaw;
        desired[i + p1] = a2 * (correction)(w[i + p1]);
        weights[i + p1] = 0.0001; //1.0;
    }

    /* Do LMS optimization */
    lms(hr->a, hi->a, hcsd->a, n, w, desired, weights, 2 * points, cprec, csdprec, POSITIVE, bincode, csdcode, csdcoder);

#if 0
  /* Print the results */
  printf(" ****************** Filter Design Results ******************* \n");
  printf("Filter type:            GENERAL FIR\n");
  printf("Number of taps:         %d\n", n);
  printf("Bands:                  0.0-%g, and %g-0.5\n", w1, w2);
  printf("Desired amplitude:      %g for band 1 and %g for band 2.\n", a1, a2);
  printf("Coefficients precision: %d\n", cprec);
  printf("CSD precision:          %d\n\n", csdprec);

  printf("Impulse response with the CSD coefficients:\n");
  for(i=0; i<n/2; i++) 
    printf("\th(%2d) = %-10g \t h(%2d) = %-10g\n", 
	   i, hcsd->a[i], n-i-1, hcsd->a[n-i-1]);
  if(n%2) printf("\th(%2d) = %-10g\n", n/2, hcsd->a[n/2]);
#endif

    /* Ready to exit */
    free(weights);
    free(desired);
    free(w);
    for (int i = 0; i < n; i++)
    {
        free(bincode[i]);
    }
    free(bincode);
    for (int i = 0; i < n; i++)
    {
        free(csdcode[i]);
    }
    free(csdcode);
    for (int i = 0; i < n; i++)
    {
        free(csdcoder[i]);
    }
    free(csdcoder);
    for (int i = 0; i < n; i++)
    {
        free(xpx[i]);
    }
    free(xpx);
    for (int i = 0; i < n; i++)
    {
        free(xmx[i]);
    }
    free(xmx);
    for (int i = 0; i < n; i++)
    {
        free(x[i]);
    }
    free(x);

    return 0;
}

void GenerateFilter(int n, float w1, float w2, float a1, float a2, float* coeffs)
{
    struct dfilter hr, hi, hcsd; /* Filter transfer functions */
    /* Find the filter coefficients */
    gfir_lms(&hr, &hi, &hcsd, n, w1, w2, a1, a2, CPREC, CSDPREC, NONE);
    for (int i = 0; i < n; i++)
        coeffs[i] = hi.a[i];
}
