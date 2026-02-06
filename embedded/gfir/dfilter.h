#ifndef LIME_GFIR_DFILTER_H
#define LIME_GFIR_DFILTER_H

#include <string.h>

/* ************************************************************************ 
   FILE:	dfilter.h
   COMMENT:	Define some structures used in DSP calculations.
   CONTENT:
   AUTHOR:	Lime Microsystems
   DATE:	Feb 8, 2000
   REVISION:
   ************************************************************************ */
#define POINTS 256 /* Maximum number of points to calculate */
//#define POINTS  40000	/* Maximum number of points to calculate */

/** @brief Digital filter transfer function parameters */
struct dfilter {
    int m; ///< Polynomial order of the denominator
    int n; ///< Polynomial order of the nominator
    float* a; ///< Array of denominator coefficients
    float* b; ///< Array of nominator coefficients
    float* w; ///< Points on a frequency grid
    int nw; ///< Number of points in @ref w
    float amplitude[POINTS]; ///< To store calculated data
    float phase[POINTS]; ///< The phase of the data
    float logamp[POINTS]; ///< Amplitude in log scale
    float max; ///< Maximum of amplitude
};

void dfilter_init(struct dfilter* f)
{
    memset(f, 0, sizeof(struct dfilter));
}

void dfilter_destroy(struct dfilter* f)
{
    if (f->a)
        free(f->a);
    f->a = NULL;

    if (f->b)
        free(f->b);
    f->b = NULL;

    if (f->w)
        free(f->w);
    f->w = NULL;
}

#endif // LIME_GFIR_DFILTER_H
