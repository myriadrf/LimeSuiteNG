
/* ************************************************************************
   FILE:        corrections.c
   COMMENT:     Functions to correct desired amplitude response
		like inverse sinc, constant, ...
   CONTENT:
		float InvSinc(x)
		float x;

		float One(x)
		float x;

   AUTHOR:      Lime Microsystems
   DATE:        Feb 25, 2000
   REVISION:            
   ************************************************************************ */
#ifdef _MSC_VER
    #define _USE_MATH_DEFINES
#endif

#include <math.h>

/* To avoid numerical problems in calculating sin(x)/x */
#define DELTA 0.0

/* ************************************************************ */
/* Inverse sinc function to correct DAC sinc envelope 		*/
/* ************************************************************ */
float InvSinc(float x)
{
    if (fpclassify(x) == FP_ZERO)
        return (1.0);
    return (fabs((M_PI * x) / (sin(M_PI * x) + DELTA)));
}

/* ************************************************************ */
/* Invsinc function shifted into lower frequency stage.		*/
/* ************************************************************ */
float InvSincS(float x, float k, float x0)
{
    const float xin = (x + x0) / k;
    if (fpclassify(xin) == FP_ZERO)
        return (1.0);
    return (fabs((M_PI * xin) / (sin(M_PI * xin) + DELTA)));
}

/* ************************************************************ */
/* Unity function, no correction is involved.			*/
/* ************************************************************ */
float One(float x)
{
    return (1.0);
}
