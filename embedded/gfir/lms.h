/* ************************************************************************ 
   FILE:	lms.h
   COMMENT:	
   CONTENT:
   AUTHOR:	Lime Microsystems
   DATE:	Feb 24, 2000
   REVISION:
   ************************************************************************ */
#ifndef LMS_GFIR_HDR
#define LMS_GFIR_HDR

#include <stdlib.h>
#include <math.h>

/* This two parameters define filter symmetry */
#define POSITIVE (1)
#define NEGATIVE (-1)

/* Number of points on the frequency grid */
//#define LMS_POINTS 1000
#define LMS_POINTS 4000

/* Amplitude correction functions */
float InvSinc(float);
float InvSincS(float, float, float);
float One(float);

/* Amplitude correction functions we can support */
#define NONE One /* Constant */
#define ONE One /* as well */
#define INVSINC InvSinc /* Inverse sinc */
#define INVSINCS InvSincS /* Shifted Inverse Sinc */

/* Cosine filter type selection */
#define RAISEDCOSINE 1
#define ROOTRAISEDCOSINE 2

int lms(float* hr,
    float* hi,
    float* hcsd,
    int n,
    float* w,
    float* des,
    float* weight,
    int p,
    int cprec,
    int csdprec,
    int symmetry,
    int** bincode,
    int** csdcode,
    int** csdcoder);

#endif
