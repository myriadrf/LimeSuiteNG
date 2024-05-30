/* ************************************************************************ 
   FILE:	lms.c
   COMMENT:	Optimal FIR filter design by LMS algorithm.
   CONTENT:
		float Case1F(float w, int i)
		float Case2F(float w, int i)
		float Case3F(float w, int i)
		float Case4F(float w, int i)

   AUTHOR:	Lime Microsystems
   DATE:	Feb 24, 2000
   REVISION:
   ************************************************************************ */
#ifdef _MSC_VER
    #define _USE_MATH_DEFINES
#endif

#include "lms.h"
#include "recipes.h"
#include "rounding.h"

/* Filter parity constants */
#define EVEN 0
#define ODD 1

/* Declare functions from Numerical Recipes that are used here */
float* vector(int, int);
int* ivector(int, int);
float** matrix(int, int, int, int);

/* ************************************************************************ 
 *  Trigonometric functions for CASE1, CASE2, CASE3 and CASE4 filters
 *
 *  INPUTS:
 *  float	w	- Normalised frequency [0, 0.5]
 *  int		i
 *
 *  RETURN VALUE:
 *  Value of trigonometric function
 * ************************************************************************ */
float Case1F(float w, int i)
{
    return (cos(2.0 * M_PI * w * ((float)(i)-1.0)));
}

float Case2F(float w, int i)
{
    return (cos(2.0 * M_PI * w * ((float)(i)-0.5)));
}

float Case3F(float w, int i)
{
    return (sin(2.0 * M_PI * w * (float)(i)));
}

float Case4F(float w, int i)
{
    return (sin(2.0 * M_PI * w * ((float)(i)-0.5)));
}

/* ************************************************************************ 
 *	OUTPUT:
 *	float 	hr[n]		- Filter impulse response
 *	float 	hi[n]		- Filter impulse response rounded to integer
 *	float 	hcsd[n]		- Filter impulse response rounded to CSD
 *
 *	INPUTS:
 *	int 	n		- Number of taps
 *	float	w[p]		- Frequency points
 *	float	des[p]		- Filter desired response
 *	float	weight[p]	- Wighting function
 *	int	p		- Number of points on a frequency scale,
 *	int	cprec		- Desired coefficients precision,
 *	int	csdprec		- CSD coefficients precision,
 *	int	symmetry	- Filter response symmetry
 *
 *	RETURN VALUE:
 *	0			- if everything is OK
 *	-1			- otherwise
 * ************************************************************************ */
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
    int** csdcoder)
{
    /* Check the correctness of inputs */
    if ((hr == NULL) || (w == NULL) || (des == NULL) || (weight == NULL))
        return (-1);
    if (n == 0)
        return (-1);
    if ((symmetry != POSITIVE) && (symmetry != NEGATIVE))
        return (-1);

    /* Parity of the filter (ODD or EVEN) */
    int parity = EVEN;
    if (n % 2)
        parity = ODD;

    /* Calculate L. If we use integer arithmetic, L is n/2 */
    /* for all cases except for CASE1 filters */
    int L = n / 2; /* Number of terms in Hr(w) sum */
    if ((symmetry == POSITIVE) && (parity == ODD))
        L++;

    /* Find which trigonometric function to use depending on filter type */
    float (*f)(float, int); /* Trigonometric function in Hr(w) */
    if ((symmetry == POSITIVE) && (parity == ODD))
    { /* Case 1 */
        f = Case1F;
    }
    else if ((symmetry == POSITIVE) && (parity == EVEN))
    { /* Case 2 */
        f = Case2F;
    }
    else if ((symmetry == NEGATIVE) && (parity == ODD))
    { /* Case 2 */
        f = Case3F;
    }
    else if ((symmetry == NEGATIVE) && (parity == EVEN))
    { /* Case 2 */
        f = Case4F;
    }
    else
    { /* This should never happen but ... */
        return (-1);
    }

    /* Allocate memory for a[1:L], A[1:L][1:L] and index[1:L]. */
    /* Functions from Numerical Recipes are used because I hate  */
    /* FORTRAN like indexing (starting from 1, not 0). */
    float* const a = vector(1, L); /* Coefficients */
    float** const A = matrix(1, L, 1, L);
    int* const index = ivector(1, L);

    /* Set A, a and index all to zeroes */
    for (int j = 1; j <= L; j++)
    {
        a[j] = 0.0;
        index[j] = 0;
        for (int i = 1; i <= L; i++)
            A[i][j] = 0.0;
    }

    /* OK, ready to fill up the equations */
    for (int k = 0; k < p; k++)
    {
        for (int j = 1; j <= L; j++)
        {
            const float fwkj = (f)(w[k], j);
            A[j][j] += weight[k] * fwkj * fwkj;
            a[j] += weight[k] * des[k] * fwkj;
            for (int i = j + 1; i <= L; i++)
            {
                const float fwki = (f)(w[k], i);
                A[i][j] += weight[k] * fwki * fwkj;
                A[j][i] += weight[k] * fwkj * fwki;
            }
        }
    }

    /* Solve the equations */
    float d = NAN;
    ludcmp(A, L, index, &d);
    lubksb(A, L, index, a);

    /* Calculate impulse response h[] from a[] */
    for (int i = 0; i < n; i++)
        hr[i] = 0.0;
    for (int i = 0; i < L; i++)
        hr[i] = 0.5 * a[L - i];

    /* Resolve CASE1 for i=L-1 */
    if ((symmetry == POSITIVE) && (parity == ODD))
        hr[L - 1] = a[1];

    /* Construct other half of hr[] using symmetry */
    for (int i = 0; i < n / 2; i++)
        hr[n - i - 1] = symmetry * hr[i];

    /* Round the filter coefficients to the nearest integer value */
    round2int(hr, hi, n, cprec);

    /* Round the filter coefficients to the nearest CSD code */
    round2csd(hr, hcsd, n, cprec, csdprec, bincode, csdcode, csdcoder);

    /* Free allocated memory */
    free_vector(a, 1, L);
    free_matrix(A, 1, L, 1, L);
    free_ivector(index, 1, L);

    /* That's all, let's go home */
    return (0);
}
