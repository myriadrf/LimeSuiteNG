/* ************************************************************************ 
   FILE:	rounding.c
   COMMENT:	Functions related to filter coefficients rounding to
		integer values or CSD form.
   CONTENT:
   AUTHOR:	Lime Microsystems
   DATE:	Jun 18, 2000
   REVISION:	Nov 30, 2007:	Tiger project
   ************************************************************************ */
#include <math.h>
#include <stdio.h>

#include "rounding.h"

void int2csd(int a, int cprec, int csdprec, int* bincode, int* csdcode, int* csdcoder);
int csd2int(int cprec, int* code);

/* ************************************************************************ 
   ************************************************************************ */
void round2int(float* a, float* b, int n, int cprec)
{
    for (int i = 0; i < n; i++)
    {
        const int k = a[i] > 0.0 ? 1 : -1;
        b[i] = (int)(a[i] * (1 << cprec) + k * 0.5);
        b[i] /= (float)(1 << cprec);
    }
}

/* ************************************************************************ 
   ************************************************************************ */
void round2csd(float* a, float* b, int n, int cprec, int csdprec, int** bincode, int** csdcode, int** csdcoder)
{
    for (int i = 0; i < n; i++)
    {
        const int k = a[i] > 0.0 ? 1 : -1;
        int ia = (int)(a[i] * (1 << cprec) + k * 0.5);
        int2csd(ia, cprec, csdprec, bincode[i], csdcode[i], csdcoder[i]);
        ia = csd2int(cprec, csdcoder[i]);
        b[i] = (float)(ia) / (float)(1 << cprec);
    }
}

/* ************************************************************************ 
   ************************************************************************ */
void printcode(int** code, int n, int cprec)
{
    /* Find maximum nonzero bits per coefficient */
    int csdprec = 0;
    for (int i = 0; i < n; i++)
    {
        int negative = 0;
        for (int j = 0; j <= cprec; j++)
            if (code[i][j] != 0)
                negative++;
        if (negative > csdprec)
            csdprec = negative;
    }

    /* Check symmetry of the filter */
    int symmetry = 0;
    if (csd2int(cprec, code[0]) == csd2int(cprec, code[n - 1]))
        symmetry = 1;
    else
        symmetry = -1;

    float sumh = 0.0;
    float sume = 0.0;
    float sumo = 0.0;
    for (int i = 0; i < n; i++)
    {
        const int ih = csd2int(cprec, code[i]);
        const float h = (float)ih / (float)(1 << cprec);
        sumh += fabs(h);
        if (i % 2)
            sumo += fabs(h);
        else
            sume += fabs(h);

        if ((ih != 0) && (i < (n + 1) / 2))
        {
            int negative = 0;
            int shift = 0;
            for (int j = 0; j <= cprec; j++)
            {
                if (code[i][j] == -1)
                    negative++;
                if (code[i][j] != 0)
                    shift = j;
            }

            int sign = 0;
            if (negative <= (csdprec - 1))
                sign = 1;
            else
                sign = -1;

            shift = cprec - shift;
            if (fabs(h) * (1 << shift) > 1.0)
                shift--;

            printf("h(%2d) = %11lg = %2d x (", i, h, sign);
            for (int j = cprec; j >= 0; j--)
            {
                if (sign * code[i][j] == 1)
                {
                    // printf(" +1/2^%d", cprec-j-shift);
                    printf(" +1/2^%d", cprec - j);
                }
                else if (sign * code[i][j] == -1)
                {
                    // printf(" -1/2^%d", cprec-j-shift);
                    printf(" -1/2^%d", cprec - j);
                }
            }
            // printf(" ) / ( 2^%d )\n", shift);
            printf(" )\n");
        }
        else if (ih != 0)
        {
            printf("h(%2d) = %11lg = %2d x h(%2d)\n", i, h, symmetry, n - 1 - i);
        }
        else
        {
            printf("h(%2d) = %11lg\n", i, 0.0);
        }
    }
    printf("Sum of all abs(coefficients): %lg\n", sumh);
    printf("Sum of even coefficients: %lg\n", sume);
    printf("Sum of odd  coefficients: %lg\n\n", sumo);
}

/* ************************************************************************ 
   Print CSD code in the form of two common sub-expressions sharing 
   ************************************************************************ */
void print_cses_code(int** xpx, int** xmx, int** x, int n, int cprec)
{
    /* Check symmetry of the filter */
    int symmetry = 0;
    if ((csd2int(cprec, xpx[0]) == csd2int(cprec, xpx[n - 1])) && (csd2int(cprec, xmx[0]) == csd2int(cprec, xmx[n - 1])) &&
        (csd2int(cprec, x[0]) == csd2int(cprec, x[n - 1])))
        symmetry = 1;
    else
        symmetry = -1;

    for (int i = 0; i < n; i++)
    {
        const int ixpx = csd2int(cprec, xpx[i]);
        const int ixmx = csd2int(cprec, xmx[i]);
        const int ix = csd2int(cprec, x[i]);
        const float h = (1.0 + 1.0 / 4.0) * (float)ixpx / (float)(1 << cprec) +
                        (1.0 - 1.0 / 4.0) * (float)ixmx / (float)(1 << cprec) + (float)ix / (float)(1 << cprec);

        if ((fpclassify(h) != FP_ZERO) && (i < (n + 1) / 2))
        {
            printf("h(%2d) = %11lg = ", i, h);
            if (ixpx)
            {
                printf("(1+1/4)x(");
                for (int j = cprec; j >= 0; j--)
                {
                    if (xpx[i][j] == 1)
                    {
                        printf(" +1/2^%d", cprec - j);
                    }
                    else if (xpx[i][j] == -1)
                    {
                        printf(" -1/2^%d", cprec - j);
                    }
                }
                printf(" )");
            }

            if (ixmx)
            {
                if (ixpx)
                    printf(" + (1-1/4)x(");
                else
                    printf("(1-1/4)x(");

                for (int j = cprec; j >= 0; j--)
                {
                    if (xmx[i][j] == 1)
                    {
                        printf(" +1/2^%d", cprec - j);
                    }
                    else if (xmx[i][j] == -1)
                    {
                        printf(" -1/2^%d", cprec - j);
                    }
                }
                printf(" )");
            }

            if (ix)
            {
                if (ixpx || ixmx)
                    printf(" + (");
                else
                    printf("(");

                for (int j = cprec; j >= 0; j--)
                {
                    if (x[i][j] == 1)
                    {
                        printf(" +1/2^%d", cprec - j);
                    }
                    else if (x[i][j] == -1)
                    {
                        printf(" -1/2^%d", cprec - j);
                    }
                }
                printf(" )");
            }

            printf("\n");
        }
        else if (fpclassify(h) != FP_ZERO)
        {
            printf("h(%2d) = %11lg = %2d x h(%2d)\n", i, h, symmetry, n - 1 - i);
        }
        else
        {
            printf("h(%2d) = %11lg\n", i, 0.0);
        }
    }
}

/* ************************************************************************ 
    int a; Input integer to be converted into CSD code 
    int cprec; Integer precision 
    int csdprec; CSD precistion 
    int* bincode; Binary code 
    int* csdcode; CSD code 
    int* csdcoder; CSD code rounded to 'csdprec' nonzero bits 
   ************************************************************************ */
void int2csd(int a, int cprec, int csdprec, int* bincode, int* csdcode, int* csdcoder)
{

    int sign = 0;
    if (a < 0)
    {
        a *= -1;
        sign = -1;
    }
    else
    {
        sign = 1;
    }

    /* Generate binary code of input */
    for (int i = 0; i < cprec; i++)
    {
        if (a & (1 << i))
            bincode[i] = 1;
        else
            bincode[i] = 0;
    }
    bincode[cprec] = 0;

    /* Construct CSD code */
    int ci = 0;
    for (int i = 0; i < cprec; i++)
    {
        int ci1 = 0;
        if ((ci + bincode[i] + bincode[i + 1]) > 1)
            ci1 = 1;

        csdcode[i] = sign * (bincode[i] + ci - 2 * ci1);
        bincode[i] *= sign;
        ci = ci1;
    }
    csdcode[cprec] = sign * ci;

    /* Round CSD code */
    int nzeroes = 0;
    for (int i = cprec; i >= 0; i--)
    {
        if (csdcode[i] != 0)
            nzeroes++;

        if (nzeroes <= csdprec)
            csdcoder[i] = csdcode[i];
        else
            csdcoder[i] = 0;
    }
}

/* ************************************************************************ 
   ************************************************************************ */
int csd2int(int cprec, int* code)
{
    int a = 0;
    for (int i = cprec; i >= 0; i--)
        a = a * 2 + code[i];

    return (a);
}

/* ************************************************************************ 
	Extract x+x>>2 and x-x>>2 subexpressions from the CSD code
   ************************************************************************ */
void csesh(int** code, int n, int cprec, int** xpx, int** xmx, int** x)
{
    /* Set code matrices to zero */
    for (int i = 0; i < n; i++)
    {
        for (int k = 0; k <= cprec; k++)
        {
            xpx[i][k] = 0;
            xmx[i][k] = 0;
            x[i][k] = 0;
        }
    }

    /* Extract two common subexpressions from all filter coefficients */
    for (int i = 0; i < n; i++)
    {
        int k = cprec;
        while (1)
        {
            /* Find next nonzero element in CSD code */
            for (; k >= 0; k--)
                if (code[i][k] != 0)
                    break;

            if (k == -1)
            { /* There are no more nonzero digits */
                break;
            }
            if (k == 0)
            { /* It is the last digit */
                x[i][0] = code[i][0];
                break;
            }
            if (k == 1)
            { /* Two more digits left */
                x[i][0] = code[i][0];
                x[i][1] = code[i][1];
                break;
            }
            if ((code[i][k] == 1) && (code[i][k - 2] == 1))
            {
                xpx[i][k] = 1;
                code[i][k] = 0;
                code[i][k - 2] = 0;
            }
            else if ((code[i][k] == -1) && (code[i][k - 2] == -1))
            {
                xpx[i][k] = -1;
                code[i][k] = 0;
                code[i][k - 2] = 0;
            }
            else if ((code[i][k] == 1) && (code[i][k - 2] == -1))
            {
                xmx[i][k] = 1;
                code[i][k] = 0;
                code[i][k - 2] = 0;
            }
            else if ((code[i][k] == -1) && (code[i][k - 2] == 1))
            {
                xmx[i][k] = -1;
                code[i][k] = 0;
                code[i][k - 2] = 0;
            }
            else
            {
                x[i][k] = code[i][k];
                code[i][k] = 0;
            }
        }
    }
}
