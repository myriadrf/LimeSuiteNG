#include <math.h>
#include "nrc.h"

#define ITMAX 600
#define EPS 1.0e-10

namespace lime {
namespace nrc {

void dfpmin(double* p, int n, double ftol, int* iter, double* fret, double (*func)(double*), void (*dfunc)(double*, double*))
{
    int j, i, its;
    double fp, fae, fad, fac;
    double *xi, *g, *dg, *hdg;
    double** hessin;

    hessin = matrix(1, n, 1, n);
    xi = vector(1, n);
    g = vector(1, n);
    dg = vector(1, n);
    hdg = vector(1, n);
    fp = (*func)(p);
    (*dfunc)(p, g);
    for (i = 1; i <= n; i++)
    {
        for (j = 1; j <= n; j++)
            hessin[i][j] = 0.0;
        hessin[i][i] = 1.0;
        xi[i] = -g[i];
    }
    for (its = 1; its <= ITMAX; its++)
    {
        *iter = its;
        linmin(p, xi, n, fret, func);
        if (2.0 * fabs(*fret - fp) <= ftol * (fabs(*fret) + fabs(fp) + EPS))
        {
            free_vector(hdg, 1, n);
            free_vector(dg, 1, n);
            free_vector(g, 1, n);
            free_vector(xi, 1, n);
            free_matrix(hessin, 1, n, 1, n);
            return;
        }
        fp = (*fret);
        for (i = 1; i <= n; i++)
            dg[i] = g[i];
        *fret = (*func)(p);
        (*dfunc)(p, g);
        for (i = 1; i <= n; i++)
            dg[i] = g[i] - dg[i];
        for (i = 1; i <= n; i++)
        {
            hdg[i] = 0.0;
            for (j = 1; j <= n; j++)
                hdg[i] += hessin[i][j] * dg[j];
        }
        fac = fae = 0.0;
        for (i = 1; i <= n; i++)
        {
            fac += dg[i] * xi[i];
            fae += dg[i] * hdg[i];
        }
        fac = 1.0 / fac;
        fad = 1.0 / fae;
        for (i = 1; i <= n; i++)
            dg[i] = fac * xi[i] - fad * hdg[i];
        for (i = 1; i <= n; i++)
            for (j = 1; j <= n; j++)
                hessin[i][j] += fac * xi[i] * xi[j] - fad * hdg[i] * hdg[j] + fae * dg[i] * dg[j];
        for (i = 1; i <= n; i++)
        {
            xi[i] = 0.0;
            for (j = 1; j <= n; j++)
                xi[i] -= hessin[i][j] * g[j];
        }
    }
    nrerror("Too many iterations in DFPMIN");
}

} // namespace nrc
} // namespace lime

#undef ITMAX
#undef EPS
