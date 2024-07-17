#include "nrc.h"

namespace lime {
namespace nrc {

extern int ncom; /* defined in LINMIN */
extern double *pcom, *xicom, (*nrfunc)(double *);

double f1dim(double x)
{
    int j;
    double f, *xt; //,*vector();
    //void free_vector();

    xt = vector(1, ncom);
    for (j = 1; j <= ncom; j++)
        xt[j] = pcom[j] + x * xicom[j];
    f = (*nrfunc)(xt);
    free_vector(xt, 1, ncom);
    return f;
}

} // namespace nrc
} // namespace lime