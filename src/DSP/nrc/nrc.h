/* --------------------------------------------------------------------------------------------
FILE:		nrc.h
DESCRIPTION:	Namespace which collects all Numerical Receipes in C functions.
CONTENT:
AUTHOR:		Lime Microsystems LTD
		LONGDENE HOUSE
		HEDGEHOG LANE
		HASLEMERE GU27 2PH
DATE:		Apr 28, 2005
REVISIONS:
   -------------------------------------------------------------------------------------------- */

#include "vector_matrix.h"
#define TINY 1.0e-20

namespace lime {
namespace nrc {

void nrerror(const char*);

// LU decomposition and backsubstitution
int ludcmp(double**, int, int*, double*);
void lubksb(double**, int, int*, double*);

// Gradient descent used to solve linear equations
void lgrad(double**, double*, double*, int, double);

// Gaus-Seidel to solve linear equations
void gauss_seidel(double**, double*, double*, int);

// Nonlinear optimization
void dfpmin(double* p, int n, double ftol, int* iter, double* fret, double (*func)(double*), void (*dfunc)(double*, double*));
void linmin(double* p, double* xi, int n, double* fret, double (*func)(double*));
double brent(double ax, double bx, double cx, double (*f)(double), double tol, double* xmin);
void mnbrak(double* ax, double* bx, double* cx, double* fa, double* fb, double* fc, double (*func)(double));
double f1dim(double x);

double cost(double*);
void grad(double*, double*);

void amphgd(int mode);

} // namespace nrc
} // namespace lime