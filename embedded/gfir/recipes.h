/* ************************************************************************ 
   FILE:	recipes.h
   COMMENT:	clean up C isms and fix compiler warnings. 
   CONTENT:
   AUTHOR:	M. H. Reilly 
   DATE:	May 17, 2017
   REVISION:
   ************************************************************************ */
#ifndef RECIP_GFIR_HDR
#define RECIP_GFIR_HDR

int ludcmp(float** a, int n, int* indx, float* d);

void lubksb(float** a, int n, int* indx, float* b);

void free_vector(float* v, int nl, int nh);
void free_ivector(int* v, int nl, int nh);
void free_matrix(float** m, int nrl, int nrh, int ncl, int nch);

#endif
