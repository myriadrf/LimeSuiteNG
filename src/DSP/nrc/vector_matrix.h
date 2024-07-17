#ifndef LIME_NRC_VECTOR_MATRIX_H
#define LIME_NRC_VECTOR_MATRIX_H

namespace lime {
namespace nrc {

double* vector(int, int);
void free_vector(double*, int, int);

double** matrix(int, int, int, int);
void free_matrix(double**, int, int, int, int);

int* ivector(int, int);
void free_ivector(int*, int, int);

} // namespace nrc
} // namespace lime

#endif