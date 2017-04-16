#ifndef MATRIX_UTIL_H
#define MATRIX_UTIL_H

void scale_matrix(double *matrix, double *result, double factor, int rows, int columns);
void matrix_cross_vector(double *matrix, double *vector, double *result, int rows, int columns);
void matrix_cross_matrix(double *a, double *b, double *result, int m, int n, int p);
void matrix_plus_matrix(double *a, double *b, double *result, int rows, int columns, int sign);
void matrix_quick_print(double *matrix, int rows, int columns);
void matrix_transpose(double *matrix, double *result, int rows, int columns);
void matrix_sqrt(double *matrix, double *result, int size);
void matrix_inverse(double *matrix, double *result, int size);
void matrix_identity(double *matrix, int size);

#endif
