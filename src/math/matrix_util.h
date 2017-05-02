#ifndef MATRIX_UTIL_H
#define MATRIX_UTIL_H

#define MATRIX_ADD 1
#define MATRIX_SUBTRACT 0

void matrix_column(double *matrix, double *result, int rows, int columns, int column);
void matrix_scale(double *matrix, double *result, double factor, int rows, int columns);
void matrix_cross_vector(double *matrix, double *vector, double *result, int rows, int columns);
// vector columns must equal matrix rows, result must be vector of with same number of rows of matrix
void matrix_multiply(double *a, double *b, double *result, int m, int n, int p);
// matrix a has dimensions m x n, matrix b has dimensions n x p, and the result matrix has dimensions m x p
void matrix_plus_matrix(double *a, double *b, double *result, int rows, int columns, int sign);
// if sign is nonzero matrices are added, if zero matrix b is subtracted from matrix a
void matrix_quick_print(double *matrix, int rows, int columns);
void matrix_transpose(double *matrix, double *result, int rows, int columns);
// 'rows' and 'columns' refers to the rows and columns of the input, the transpose will have opposite
void matrix_sqrt(double *matrix, double *result, int size);
// matrix square root via cholesky decomposition
// I 'cheat' here and use a gsl, in future should probably use a faster, BLAS-based library 
void matrix_inverse(double *matrix, double *result, int size);
// matrix and result must by 'size' by 'size' square matrices
// I 'cheat' here and use a gsl, in future should probably use a faster, BLAS-based library 
void matrix_identity(double *matrix, int size);
// Initialize a square matrix to the identity matrix
void matrix_diagonal(double *matrix, double value, int size);
// Initialize a square matrix with all zeroes and a diagonal of one value
void matrix_init(double *matrix, double value, int rows, int columns);
// Initialize a matrix with one value in all cells

#endif
