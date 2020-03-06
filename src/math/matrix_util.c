#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <gsl/gsl_linalg.h>

#include "matrix_util.h"

void matrix_column(double *matrix, double *result, int rows, int columns, int column)
// result must be at least the size of one matrix column
{
	int i; 
	for (i = 0; i < rows; i++) {
		result[i] = matrix[i*columns + column];
	}
}

void matrix_scale(double *matrix, double *result, double factor, int rows, int columns)
{
	int i;
	for (i = 0; i < rows * columns; i++) {
		result[i] = factor*matrix[i];
	}
}

void matrix_cross_vector(double *matrix, double *vector, double *result, int rows, int columns)
// vector columns must equal matrix rows, result must be vector of with same number of rows of matrix
{
	size_t result_size = sizeof(double)*rows;
	double *result_temp = alloca(result_size);
	int i, j;
	for (i = 0; i < rows; i++) {
		result_temp[i] = 0;
		for (j = 0; j < columns; j++) {
			result_temp[i] += matrix[i*columns + j]*vector[j];
		}
	}
	memcpy(result, result_temp, result_size);
}

void matrix_multiply(double *a, double *b, double *result, int m, int n, int p)
// matrix a has dimensions m x n, matrix b has dimensions n x p, and the result matrix has dimensions m x p
{
	size_t result_size = m*p*sizeof(double);
	double *result_temp = alloca(result_size);
	int i, j, k;
	for (i = 0; i < m; i++) {
		for (j = 0; j < p; j++) {
			result_temp[i*p + j] = 0;
			for (k = 0; k < n; k++) {
				result_temp[i*p + j] += a[i*n + k]*b[k*p +j];
			}
		}
	}
	memcpy(result, result_temp, result_size);
}

void matrix_plus_matrix(double *a, double *b, double *result, int rows, int columns, int sign)
// if sign is nonzero matrices are added, if zero matrix b is subtracted from matrix a
{
	int i;
	if (sign) {
		for (i = 0; i < rows*columns; i++) {
			result[i] = a[i] + b[i];
		}
	}
	else {
		for (i = 0; i < rows*columns; i++) {
			result[i] = a[i] - b[i];
		}
	}
}

void matrix_quick_print(double *matrix, int rows, int columns)
{
	int i, j;
	for (i = 0; i < rows; i++) {
		printf("[");
		for (j = 0; j < columns; j++) {
			printf("% 010.4f, ", matrix[i*columns + j]);
		}
		printf("]\n");
	}
	printf("\n");
}

void matrix_transpose(double *matrix, double *result, int rows, int columns)
// 'rows' and 'columns' refers to the rows and columns of the input, the transpose will have opposite
{
	size_t result_size = sizeof(double)*rows*columns;
	double *result_temp = alloca(result_size);
	int i, j;
	for (i = 0; i < columns; i++) {
		for (j = 0; j < rows; j++) {
			result_temp[i*rows + j] = matrix[j*columns + i];
		}
	}
	memcpy(result, result_temp, result_size);
}

void matrix_sqrt(double *matrix, double *result, int size)
// matrix square root via cholesky decomposition
// I 'cheat' here and use a gsl, in future should probably use a faster, BLAS-based library 
{
	double *temp = alloca(size*size*sizeof(double));
	memcpy(temp, matrix, size*size*sizeof(double));

	gsl_matrix_view m = gsl_matrix_view_array(temp, size, size);
	gsl_linalg_cholesky_decomp(&m.matrix);
	
	memcpy(result, temp, size*size*sizeof(double));
}

void matrix_inverse(double *matrix, double *result, int size)
// matrix and result must by 'size' by 'size' square matrices
// I 'cheat' here and use a gsl, in future should probably use a faster, BLAS-based library 
{
	double *result_temp = alloca(size*size*sizeof(double));
	int s;
	gsl_matrix_view m = gsl_matrix_view_array(matrix, size, size);
	gsl_matrix_view inv = gsl_matrix_view_array(result_temp, size, size);

	gsl_permutation *p = gsl_permutation_alloc(size);
	gsl_linalg_LU_decomp(&m.matrix, p, &s);
	gsl_linalg_LU_invert(&m.matrix, p, &inv.matrix);

	gsl_permutation_free(p);

	memcpy(result, result_temp, size*size*sizeof(double));
}

void matrix_diagonal(double *matrix, double value, int size)
// Initialize a square matrix with all zeroes and a diagonal of one value
{
	memset(matrix, 0.0, size*size*sizeof(double));
	int i;
	for (i = 0; i < size; i++) {
		matrix[i*size + i] = value;
	}
}

void matrix_identity(double *matrix, int size)
// Initialize a square matrix to the identity matrix
{
	matrix_diagonal(matrix, 1.0, size);
}

void matrix_init(double *matrix, double value, int rows, int columns)
// Initialize a matrix with one value in all cells
{
	int i;
	for (i = 0; i < rows*columns; i++) {
		matrix[i] = value;
	}
}

void matrix_init_column(double *matrix, int rows, int columns, ...)
{
	va_list valist;
	va_start(valist, columns);
	
	int i, j;
	for (i = 0; i < columns; i++) {
		double *column = va_arg(valist, double *);
		for (j = 0; j < rows; j++) {
			matrix[i*rows + j] = column[j];
		}
	}

	va_end(valist);
}
