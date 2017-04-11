// Sourece file implementing functions for the Kalman filter

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <gsl/gsl_linalg.h>

void scale_matrix(double *matrix, double *result, double factor, int rows, int columns)
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

void matrix_cross_matrix(double *a, double *b, double *result, int m, int n, int p)
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
	for (i = 0; i < rows*columns; i++) {
		if (sign) result[i] = a[i] + b[i];
		else result[i] = a[i] - b[i];
	}
}

void matrix_quick_print(double *matrix, int rows, int columns)
{
	int i, j;
	for (i = 0; i < rows; i++) {
		printf("[");
		for (j = 0; j < columns; j++) {
			printf("%f, ", matrix[i*columns + j]);
		}
		printf("]\n");
	}
}

void matrix_transpose(double *matrix, double *result, int rows, int columns)
// 'rows' and 'columns' refers to the rows and columns of the input, obviously the transpose will have opposite
{
	size_t result_size = sizeof(double)*rows*columns;
	double *result_temp = alloca(result_size);
	int i, j;
	for (i = 0; i < columns; i++) {
		for (j = 0; j < rows; j++) {
			result_temp[i*rows + j] = matrix[j*columns+i];
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
	gsl_linalg_cholesky_decomp1(m);
	
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

void matrix_identity(double *matrix, int size)
{
	int i, j;
	for (i = 0; i < size; i++) {
		for (j = 0; j < size; j++) {
			if (i == j) matrix[i*size + j] = 1;
			else matrix[i*size + j] = 0;
		}
	}
}

void predict(double *x, double *P, double *F, double *Q, double *x_f, double *P_f, int x_dim)
{
	// x = Fx + Bu
	matrix_cross_vector(F, x, x_f, x_dim, x_dim);

	// P = FPF^t + Q
	double *transpose = alloca(x_dim*sizeof(double));
	matrix_transpose(F, transpose, x_dim, x_dim);

	double *temp = alloca(x_dim*sizeof(double));
	matrix_cross_matrix(F, P, temp, x_dim, x_dim, x_dim);
	matrix_cross_matrix(temp, transpose, P_f, x_dim, x_dim, x_dim);
	matrix_plus_matrix(P_f, Q, P_f, x_dim, x_dim, 1);
}

void update(double *x, double *z, double *P, double *H, double *R, double *x_f, double *P_f, int x_dim, int z_dim)
{
	// y = z-Hx
	double *y = alloca(z_dim*sizeof(double));
	double *temp_v = alloca(z_dim*sizeof(double));
	matrix_cross_vector(H, x, temp_v, x_dim, z_dim);
	matrix_plus_matrix(z, temp_v, y, z_dim, 1, 0);

	// K = PH^t(HPH^t + R)^-1
	double *K = alloca(x_dim*z_dim*sizeof(double));
	double *temp_k = alloca(z_dim*z_dim*sizeof(double));
	double *H_t = alloca(x_dim*z_dim*sizeof(double));
	matrix_transpose(H, H_t, x_dim, z_dim);
	matrix_cross_matrix(H, P, K, z_dim, x_dim, x_dim);
	matrix_cross_matrix(K, H_t, temp_k, z_dim, x_dim, z_dim); 
	matrix_plus_matrix(temp_k, R, temp_k, z_dim, z_dim, 1);
	matrix_inverse(temp_k, temp_k, z_dim);
	matrix_cross_matrix(P, H_t, K, x_dim, x_dim, z_dim);
	matrix_cross_matrix(K, temp_k, K, x_dim, z_dim, z_dim);

	// x = x + Ky
	matrix_cross_vector(K, y, x_f, x_dim, z_dim);
	matrix_plus_matrix(x, x_f, x_f, x_dim, 1, 1);

	// P = (I - KH)P
	double *identity = alloca(x_dim*x_dim*sizeof(double));
	matrix_identity(identity, x_dim);
	matrix_cross_matrix(K, H, P_f, x_dim, z_dim, x_dim);
	matrix_plus_matrix(identity, P_f, P_f, x_dim, x_dim, 0);
	matrix_cross_matrix(P_f, P, P_f, x_dim, x_dim, x_dim);
}

void vdm_scaled_points(double *x, double *P, double *chi, int n, double a, double k)
// My own implementation of Rudolph Van der Merwe's scaled sigma point algorithm, returns points
// chi is an (n)x(2n+1) matrix and contains each sigma point
{
	memcpy(chi, x, n*sizeof(double)); // set the first sigma point (chi sub 0) to the mean

	double scale = a*a*(n + k);
	double *temp = alloca(n * n * sizeof(double));
	scale_matrix(P, temp, scale, n, n);
	matrix_sqrt(temp, temp, n);

	int i, j;
	for (i = 1; i <= n; i++) {
		for (j = 0; j < n; j++) {
			chi[i*n + j] = x[j] + temp[i*n + j];
		}
	}
	for (i = n+1; i <= 2*n; i++) {
		for (j = 0; j < n; j++) {
			chi[i*n + j] = x[j] - temp[(i-n)*n + j];
		}
	}
}

void vdm_scaled_weights(double *w_m, double *w_c, int n, double a, double b, double k)
// My own implementation of Rudolph Van der Merwe's scaled sigma point algorithm
// returns weights in two (1)x(2n+1) matrices, one for the means and one for the covariances
{
	double sigma = a*a*(n + k) - n;

	w_m[0] = sigma/(n + sigma);
	w_c[0] = sigma/(n + sigma) + 1 - a*a + b;

	double remaining = 1/(2*(n + sigma));
	int i;
	for (i = 1; i <= 2n; i++) {
		w_m[i] = remaining;
		w_c[i] = remaining;
	}
}

void ukf_predict()
{
}

void ukf_update()
{
}
