// just a test file. not for production

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <gsl/gsl_linalg.h>

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

void generate_transform(double delta_t, double *result)
{
	result[0] = 1;
	result[1] = delta_t;
	result[2] = 0;
	result[3] = 1;
}

int main()
{
	double state[2] = {10, 4.5}; // x, 2-vector to represent current state with position(0) and velocity(1)
	double state_covariance[4] = {500, 0, 0, 49}; // P, state covariance, 2x2 matrix
	double process_noise[4] = {0, 0, 0, 0}; // Q, for now we have no process noise
	double m_function[2] = {1, 0}; // H, only measuring position
	double m_covariance[1] = {5}; // R
	double state_transition[4]; // F
	double prior[2];
	double posterior[2];
	double prior_covariance[4];
	double posterior_covariance[4];
	double measurements[10] = {10, 10.45, 11, 12, 13, 14, 16, 17, 19, 21.5};

	generate_transform(0.1, state_transition);

	int i;
	for (i = 0; i < 10; i++) {
		predict(state, state_covariance, state_transition, process_noise, prior, prior_covariance, 2);
		update(prior, &measurements[i], prior_covariance, m_function, m_covariance, posterior, posterior_covariance, 2, 1);
		memcpy(state, posterior, sizeof(state));
		memcpy(state_covariance, posterior_covariance, sizeof(state_covariance));
	}

	matrix_quick_print(posterior, 2, 1);
	matrix_quick_print(posterior_covariance, 2, 2);

	return 0;
}
