// just a test file. not for production

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <gsl/gsl_linalg.h>

void matrix_cross_vector(double *matrix, double *vector, double *result, int rows, int columns)
// vector columns must equal matrix rows, result must be vector of with same number of rows of matrix
{
	int i, j;
	for (i = 0; i < rows; i++) {
		result[i] = 0;
		for (j = 0; j < columns; j++) {
			result[i] += matrix[i*columns + j]*vector[j];
		}
	}
}

void matrix_cross_matrix(double *a, double *b, double *result, int m, int n, int p)
// matrix a has dimensions m x n, matrix b has dimensions n x p, and the result matrix has dimensions m x p
{
	int i, j, k;
	for (i = 0; i < m; i++) {
		for (j = 0; j < p; j++) {
			result[i*p + j] = 0;
			for (k = 0; k < n; k++) {
				result[i*p + j] += a[i*n + k]*b[k*p +j];
			}
		}
	}
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

void matrix_quick_print(double *matrix, int row, int columns)
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
	int i, j;
	for (i = 0; i < columns; i++) {
		for (j = 0; j < rows; j++) {
			result[i*rows + j] = matrix[j*columns+i];
		}
	}
}

void matrix_inverse(double *matrix, double *result, int size)
// matrix and result must by 'size' by 'size' square matrices
// I 'cheat' here and use a gsl, in future should probably use a faster, BLAS-based library 
{
	int s;
	gsl_matrix_view m = gsl_matrix_view_array(matrix, size, size);
	gsl_matrix_view inv = gsl_matrix_view_array(result, size, size);

	gsl_permutation *p = gsl_permutation_alloc(size);
	gsl_linalg_LU_decomp(&m.matrix, p, &s);
	gsl_linalg_LU_invert(&m.matrix, p, &inv.matrix);

	gsl_permutation_free(p);
}

void predict(double *x, double *P, double *F, double *Q, double *x_f, double *P_f, int x_dim)
{
	matrix_cross_vector(F, x, x_f, x_dim, x_dim);

	double *transpose = alloca(x_dim*sizeof(double));
	matrix_transpose(F, transpose, x_dim, x_dim);

	double *temp = alloca(x_dim*sizeof(double));
	matrix_cross_matrix(F, P, temp, x_dim, x_dim, x_dim);
	matrix_cross_matrix(temp, transpose, P_f, x_dim, x_dim, x_dim);
	matrix_plus_matrix(P_f, Q, P_f, x_dim, x_dim, 1);
}

void update(double *x, double *z, double *P, double *H, double *R, double *x_f, double *P_f, int x_dim, int z_dim)
{
	double *y = alloca(z_dim*sizeof(double));
	double *temp_v = alloca(z_dim*sizeof(double));

	matrix_cross_vector(H, x, temp_v, x_dim, z_dim);
	matrix_plus_matrix(z, temp_v, y, z_dim, 1);

	double *K = alloca(z_dim*z_dim*sizeof(double));
	double *H_t = allocate(x_dim*z_dim*sizeof(double));
	double *temp_m_a = alloca(z_dim*z_dim*sizeof(double));
	double *temp_m_b = alloca(z_dim*z_dim*sizeof(double));

	matrix_transpose(H, H_t, z_dim, z_dim);
	matrix_cross_matrix(P, H_t, temp_m_a, x_dim, x_dim, z_dim);

	//TODO: finish this implementation! good work, you're getting close
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
	double process_noise[4] = {0, 0, 0, 0}; // Q, for now we have no process nois

	double state_transition[4];
	generate_transform(0.1, state_transition);

	double prior[2];
	double prior_covariance[4];
	predict(state, state_covariance, state_transition, process_noise, prior, prior_covariance, 2);
	printf("assumption = {%f, %f}\n", state[0], state[1]);
	printf("prior = {%f, %f}\n", prior[0], prior[1]);
	printf("covariance = {%f, %f, %f, %f}\n", prior_covariance[0], prior_covariance[1], prior_covariance[2], prior_covariance[3]);

	return 0;
}
