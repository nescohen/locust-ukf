// Sourece file implementing functions for the Kalman filter

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <gsl/gsl_linalg.h>

#include "kalman.h"
#include "../math/matrix_util.h"

void predict(double *x, double *P, double *F, double *Q, double *x_f, double *P_f, int x_dim)
{
	// x = Fx + Bu
	matrix_cross_vector(F, x, x_f, x_dim, x_dim);

	// P = FPF^t + Q
	double *transpose = alloca(x_dim*sizeof(double));
	matrix_transpose(F, transpose, x_dim, x_dim);

	double *temp = alloca(x_dim*sizeof(double));
	matrix_multiply(F, P, temp, x_dim, x_dim, x_dim);
	matrix_multiply(temp, transpose, P_f, x_dim, x_dim, x_dim);
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
	matrix_multiply(H, P, K, z_dim, x_dim, x_dim);
	matrix_multiply(K, H_t, temp_k, z_dim, x_dim, z_dim); 
	matrix_plus_matrix(temp_k, R, temp_k, z_dim, z_dim, 1);
	matrix_inverse(temp_k, temp_k, z_dim);
	matrix_multiply(P, H_t, K, x_dim, x_dim, z_dim);
	matrix_multiply(K, temp_k, K, x_dim, z_dim, z_dim);

	// x = x + Ky
	matrix_cross_vector(K, y, x_f, x_dim, z_dim);
	matrix_plus_matrix(x, x_f, x_f, x_dim, 1, 1);

	// P = (I - KH)P
	double *identity = alloca(x_dim*x_dim*sizeof(double));
	matrix_identity(identity, x_dim);
	matrix_multiply(K, H, P_f, x_dim, z_dim, x_dim);
	matrix_plus_matrix(identity, P_f, P_f, x_dim, x_dim, 0);
	matrix_multiply(P_f, P, P_f, x_dim, x_dim, x_dim);
}

void vdm_scaled_points(double *x, double *P, double *chi, int n, double a, double k)
// My own implementation of Rudolph Van der Merwe's scaled sigma point algorithm, returns points
// chi is an (2n+1)x(n) matrix and contains each sigma point
{
	memcpy(chi, x, n*sizeof(double)); // set the first sigma point (chi sub 0) to the mean

	double scale = a*a*(n + k);
	double *temp = alloca(n * n * sizeof(double));
	matrix_scale(P, temp, scale, n, n);
	matrix_sqrt(temp, temp, n);

	int i, j;
	for (i = 1; i <= n; i++) {
		for (j = 0; j < n; j++) {
			chi[i*n + j] = x[j] + temp[(i-1) + j*n];
		}
	}
	for (i = n+1; i <= 2*n; i++) {
		for (j = 0; j < n; j++) {
			chi[i*n + j] = x[j] - temp[(i-n-1) + j*n];
		}
	}
}

void vdm_scaled_weights(double *w_m, double *w_c, int n, double a, double b, double k)
// My own implementation of Rudolph Van der Merwe's scaled sigma point algorithm
// returns weights in two (2n+1)x(1) matrices, one for the means and one for the covariances
{
	double lambda = a*a*(n + k) - n;

	w_m[0] = lambda/(n + lambda);
	w_c[0] = lambda/(n + lambda) + 1 - a*a + b;

	double remaining = 1/(2*(n + lambda));
	int i;
	for (i = 1; i <= 2*n; i++) {
		w_m[i] = remaining;
		w_c[i] = remaining;
	}
}

void vdm_get_all(double *x, double *P, int n, double a, double b, double k, double *chi, double *w_m, double *w_c)
// chi - (2n+1)x(n)
// w_m - (2n+1)x(1)
// w_c - (2n+1)x(1)
{
	vdm_scaled_points(x, P, chi, n, a, k);
	vdm_scaled_weights(w_m, w_c, n, a, b, k);
}

void ukf_init_options(Ukf_options *options, int state_size, int measurement_size)
{
	options->f = NULL;
	options->h = NULL;
	options->state_mean = NULL;
	options->measurement_mean = NULL;
	options->state_diff = NULL;
	options->measure_diff = NULL;
	options->state_add = NULL;

	options->n = state_size;
	options->m = measurement_size;
}

void ukf_predict(double *x, double *P, double *Q, double delta_t, double *chi, double *gamma, double *weight_m, double *weight_c,  double *x_f, double *P_f, Ukf_options *options)
{
	int i;
	int n = options->n;
	for (i = 0; i <= 2*n; i++) {
		(*options->f)(chi + i*n, gamma + i*n, delta_t, n);
	}

	double *state_f = alloca(n*sizeof(double));
	double *cov_f = alloca(n*n*sizeof(double));
	for (i = 0; i < n; i++) {
		state_f[i] = 0.f;
	}
	for (i = 0; i < n*n; i++) {
		cov_f[i] = 0.f;
	}

	double *temp_x = alloca(n*sizeof(double));
	if (options->state_mean == NULL) {
		// sum the scaled means 
		for (i = 0; i <= 2*n; i++) {
			memcpy(temp_x, gamma + i*n, n*sizeof(double));
			matrix_scale(temp_x, temp_x, weight_m[i], n, 1);
			matrix_plus_matrix(temp_x, state_f, state_f, n, 1, 1);
		}
	}
	else (*options->state_mean)(gamma, weight_m, state_f, n, 2*n + 1);

	// sum the scaled covariances
	double *temp_P = alloca(n*n*sizeof(double));
	for (i = 0; i <= 2*n; i++) {
		memcpy(temp_P, gamma + i*n, n*sizeof(double));
		if (options->state_diff == NULL) matrix_plus_matrix(temp_P, state_f, temp_P, n, 1, 0);
		else (*options->state_diff)(temp_P, state_f, temp_P, n);
		matrix_transpose(temp_P, temp_x, n, 1);
		matrix_scale(temp_P, temp_P, weight_c[i], n, 1);
		matrix_multiply(temp_P, temp_x, temp_P, n, 1, n);
		matrix_plus_matrix(temp_P, cov_f, cov_f, n, n, 1);
	}
	matrix_plus_matrix(cov_f, Q, cov_f, n, n, 1);

	memcpy(x_f, state_f, n*sizeof(double));
	memcpy(P_f, cov_f, n*n*sizeof(double));
}

void ukf_update(double *x, double *z, double *P, double *R, double *gamma, double *weight_m, double *weight_c, double *x_f, double *P_f, Ukf_options *options)
// n is the number of dimensions in state space, m is the number of dimensions in measurement space
{
	int n = options->n;
	int m = options->m;
	// Find maximum matrix size
	int mat_max;
	if (m > n) mat_max = m;
	else mat_max = n;
	double *temp = alloca(mat_max*mat_max*sizeof(double));
	double *temp_t = alloca(mat_max*mat_max*sizeof(double));

	// Z = h(Y)
	double *zeta = alloca(m*(2*n + 1)*sizeof(double));
	int i;
	for (i = 0; i <= 2*n; i++) {
		(*options->h)(gamma + i*n, zeta + i*m, n, m);
	}

	double *u_z = alloca(m*sizeof(double));
	// u_z = sum[ w_m*Z .. Mean of the sigma points in measurement space
	if (options->measurement_mean == NULL) {
		for (i = 0; i < m; i++) {
			u_z[i] = 0;
		}
		for (i = 0; i <= 2*n; i++) {
			memcpy(temp, zeta + i*m, m*sizeof(double));
			matrix_scale(temp, temp, weight_m[i], m, 1);
			matrix_plus_matrix(temp, u_z, u_z, m, 1, 1);
		}
	}
	else (*options->measurement_mean)(zeta, weight_m, u_z, m, 2*n + 1);

#ifdef DEBUG
	//debug printing for testing
	//printf("TRANSFORMED SIGMA POINTS MEAN\n");
	//matrix_quick_print(u_z, options->m, 1);
#endif

	// P_z = sum[ w_c*(Z-u_z)(Z-u_z)^t ] + R .. Covariance of the sigma points in measurement space
	double *P_z = alloca(m*m*sizeof(double));
	for (i = 0; i < m*m; i++) {
		P_z[i] = 0.f;
	}
	for (i = 0; i <= 2*n; i++) {
		if (options->measure_diff == NULL) matrix_plus_matrix(zeta + i*m, u_z, temp, m, 1, 0);
		else (*options->measure_diff)(zeta + i*m, u_z, temp, m);
		matrix_transpose(temp, temp_t, m, 1);
		matrix_scale(temp, temp, weight_c[i], m, m);
		matrix_multiply(temp, temp_t, temp, m, 1, m);
		matrix_plus_matrix(temp, P_z, P_z, m, m, 1);
	}
	matrix_plus_matrix(P_z, R, P_z, m, m, 1);

	// y = z - u_z .. Residual
	double *y = alloca(m*sizeof(double));
	matrix_plus_matrix(z, u_z, y, m, 1, 0);

	// P_xz = sum[ w_c*(Y - x)(Z - u_z)^t .. Cross covariance of state and measurements
	double *P_xz = alloca(n*m*sizeof(double));
	for (i = 0; i < n*m; i++) {
		P_xz[i] = 0.f;
	}
	for (i = 0; i <= 2*n; i++) {
		if (options->state_diff == NULL) matrix_plus_matrix(gamma + i*n, x, temp, n, 1, 0);
		else (*options->state_diff)(gamma + i*n, x, temp, n);
		if (options->measure_diff == NULL) matrix_plus_matrix(zeta + i*m, u_z, temp_t, m, 1, 0);
		else (*options->measure_diff)(zeta + i*m, u_z, temp_t, m);
		matrix_transpose(temp_t, temp_t, m, 1);
		matrix_scale(temp, temp, weight_c[i], n, m);
		matrix_multiply(temp, temp_t, temp, n, 1, m);
		matrix_plus_matrix(temp, P_xz, P_xz, n, m, 1);
	}
	
	// K = P_xz * P_z^-1 .. Kalman Gain
	double *K = alloca(n*m*sizeof(double));
	matrix_inverse(P_z, temp, m);
	matrix_multiply(P_xz, temp, K, n, m, m);

	// x_f = x + K*y .. New state estimate
	matrix_multiply(K, y, temp, n, m, 1);
	if (options->state_add == NULL) matrix_plus_matrix(temp, x, x_f, n, 1, 1);
	else (*options->state_add)(x, temp, x_f, n);

	// P_f = P - K*P_z*K^t .. New covariance
	matrix_multiply(K, P_z, temp, n, m, m);
	matrix_transpose(K, temp_t, n, m);
	matrix_multiply(temp, temp_t, temp, n, m, n);
	matrix_plus_matrix(P, temp, P_f, n, n, 0);
}
