#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <assert.h>
#include <fenv.h>

#include "kalman.h"
#include "matrix_util.h"
#include "quaternion_util.h"

#define SIZE_STATE 9
#define SIZE_MEASUREMENT 3
#define SENSOR_VARIANCE (double)11 + (double)1 / (double)9

#define ALPHA 0.1
#define BETA 2.0
#define KAPPA (double)(3 - SIZE_STATE)

#define POWER_ITERATIONS 100

void rotate_mrp(double orientation[3], double rotation[3], double result[3])
{
	//TODO: fix this routine, it does not work
	double temp[3];
	// equation from NASA paper "Attitude Estimation Using Modified Rodrigues Parameters"
	// 1/4{(1 - |P|^2)w - 2w x P + 2(w*P)P

	// (1 - |P|^2)w
	double factor = 1 - pow(vector_magnitude(orientation), 2);
	scale_matrix(rotation, result, factor, 3, 1);

	// -2w x P
	scale_matrix(rotation, temp, -2, 3, 1);
	cross_product(temp, orientation, temp);
	matrix_plus_matrix(temp, result, result, 3, 1, MATRIX_ADD);

	// 2(w*P)P
	factor = dot_product(rotation, orientation);
	scale_matrix(orientation, temp, factor, 3, 1);
	matrix_plus_matrix(temp, result, result, 3, 1, MATRIX_ADD);

	// 1/4{ .. }
	scale_matrix(result, result, 0.25, 3, 1);
}

void alt_rotate_mrp(double orientation[3], double rotation[3], double result[3])
{
	double angle;
	double axis[3];
	double qo[4];
	double qr[4];
	
	normalize_vector(orientation, axis);
	angle = 4*atan(vector_magnitude(orientation));
	gen_quaternion(angle, axis, qo);
	normalize_vector(rotation, axis);
	angle = 4*atan(vector_magnitude(rotation));
	gen_quaternion(angle, axis, qr);

	mult_quaternion(qo, qr, qo);
	decomp_quaternion(qo, axis);
	normalize_vector(axis, result);
	angle = vector_magnitude(axis);
	vector_by_scalar(result, tan(angle)/4, result);
}

void process_model(double *curr_state, double *next_state, double delta_t, int n)
{
	assert(n == SIZE_STATE);

	double mrp[3];
	double omega[3];
	double alpha[3];

	double result_mrp[3];

	memcpy(mrp, curr_state, sizeof(mrp));
	memcpy(omega, curr_state + 3, sizeof(omega));
	memcpy(alpha, curr_state + 6, sizeof(alpha));
	memcpy(next_state + 6, alpha, sizeof(alpha));

	scale_matrix(alpha, alpha, delta_t, 3, 1);
	matrix_plus_matrix(alpha, omega, omega, 3, 1, MATRIX_ADD);
	memcpy(next_state + 3, omega, sizeof(omega));

	scale_matrix(omega, omega, delta_t, 3, 1);
	alt_rotate_mrp(mrp, omega, result_mrp);
	memcpy(next_state, result_mrp, sizeof(result_mrp));
}

void measurement(double *state, double *measurement, int n, int m)
{
	measurement[0] = state[3];
	measurement[1] = state[4];
	measurement[2] = state[5];
}

void mean_state(double *points, double *weights, double *mean, int size, int count)
// compute weighted mean of state points, specifically the modified rodrigues parameter component
{
	double sum_mrp_angle[2] = {0.0}; // format: { cos(a), sin(a) }
	double sum_mrp_vector[3] = {0.0};
	double sum_state[6] = {0.0};
	int i;
	for (i = 0; i < count; i++) {
		double mrp[3];
		double angle;
		double rest_state[6];
		memcpy(rest_state, points + i*size + 3, sizeof(rest_state));
		scale_matrix(rest_state, rest_state, weights[i], 6, 1);
		matrix_plus_matrix(rest_state, sum_state, sum_state, 6, 1, MATRIX_ADD);

		memcpy(mrp, points + i*size, sizeof(mrp));
		angle = vector_magnitude(mrp);
		angle = atan(angle)*4;
		normalize_vector(mrp, mrp);
		sum_mrp_angle[0] += cos(angle)*weights[i];
		sum_mrp_angle[1] += sin(angle)*weights[i];
		scale_matrix(mrp, mrp, weights[i], 3, 1);
		matrix_plus_matrix(mrp, sum_mrp_vector, sum_mrp_vector, 3, 1, MATRIX_ADD);
	}
	double mean_mrp[3];
	double mean_angle = atan2(sum_mrp_angle[1], sum_mrp_angle[0]);
	normalize_vector(sum_mrp_vector, mean_mrp);
	scale_matrix(mean_mrp, mean_mrp, tan(mean_angle/4), 3, 1);

	memcpy(mean, mean_mrp, sizeof(mean_mrp));
	memcpy(mean + 3, sum_state, sizeof(sum_state));
}

void state_error(double *point, double *mean, double *error, int n)
{
	assert(n == SIZE_STATE);
	// preform naive difference for angular velocity and acceleration
	matrix_plus_matrix(point + 3, mean + 3, error + 3, 6, 1, MATRIX_SUBTRACT);

	double angle;
	double axis[3];
	double point_q[4];
	double mean_q[4];
	double final_q[4];
	angle = 4*atan(vector_magnitude(point));
	normalize_vector(point, axis);
	gen_quaternion(angle, axis, point_q);
	angle = 4*atan(vector_magnitude(mean));
	normalize_vector(mean, axis);
	gen_quaternion(angle, axis, mean_q);
	
	int i;
	for (i = 1; i < 4; i++) {
		mean_q[i] *= -1;
	}
	mult_quaternion(point_q, mean_q, final_q);

	decomp_quaternion(final_q, axis);
	angle = vector_magnitude(axis);
	normalize_vector(axis, axis);
	scale_matrix(axis, error, tan(angle/4), 3, 1);
}

void custom_scaled_points(double *x, double *P, double *chi, int n, double a, double k)
// modified scaled points algorithm for use with MRPs
{
	memcpy(chi, x, n*sizeof(double)); // set the first sigma point (chi sub 0) to the mean

	double scale = a*a*(n + k);
	double *temp = alloca(n * n * sizeof(double));
	scale_matrix(P, temp, scale, n, n);
	matrix_sqrt(temp, temp, n);

	int i, j;
	for (i = 1; i <= n; i++) {
		double point_rotation[3];
		for (j = 0; j < 3; j++) {
			point_rotation[j] = temp[(i-1) + j*n];
		}
		rotate_mrp(x, point_rotation, chi + i*n);
		for (j = 3; j < n; j++) {
			chi[i*n + j] = x[j] + temp[(i-1) + j*n];
		}
	}
	for (i = n+1; i <= 2*n; i++) {
		double point_rotation[3];
		for (j = 0; j < 3; j++) {
			point_rotation[j] = -1*temp[(i-1) + j*n];
		}
		rotate_mrp(x, point_rotation, chi + i*n);
		for (j = 3; j < n; j++) {
			chi[i*n + j] = x[j] - temp[(i-n-1) + j*n];
		}
	}
}

int main()
{
	// double start[3] = {1, 0, 0};
	// double rotation[3] = {1, 0, 0};
	// double result[3];
	// double alt_result[3];

	// rotate_mrp(start, rotation, result);
	// alt_rotate_mrp(start, rotation, alt_result);
	// printf("Reg - {%f, %f, %f}\n", result[0], result[1], result[2]);
	// printf("Alt - {%f, %f, %f}\n", alt_result[0], alt_result[1], alt_result[2]);
	// return 0;
	int i;
	double state[SIZE_STATE] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double covariance[SIZE_STATE*SIZE_STATE];
	double new_state[SIZE_STATE];
	double new_covariance[SIZE_STATE*SIZE_STATE];
	matrix_diagonal(covariance, 10, SIZE_STATE);

	double R[SIZE_MEASUREMENT*SIZE_MEASUREMENT];
	double Q[SIZE_STATE*SIZE_STATE];
	matrix_diagonal(R, SENSOR_VARIANCE, SIZE_MEASUREMENT);
	matrix_diagonal(Q, 10, SIZE_STATE);

	double chi[SIZE_STATE*(2*SIZE_STATE + 1)];
	double gamma[SIZE_STATE*(2*SIZE_STATE + 1)];
	double w_m[2*SIZE_STATE + 1];
	double w_c[2*SIZE_STATE + 1];
	
	double delta_t = 0.1;

	double measurements[SIZE_MEASUREMENT];
	srand(2);
	for (i = 0; i < 3; i++) {
		measurements[i] = (double)rand() / (double)RAND_MAX * 2.0 - 1.0;
	}
	printf("Random omega: [%f, %f, %f]\n", measurements[0], measurements[1], measurements[2]);

#if defined(FE_DIVBYZERO) && defined(FE_INVALID)
	feenableexcept( FE_DIVBYZERO | FE_INVALID);
#endif

	printf("INITIAL\n");
	matrix_quick_print(state, SIZE_STATE, 1);
	matrix_quick_print(covariance, SIZE_STATE, SIZE_STATE);
	for (i = 0; i < 100; i++) {
		printf("Time Elapsed: %fs\n", (i+1)*delta_t);

		// vdm_get_all(state, covariance, SIZE_STATE, ALPHA, BETA, KAPPA, chi, w_m, w_c);
		custom_scaled_points(state, covariance, chi, SIZE_STATE, ALPHA, KAPPA);
		vdm_scaled_weights(w_m, w_c, SIZE_STATE, ALPHA, BETA, KAPPA);

		ukf_predict(state, covariance, &process_model, &mean_state, &state_error, Q, delta_t, chi, gamma, w_m, w_c, new_state, new_covariance, SIZE_STATE);
		printf("PREDICT\n");
		matrix_quick_print(new_state, SIZE_STATE, 1);
		matrix_quick_print(new_covariance, SIZE_STATE, SIZE_STATE);
		
		ukf_update(new_state, measurements, new_covariance, &measurement, NULL, &state_error, NULL, R, gamma, w_m, w_c, state, covariance, SIZE_STATE, SIZE_MEASUREMENT);
		printf("UPDATE\n");
		matrix_quick_print(state, SIZE_STATE, 1);
		matrix_quick_print(covariance, SIZE_STATE, SIZE_STATE);
		// WARNING - hack
		//double *temp = alloca(SIZE_STATE*SIZE_STATE*sizeof(double));
		//scale_matrix(covariance, covariance, 0.5, SIZE_STATE, SIZE_STATE);
		//matrix_transpose(covariance, temp, SIZE_STATE, SIZE_STATE);
		//matrix_plus_matrix(covariance, temp, covariance, SIZE_STATE, SIZE_STATE, 1);
		//matrix_diagonal(temp, 0.01, SIZE_STATE);
		//matrix_plus_matrix(covariance, temp, covariance, SIZE_STATE, SIZE_STATE, 1);
		// hack over
	}

	return 0;
}
