// Specific Ukf implementation for attitude filtering using Modified Rodriguez Parameters

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <assert.h>

#include "ukf_mrp.h"
#include "kalman.h"
#include "../math/matrix_util.h"
#include "../math/quaternion_util.h"

extern double g_north[3];
extern double g_down[3];

void normalize_mrp_angle(double *mrp, double *result)
{
	double result_temp[3];

	double angle = 4*atan(vector_magnitude(mrp));
	angle = convert_angle(angle);
	normalize_vector(mrp, result_temp);
	matrix_scale(result_temp, result_temp, tan(angle/4), 3, 1);
	memcpy(result, result_temp, sizeof(result_temp));
}

void compose_mrp(double mrp_a[3], double mrp_b[3], double mrp_f[3])
// equation from Journal of Astronautical Sciences paper "A Survey of Attitude Representations"
// P" = ((1 - |P|^2)P' + (1 - |P'|^2)P - 2P'xP) / (1 + |P'|^2 * |P|^2 - 2P'*P)
{
	double mag2_a = pow(vector_magnitude(mrp_a), 2);
	double mag2_b = pow(vector_magnitude(mrp_b), 2);
	double denominator;
	denominator = 1;
	denominator += mag2_b * mag2_a;
	denominator -= 2 * dot_product(mrp_b, mrp_a);

	double numerator[3];
	double temp[3];
	matrix_scale(mrp_b, numerator, 1 - mag2_a, 3, 1);
	matrix_scale(mrp_a, temp, 1 - mag2_b, 3, 1);
	matrix_plus_matrix(temp, numerator, numerator, 3, 1, MATRIX_ADD);
	matrix_scale(mrp_b, temp, 2, 3, 1);
	cross_product(temp, mrp_a, temp);
	matrix_plus_matrix(numerator, temp, numerator, 3, 1, MATRIX_SUBTRACT);
	
	if (denominator != 0) {
		matrix_scale(numerator, mrp_f, 1/denominator, 3, 1);
	}
	else {
		matrix_scale(numerator, mrp_f, 0, 3, 1);
	}
	normalize_mrp_angle(mrp_f, mrp_f);
}

void rotate_mrp(double orientation[3], double omega[3], double result[3], double dt)
// DOES NOT WORK! DO NOT USE!
{
	double temp[3];
	double result_copy[3];
	// equation from NASA paper "Attitude Estimation Using Modified Rodrigues Parameters"
	// 1/4{(1 - |P|^2)w - 2w x P + 2(w*P)P

	// (1 - |P|^2)w
	double factor = 1 - pow(vector_magnitude(orientation), 2);
	matrix_scale(omega, result_copy, factor, 3, 1);

	// -2w x P
	matrix_scale(omega, temp, -2, 3, 1);
	cross_product(temp, orientation, temp);
	matrix_plus_matrix(temp, result_copy, result_copy, 3, 1, MATRIX_ADD);

	// 2(w*P)P
	factor = dot_product(omega, orientation);
	matrix_scale(orientation, temp, factor, 3, 1);
	matrix_plus_matrix(temp, result_copy, result_copy, 3, 1, MATRIX_ADD);

	// 1/4{ .. }
	matrix_scale(result_copy, result_copy, 0.25*dt, 3, 1);
	matrix_plus_matrix(orientation, result_copy, result_copy, 3, 1, MATRIX_ADD);
	memcpy(result, result_copy, sizeof(result_copy));
}

void process_model(double *curr_state, double *next_state, double delta_t, int n)
{
	assert(n == SIZE_STATE);

	double mrp[3];
	double omega[3];

	double result_mrp[3];

	memcpy(mrp, curr_state, sizeof(mrp));
	memcpy(omega, curr_state + 3, sizeof(omega));

	double rot_total[3];
	double rot_angle = delta_t*vector_magnitude(omega);
	normalize_vector(omega, rot_total);
	matrix_scale(rot_total, rot_total, tan(rot_angle/4), 3, 1);
	compose_mrp(mrp, rot_total, result_mrp);

	memcpy(next_state, result_mrp, sizeof(result_mrp));
	memcpy(next_state + 3, omega, sizeof(omega));
}

void measurement_function(double *state, double *measurement, int n, int m)
{
	measurement[6] = state[3];
	measurement[7] = state[4];
	measurement[8] = state[5];

	double angle;
	double axis[3];
	double matrix[9];
	angle = atan(vector_magnitude(state))*4;
	normalize_vector(state, axis);
	axis_angle_matrix(axis, angle, matrix);
	matrix_multiply(matrix, g_down, measurement, 3, 3, 1);
	matrix_multiply(matrix, g_north, measurement + 3, 3, 3, 1);
}

void mean_state(double *points, double *weights, double *mean, int size, int count)
// compute weighted mean of state points, specifically the modified rodrigues parameter component
{
	double sum_mrp_angle[2] = {0.0}; // format: { cos(a), sin(a) }
	double sum_mrp_vector[3] = {0.0};
	double sum_state[3] = {0.0};
	int i;
	for (i = 0; i < count; i++) {
		double mrp[3];
		double angle;
		double rest_state[3];
		memcpy(rest_state, points + i*size + 3, sizeof(rest_state));
		matrix_scale(rest_state, rest_state, weights[i], 3, 1);
		matrix_plus_matrix(rest_state, sum_state, sum_state, 3, 1, MATRIX_ADD);

		memcpy(mrp, points + i*size, sizeof(mrp));
		angle = vector_magnitude(mrp);
		angle = atan(angle)*4;
		sum_mrp_angle[0] += cos(angle)*fabs(weights[i]);
		sum_mrp_angle[1] += sin(angle)*fabs(weights[i]);
		normalize_vector(mrp, mrp);
		matrix_scale(mrp, mrp, fabs(weights[i]), 3, 1);
		matrix_plus_matrix(mrp, sum_mrp_vector, sum_mrp_vector, 3, 1, MATRIX_ADD);
	}
	double mean_mrp[3];
	double mean_angle = atan2(sum_mrp_angle[1], sum_mrp_angle[0]);
	if (mean_angle < 0) mean_angle += 2*M_PI;
	normalize_vector(sum_mrp_vector, mean_mrp);
	matrix_scale(mean_mrp, mean_mrp, tan(mean_angle/4), 3, 1);

	memcpy(mean, mean_mrp, sizeof(mean_mrp));
	memcpy(mean + 3, sum_state, sizeof(sum_state));
}

void state_error(double *point, double *mean, double *error, int n)
{
	assert(n == SIZE_STATE);
	// preform naive difference for angular velocity and acceleration
	matrix_plus_matrix(point + 3, mean + 3, error + 3, 3, 1, MATRIX_SUBTRACT);

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
	matrix_scale(axis, error, tan(angle/4), 3, 1);
	normalize_mrp_angle(error, error);
}

void add_state(double *state, double *change, double *result, int n)
{
	double final_state[SIZE_STATE];

	matrix_plus_matrix(state + 3, change + 3, final_state + 3, 3, 1, MATRIX_ADD);
	compose_mrp(state, change, final_state);
	memcpy(result, final_state, sizeof(final_state));
}

void custom_scaled_points(double *x, double *P, double *chi, int n, double a, double k)
// modified scaled points algorithm for use with MRPs
{
	int i, j;
	memcpy(chi, x, n*sizeof(double)); // set the first sigma point (chi sub 0) to the mean

	double scale = a*a*(n + k);
	double *temp = alloca(n * n * sizeof(double));
	matrix_scale(P, temp, scale, n, n);
	matrix_sqrt(temp, temp, n);

	for (i = 1; i <= n; i++) {
		double point_rotation[3];
		for (j = 0; j < 3; j++) {
			point_rotation[j] = temp[(i-1) + j*n];
		}
		double angle = convert_angle(vector_magnitude(point_rotation));
		normalize_vector(point_rotation, point_rotation);
		matrix_scale(point_rotation, point_rotation, tan(angle/4), 3, 1);
		compose_mrp(x, point_rotation, chi + i*n);
		for (j = 3; j < n; j++) {
			chi[i*n + j] = x[j] + temp[(i-1) + j*n];
		}
	}
	for (i = n+1; i <= 2*n; i++) {
		double point_rotation[3];
		for (j = 0; j < 3; j++) {
			point_rotation[j] = temp[(i-n-1) + j*n];
		}
		double angle = convert_angle(-1*vector_magnitude(point_rotation));
		normalize_vector(point_rotation, point_rotation);
		matrix_scale(point_rotation, point_rotation, tan(angle/4), 3, 1);
		compose_mrp(x, point_rotation, chi + i*n);
		for (j = 3; j < n; j++) {
			chi[i*n + j] = x[j] - temp[(i-n-1) + j*n];
		}
	}
}

void process_noise(double *Q, double dt, double scale)
// calculates the process noise for a given time increment
{
	int i;
	memset(Q, 0, SIZE_STATE*SIZE_STATE*sizeof(double));
	for (i = 0; i < 3; i++) {
		Q[i + i*SIZE_STATE] = pow(dt, 4) / 4;
		Q[i+3 + (i+3)*SIZE_STATE] = pow(dt, 2);
		Q[i+3 + i*SIZE_STATE] = pow(dt, 3) / 2;
		Q[i + (i+3)*SIZE_STATE] = pow(dt, 3) / 2;
	}
	matrix_scale(Q, Q, scale, SIZE_STATE, SIZE_STATE);
}

void triad_mrp(double *result_mrp, double *r1, double *r2, double *R1, double *R2)
// Algorithm from "Triad Method" wikipedia article
{
	double R1xR2[3];
	double r1xr2[3];
	double S[3];
	double s[3];
	double M[3];
	double m[3];
	double SxM[3];
	double sxm[3];
	double rot_matrix[9];
	double temp[9];

	cross_product(R1, R2, R1xR2);
	cross_product(r1, r2, r1xr2);
	normalize_vector(R1, S);
	normalize_vector(r1, s);
	normalize_vector(R1xR2, M);
	normalize_vector(r1xr2, m);

	matrix_init_column(rot_matrix, 3, 3, S, M, SxM);
	matrix_init_column(temp, 3, 3, s, m, sxm);
	matrix_transpose(temp, temp, 3, 3);
	matrix_multiply(rot_matrix, temp, rot_matrix, 3, 3, 3);

	double axis[3];
	double angle;
	matrix_to_euler(rot_matrix, axis, &angle);
	matrix_scale(axis, result_mrp, tan(angle/4), 3, 1);
}

void ukf_reverse_measure(double *state, double *measurement)
//converts a measurement to a state
// DO NOT USE! Not in working stat
{
	triad_mrp(state, measurement, measurement + 3, g_down, g_north);
	memcpy(state + 3, measurement + 6, 3*sizeof(double));
}

void ukf_param_init(Ukf_parameters *parameters)
//performs very basic initialization and gauranntees that no uninitialized memory will slip through
{
	memset(parameters->state, 0, sizeof(parameters->state));
	memset(parameters->R, 0, sizeof(parameters->R));

	matrix_identity(parameters->covariance, SIZE_STATE);
}

void ukf_run(Ukf_parameters *parameters, double *measurement, double delta_t)
{
	static int initialized = 0;
	static Ukf_options options;

	double Q[SIZE_STATE*SIZE_STATE];
	double chi[SIZE_STATE*(2*SIZE_STATE + 1)];
	double gamma[SIZE_STATE*(2*SIZE_STATE + 1)];
	double w_m[2*SIZE_STATE + 1];
	double w_c[2*SIZE_STATE + 1];

	if (!initialized) {
		// initialize and configure additional options for the ukf
		ukf_init_options(&options, SIZE_STATE, SIZE_MEASUREMENT);
		options.f = &process_model;
		options.h = &measurement_function;
		options.state_mean = &mean_state;
		options.state_diff = &state_error;
		options.state_add = &add_state;
		// printf("INITIAL\n");
		// matrix_quick_print(parameters->state, SIZE_STATE, 1);
		// matrix_quick_print(parameters->covariance, SIZE_STATE, SIZE_STATE);
	}

	custom_scaled_points(parameters->state, parameters->covariance, chi, SIZE_STATE, ALPHA, KAPPA);
	vdm_scaled_weights(w_m, w_c, SIZE_STATE, ALPHA, BETA, KAPPA);

	process_noise(Q, delta_t, 10);
	ukf_predict(parameters->state, parameters->covariance, Q, delta_t, chi, gamma, w_m, w_c, parameters->state, parameters->covariance, &options);

	// printf("PREDICT\n");
	// printf("Prediction rotation = %f radians\n", 4*atan(vector_magnitude(parameters->state)));
	// matrix_quick_print(parameters->state, SIZE_STATE, 1);
	// matrix_quick_print(parameters->covariance, SIZE_STATE, SIZE_STATE);

	// printf("MEASUREMENT\n");
	// matrix_quick_print(measurement, SIZE_MEASUREMENT, 1);
	
	ukf_update(parameters->state, measurement, parameters->covariance, parameters->R, gamma, w_m, w_c, parameters->state, parameters->covariance, &options);

	// printf("UPDATE\n");
	// printf("Update rotation = %f radians\n", 4*atan(vector_magnitude(parameters->state)));
	// matrix_quick_print(parameters->state, SIZE_STATE, 1);
	// matrix_quick_print(parameters->covariance, SIZE_STATE, SIZE_STATE);

	// WARNING - hack
	double *temp = alloca(SIZE_STATE*SIZE_STATE*sizeof(double));
	matrix_scale(parameters->covariance, parameters->covariance, 0.5, SIZE_STATE, SIZE_STATE);
	matrix_transpose(parameters->covariance, temp, SIZE_STATE, SIZE_STATE);
	matrix_plus_matrix(parameters->covariance, temp, parameters->covariance, SIZE_STATE, SIZE_STATE, 1);
	matrix_diagonal(temp, 0.1, SIZE_STATE);
	matrix_plus_matrix(parameters->covariance, temp, parameters->covariance, SIZE_STATE, SIZE_STATE, 1);
	// hack over
}
