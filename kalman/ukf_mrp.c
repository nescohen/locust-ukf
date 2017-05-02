#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <assert.h>

#include "kalman.h"
#include "matrix_util.h"
#include "quaternion_util.h"

#define SIZE_STATE 6
#define SIZE_MEASUREMENT 9
#define GYRO_VARIANCE 0.193825 // 11.111... degress in radians
#define POSITION_VARIANCE 0.1

#define ALPHA 0.001
#define BETA 2.0
#define KAPPA 0.0 // (double)(3 - SIZE_STATE)

#define POWER_ITERATIONS 100

double g_down[3] = {0, -1, 0};
double g_north[3] = {0, 0, 1};

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
	
	matrix_scale(numerator, mrp_f, 1/denominator, 3, 1);
}

void rotate_mrp(double orientation[3], double omega[3], double result[3], double dt)
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

void measurement(double *state, double *measurement, int n, int m)
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
		sum_mrp_angle[0] += cos(angle)*abs(weights[i]);
		sum_mrp_angle[1] += sin(angle)*abs(weights[i]);
		normalize_vector(mrp, mrp);
		matrix_scale(mrp, mrp, abs(weights[i]), 3, 1);
		matrix_plus_matrix(mrp, sum_mrp_vector, sum_mrp_vector, 3, 1, MATRIX_ADD);
	}
	double mean_mrp[3];
	double mean_angle = atan2(sum_mrp_angle[1], sum_mrp_angle[0]);
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
		double angle = vector_magnitude(point_rotation);
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
		double angle = -1*vector_magnitude(point_rotation);
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

double rand_gauss()
// returns a guassian distributed random number with mean 0 and standard deviation 1
// assumes srand() has already been seeded
{
	double u1 = (double)rand() / (double) RAND_MAX;
	double u2 = (double)rand() / (double) RAND_MAX;
	return sqrt(-2*log(u1))*cos(2*M_PI*u2);
}

void generate_measurements(double *z, double *omega, double *down_vect, double *north_vect, double dt)
{
	memcpy(z, down_vect, 3*sizeof(double));
	memcpy(z + 3, north_vect, 3*sizeof(double));
	memcpy(z + 6, omega, 3*sizeof(double));

	int i;
	for(i = 0; i < 6; i++) {
		z[i] += rand_gauss()*sqrt(POSITION_VARIANCE)*dt;
	}
	for(i = 6; i < 9; i++) {
		z[i] += rand_gauss()*sqrt(GYRO_VARIANCE)*dt;
	}
}

int main()
{
	int i;
	double state[SIZE_STATE] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
	double covariance[SIZE_STATE*SIZE_STATE];
	double new_state[SIZE_STATE];
	double new_covariance[SIZE_STATE*SIZE_STATE];
	matrix_diagonal(covariance, 1, SIZE_STATE);

	double R[SIZE_MEASUREMENT*SIZE_MEASUREMENT];
	double Q[SIZE_STATE*SIZE_STATE];
	matrix_init(R, 0, SIZE_MEASUREMENT, SIZE_MEASUREMENT);
	for (i = 0; i < 6; i++) {
		R[i + i*SIZE_MEASUREMENT] = POSITION_VARIANCE;
	}
	for (i = 6; i < 9; i++) {
		R[i + i*SIZE_MEASUREMENT] = GYRO_VARIANCE;
	}

	double chi[SIZE_STATE*(2*SIZE_STATE + 1)];
	double gamma[SIZE_STATE*(2*SIZE_STATE + 1)];
	double w_m[2*SIZE_STATE + 1];
	double w_c[2*SIZE_STATE + 1];
	
	double delta_t = 0.01;

	double measurements[SIZE_MEASUREMENT] = {0.0};
	double true_orientation[3] = {0.0, 0.0, 0.0};
	double true_omega[3] = {1.0, 0.0, 0.0};

	srand(2);// seed random number generator for rand_gauss()

	printf("INITIAL\n");
	matrix_quick_print(state, SIZE_STATE, 1);
	matrix_quick_print(covariance, SIZE_STATE, SIZE_STATE);
	for (i = 0; i < 30; i++) {
		printf("Time Elapsed: %fs\n", (i+1)*delta_t);

		// vdm_get_all(state, covariance, SIZE_STATE, ALPHA, BETA, KAPPA, chi, w_m, w_c);
		custom_scaled_points(state, covariance, chi, SIZE_STATE, ALPHA, KAPPA);
		vdm_scaled_weights(w_m, w_c, SIZE_STATE, ALPHA, BETA, KAPPA);

		// for test purposes
		double down[3];
		double north[3];
		double angle;
		double axis[3];
		double matrix[9];
		angle = delta_t*vector_magnitude(true_omega);
		normalize_vector(true_omega, axis);
		matrix_scale(axis, axis, tan(angle/4), 3, 1);
		compose_mrp(true_orientation, axis, true_orientation);
		angle = atan(vector_magnitude(true_orientation))*4;
		normalize_vector(true_orientation, axis);
		axis_angle_matrix(axis, angle, matrix);
		matrix_multiply(matrix, g_north, north, 3, 3, 1);
		matrix_multiply(matrix, g_down, down, 3, 3, 1);
		generate_measurements(measurements, true_omega, down, north, delta_t);
		printf("TRUE\n");
		printf("True rotation: %f radians\n", angle);
		matrix_quick_print(true_orientation, 3, 1);
		matrix_quick_print(true_omega, 3, 1);
		// end test purposes

		process_noise(Q, delta_t, 10);
		ukf_predict(state, covariance, &process_model, &mean_state, &state_error, Q, delta_t, chi, gamma, w_m, w_c, new_state, new_covariance, SIZE_STATE);
		printf("PREDICT\n");
		printf("Prediction rotation = %f radians\n", 4*atan(vector_magnitude(new_state)));
		matrix_quick_print(new_state, SIZE_STATE, 1);
		matrix_quick_print(new_covariance, SIZE_STATE, SIZE_STATE);

		printf("MEASUREMENT\n");
		matrix_quick_print(measurements, SIZE_MEASUREMENT, 1);
		
		ukf_update(new_state, measurements, new_covariance, &measurement, NULL, &state_error, NULL, &add_state, R, gamma, w_m, w_c, state, covariance, SIZE_STATE, SIZE_MEASUREMENT);
		printf("UPDATE\n");
		printf("Update rotation = %f radians\n", 4*atan(vector_magnitude(state)));
		matrix_quick_print(state, SIZE_STATE, 1);
		matrix_quick_print(covariance, SIZE_STATE, SIZE_STATE);

		// WARNING - hack
		double *temp = alloca(SIZE_STATE*SIZE_STATE*sizeof(double));
		matrix_scale(covariance, covariance, 0.5, SIZE_STATE, SIZE_STATE);
		matrix_transpose(covariance, temp, SIZE_STATE, SIZE_STATE);
		matrix_plus_matrix(covariance, temp, covariance, SIZE_STATE, SIZE_STATE, 1);
		matrix_diagonal(temp, 0.1, SIZE_STATE);
		matrix_plus_matrix(covariance, temp, covariance, SIZE_STATE, SIZE_STATE, 1);
		// hack over
	}

	return 0;
}
