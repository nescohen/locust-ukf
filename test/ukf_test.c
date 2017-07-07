// Nes Cohen 07/07/17
// test routine for mrp ukf

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <fenv.h>

#include "../src/kalman/kalman.h"
#include "../src/math/matrix_util.h"
#include "../src/math/quaternion_util.h"

#define SIZE_STATE 10
#define SIZE_MEASUREMENT 3
#define SENSOR_VARIANCE (double)11 + (double)1 / (double)9

#define ALPHA 0.1
#define BETA 2.0
#define KAPPA (double)(3 - SIZE_STATE)

#define POWER_ITERATIONS 100

void process_model(double *curr_state, double *next_state, double delta_t, int n)
{
	double orientation[4];
	double rotation[4];
	double omega[3]; // omega's magnitude is angular velocity in radians per second
	double alpha[3]; // angular acceleration vector radians per second per second
	double rotation_magnitude;
	double axis[3];

	memcpy(orientation, curr_state, sizeof(orientation));
	memcpy(omega, curr_state + 4, sizeof(omega)); 
	memcpy(alpha, curr_state + 7, sizeof(alpha));
	
	double d_omega[3];
	matrix_scale(alpha, d_omega, delta_t, 3, 1);
	matrix_plus_matrix(d_omega, omega, omega, 3, 3, 1);
	rotation_magnitude = vector_magnitude(omega);
	rotation_magnitude *= delta_t;
	normalize_vector(omega, axis);
	gen_quaternion(rotation_magnitude, axis, rotation);
	// normalize_quaternion(orientation, orientation);
	mult_quaternion(orientation, rotation, orientation);

	memcpy(next_state, orientation, sizeof(orientation));
	memcpy(next_state + 4, omega, sizeof(omega));
	memcpy(next_state + 7, alpha, sizeof(alpha));
}

void measurement(double *state, double *measurement, int n, int m)
{
	measurement[0] = state[4];
	measurement[1] = state[5];
	measurement[2] = state[6];
}

void mean_state(double *points, double *weights, double *mean, int size, int count)
{
	double *Q = alloca(4*count*sizeof(double));
	double *Q_t = alloca(4*count*sizeof(double));
	double *M = alloca(4*4*sizeof(double));
	int i;
	for (i = 0; i < count; i++) {
		matrix_scale(points + i*size, Q + i*4, weights[i], 4, 1);
	}
	matrix_transpose(Q, Q_t, 4, count);
	matrix_multiply(Q, Q_t, M, 4, count, 4);

	// find the eigenvector with the greatest eigenvalue using the power iteration method
	// in most cases this is equivilent to finding the mean quaternion
	double b[4] = {1, 0, 0, 0};
	for (i = 0; i < POWER_ITERATIONS; i++) {
		matrix_multiply(Q, b, b, 4, 4, 1);
		normalize_quaternion(b, b);
	}
	memcpy(mean, b, sizeof(b));

	double avg[6] = {0, 0, 0};
	for (i = 0; i < count; i++) {
		double temp[6];
		matrix_scale(points + 4 + i*size, temp, weights[i], 6, 1);
		matrix_plus_matrix(temp, avg, avg, 6, 1, 1);
	}
	memcpy(mean + 4, avg, sizeof(avg));
}

void state_difference(double *point, double *mean, double *difference, int n)
{
	double point_quat[4];
	double mean_quat[4];
	double point_vector[6];
	double mean_vector[6];
	memcpy(point_quat, point, sizeof(point_quat));
	memcpy(mean_quat, mean, sizeof(mean_quat));
	memcpy(point_vector, point + 4, sizeof(point_vector));
	memcpy(mean_vector, mean + 4, sizeof(mean_vector));

	// Tradition difference for angular velocity and acceleration
	matrix_plus_matrix(point_vector, mean_vector, difference + 4, n, 1, 0);

	// Quaternion between mean and point quaternions
	int i;
	for (i = 1; i < 4; i++) {
		point_quat[i] *= -1;
	}
	mult_quaternion(mean_quat, point_quat, difference);
}

void custom_scaled_points(double *x, double *P, double *chi, int n, double a, double k)
// one possible solution to the quaternion problem
{
	int i, j;
	memcpy(chi, x, n*sizeof(double)); // set the first sigma point (chi sub 0) to the mean

	double scale = a*a*(n + k);
	double vector[3];
	double quat[4];
	for (i = 0; i < 4; i++) {
		quat[i] = P[i*n + i];
	}
	decomp_quaternion(quat, vector);
	double *temp = alloca((n-1)*(n-1)*sizeof(double));
	memcpy(temp, P + n + 1, (n-1)*(n-1)*sizeof(double));
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			if (i == j) temp[j + i*n] = vector[i];
			else temp[j + i*n] = 0.0;
		}
	}
	matrix_scale(temp, temp, scale, n-1 , n-1);
	matrix_sqrt(temp, temp, n-1);

	for (i = 1; i <= n; i++) {
		for (j = 4; j < n; j++) {
			chi[i*n + j] = x[j] + temp[(i-1) + j*n];
		}
		double angle;
		double axis[3];
		matrix_column(temp, axis, 3, n, i-1);
		angle = vector_magnitude(axis);
		normalize_vector(axis, axis);
		gen_quaternion(angle, axis, chi + i*n);
	}
	for (i = n+1; i <= 2*n; i++) {
		for (j = 4; j < n; j++) {
			chi[i*n + j] = x[j] - temp[(i-n-1) + j*n];
		}
		double angle;
		double axis[3];
		matrix_column(temp, axis, 3, n, i-1);
		angle = -1*vector_magnitude(axis);
		normalize_vector(axis, axis);
		gen_quaternion(angle, axis, chi + i*n);
	}
}

int main()
{
	int i;
	double state[SIZE_STATE] = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double covariance[SIZE_STATE*SIZE_STATE];
	double new_state[SIZE_STATE];
	double new_covariance[SIZE_STATE*SIZE_STATE];
	matrix_diagonal(covariance, 0.001, SIZE_STATE);
	for (i = 4; i < SIZE_STATE; i++) {
		covariance[i*SIZE_STATE + i] = 10.0;
	}

	double R[SIZE_MEASUREMENT*SIZE_MEASUREMENT];
	double Q[SIZE_STATE*SIZE_STATE];
	matrix_diagonal(R, SENSOR_VARIANCE, SIZE_MEASUREMENT);
	memset(Q, 0.0, SIZE_STATE*SIZE_STATE*sizeof(double));
	for (i = 4; i < SIZE_STATE; i++) {
		Q[i*SIZE_STATE + i] = 10.0;
	}

	double chi[SIZE_STATE*(2*SIZE_STATE + 1)];
	double gamma[SIZE_STATE*(2*SIZE_STATE + 1)];
	double w_m[2*SIZE_STATE + 1];
	double w_c[2*SIZE_STATE + 1];
	
	double delta_t = 0.1;

	double measurements[SIZE_MEASUREMENT];
	srand(1);
	for (i = 0; i < 3; i++) {
		measurements[i] = (double)rand() / (double)RAND_MAX * 10 - 5.0;
	}
	printf("Random omega: [%f, %f, %f]\n", measurements[0], measurements[1], measurements[2]);

#if defined(FE_DIVBYZERO) && defined(FE_INVALID) && defined(FE_UNDERFLOW) && defined(FE_OVERFLOW)
	feenableexcept( FE_DIVBYZERO | FE_INVALID | FE_UNDERFLOW | FE_OVERFLOW );
#endif

	printf("INITIAL\n");
	matrix_quick_print(state, SIZE_STATE, 1);
	matrix_quick_print(covariance, SIZE_STATE, SIZE_STATE);
	for (i = 0; i < 10; i++) {
		//vdm_get_all(state, covariance, SIZE_STATE, ALPHA, BETA, KAPPA, chi, w_m, w_c);
		custom_scaled_points(state, covariance, chi, SIZE_STATE, ALPHA, KAPPA);
		vdm_scaled_weights(w_m, w_c, SIZE_STATE, ALPHA, BETA, KAPPA);

		ukf_predict(state, covariance, &process_model, &mean_state, NULL, Q, delta_t, chi, gamma, w_m, w_c, new_state, new_covariance, SIZE_STATE);
		printf("PREDICT\n");
		matrix_quick_print(new_state, SIZE_STATE, 1);
		matrix_quick_print(new_covariance, SIZE_STATE, SIZE_STATE);
		
		ukf_update(new_state, measurements, new_covariance, &measurement, NULL, NULL, NULL, R, gamma, w_m, w_c, state, covariance, SIZE_STATE, SIZE_MEASUREMENT);
		printf("UPDATE\n");
		matrix_quick_print(state, SIZE_STATE, 1);
		matrix_quick_print(covariance, SIZE_STATE, SIZE_STATE);
		// WARNING - hack
		//double *temp = alloca(SIZE_STATE*SIZE_STATE*sizeof(double));
		//matrix_scale(covariance, covariance, 0.5, SIZE_STATE, SIZE_STATE);
		//matrix_transpose(covariance, temp, SIZE_STATE, SIZE_STATE);
		//matrix_plus_matrix(covariance, temp, covariance, SIZE_STATE, SIZE_STATE, 1);
		//matrix_diagonal(temp, 0.01, SIZE_STATE);
		//matrix_plus_matrix(covariance, temp, covariance, SIZE_STATE, SIZE_STATE, 1);
		// hack over
	}

	return 0;
}
