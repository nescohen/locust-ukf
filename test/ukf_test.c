// Nes Cohen
// 07/07/2017
// Program used for analyzing the UKF

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <assert.h>

#include "src/kalman/kalman.h"
#include "src/kalman/ukf_mrp.h"
#include "src/math/matrix_util.h"
#include "src/math/quaternion_util.h"

#define GYRO_VARIANCE 0.193825 // 11.111... degress in radians
#define POSITION_VARIANCE 0.1

double g_down[3] = {0, -1, 0};
double g_north[3] = {0, 0, 1};

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
	Ukf_parameters parameters;
	ukf_param_init(&parameters);
	parameters.state[3] = 1.0; //double state[SIZE_STATE] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
	matrix_diagonal(parameters.covariance, 1, SIZE_STATE);

	matrix_init(parameters.R, 0, SIZE_MEASUREMENT, SIZE_MEASUREMENT);
	for (i = 0; i < 6; i++) {
		parameters.R[i + i*SIZE_MEASUREMENT] = POSITION_VARIANCE;
	}
	for (i = 6; i < 9; i++) {
		parameters.R[i + i*SIZE_MEASUREMENT] = GYRO_VARIANCE;
	}

	double delta_t = 0.01;
	double measurements[SIZE_MEASUREMENT] = {0.0};

	double true_omega = {1, 0, 0};

	srand(1);// seed random number generator for rand_gauss()

	printf("INITIAL\n");
	matrix_quick_print(parameters.state, SIZE_STATE, 1);
	matrix_quick_print(parameters.covariance, SIZE_STATE, SIZE_STATE);
	for (i = 0; i < 30; i++) {
		printf("Time Elapsed: %fs\n", (i+1)*delta_t);

		// Prepare simulated test data
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

		// Send data to ukf
		ukf_run(&parameters, measurements, delta_t);

		// Display results
		printf("MEASUREMENT\n");
		matrix_quick_print(measurements, SIZE_MEASUREMENT, 1);
		
		printf("FILTER\n");
		printf("Filter rotation = %f radians\n", 4*atan(vector_magnitude(parameters.state)));
		matrix_quick_print(parameters.state, SIZE_STATE, 1);
		matrix_quick_print(parameters.covariance, SIZE_STATE, SIZE_STATE);
	}

	return 0;
}
