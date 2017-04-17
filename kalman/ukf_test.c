#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <fenv.h>

#include "kalman.h"
#include "matrix_util.h"
#include "quaternion_util.h"

#define SIZE_STATE 7
#define SIZE_MEASUREMENT 3
#define SENSOR_VARIANCE (double)11 + (double)1 / (double)9

void process_model(double *curr_state, double *next_state, double delta_t, int n)
{
	double orientation[4];
	double rotation[4];
	double omega[3]; // omega's magnitude is angular velocity in radians per second
	double rotation_magnitude;
	double axis[3];

	memcpy(orientation, curr_state, sizeof(orientation));
	memcpy(omega, curr_state + 4, sizeof(omega)); 
	
	rotation_magnitude = vector_magnitude(omega);
	rotation_magnitude *= delta_t;
	normalize_vector(omega, axis);
	gen_quaternion(rotation_magnitude, axis, rotation);
	mult_quaternion(orientation, rotation, orientation);

	memcpy(next_state, orientation, sizeof(orientation));
	memcpy(next_state + 4, omega, sizeof(omega));
}

void measurement(double *state, double *measurement, int n, int m)
{
	measurement[0] = state[4];
	measurement[1] = state[5];
	measurement[2] = state[6];
}

int main()
{
	double state[SIZE_STATE] = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double covariance[SIZE_STATE*SIZE_STATE];
	double new_state[SIZE_STATE];
	double new_covariance[SIZE_STATE*SIZE_STATE];
	matrix_identity(covariance, SIZE_STATE);

	double R[SIZE_MEASUREMENT*SIZE_MEASUREMENT];
	double Q[SIZE_STATE*SIZE_STATE];
	matrix_diagonal(R, SENSOR_VARIANCE, SIZE_MEASUREMENT);
	matrix_diagonal(Q, 10.0, SIZE_STATE);

	double chi[SIZE_STATE*(2*SIZE_STATE + 1)];
	double gamma[SIZE_STATE*(2*SIZE_STATE + 1)];
	double w_m[2*SIZE_STATE + 1];
	double w_c[2*SIZE_STATE + 1];
	
	double delta_t = 0.1;

	double measurements[SIZE_MEASUREMENT];
	srand(time(NULL));
	int i;
	for (i = 0; i < 3; i++) {
		measurements[i] = (double)rand() / (double)RAND_MAX * 10 - 5.0;
	}
	printf("Random omega: [%f, %f, %f]\n", measurements[0], measurements[1], measurements[2]);

#if defined(DEBUG) && defined(FE_DIVBYZERO) && defined(FE_INVALID) && defined(FE_UNDERFLOW) && defined(FE_OVERFLOW)
	feenableexcept( FE_DIVBYZERO | FE_INVALID | FE_UNDERFLOW | FE_OVERFLOW );
#endif

	printf("INITIAL\n");
	matrix_quick_print(state, SIZE_STATE, 1);
	matrix_quick_print(covariance, SIZE_STATE, SIZE_STATE);
	for (i = 0; i < 10; i++) {
		vdm_get_all(state, covariance, SIZE_STATE, 0.1, 2.0, -4.0, chi, w_m, w_c);

		ukf_predict(state, covariance, &process_model, Q, delta_t, chi, gamma, w_m, w_c, new_state, new_covariance, SIZE_STATE);
		printf("PREDICT\n");
		matrix_quick_print(new_state, SIZE_STATE, 1);
		matrix_quick_print(new_covariance, SIZE_STATE, SIZE_STATE);
		
		ukf_update(new_state, measurements, new_covariance, &measurement, R, gamma, w_m, w_c, state, covariance, SIZE_STATE, SIZE_MEASUREMENT);
		printf("UPDATE\n");
		matrix_quick_print(state, SIZE_STATE, 1);
		matrix_quick_print(covariance, SIZE_STATE, SIZE_STATE);
	}

	return 0;
}
