#include <stdio.h>
#include <math.h>
#include <string.h>

#include "kalman.h"
#include "matrix_util.h"

#define SIZE_STATE 7
#define SIZE_MEASUREMENT 3

void process_model(double *curr_state, double *next_state, double delta_t, int n)
{
}

void measurement(double *state, double *measurement, int n, int m)
{
}

int main()
{
	double state[SIZE_STATE] = {0.0};
	double covariance[SIZE_STATE*SIZE_STATE] = {0.0};
	double new_state[SIZE_STATE];
	double new_covariance[SIZE_STATE*SIZE_STATE];
	double R[SIZE_MEASUREMENT*SIZE_MEASUREMENT];
	double Q[SIZE_STATE*SIZE_STATE];
	double chi[SIZE_STATE*(2*SIZE_STATE + 1)];
	double gamma[SIZE_STATE*(2*SIZE_STATE + 1)];
	double w_m[2*SIZE_STATE + 1];
	double w_c[2*SIZE_STATE + 1];
	double delta_t = 0.1;
	double measurements[5][SIZE_MEASUREMENT]; //TODO: initialize this array, possibly generated or from data file

	printf("INITIAL\n");
	matrix_quick_print(state, SIZE_STATE, 1);
	matrix_quick_print(covariance, SIZE_STATE, SIZE_STATE);
	int i;
	for (i = 0; i < 5; i++) {
		vdm_get_all(state, covariance, SIZE_STATE, 0.1, 2.0, -1.0, chi, w_m, w_c);

		ukf_predict(state, covariance, &process_model, Q, delta_t, chi, gamma, w_m, w_c, new_state, new_covariance, SIZE_STATE);
		printf("PREDICT\n");
		matrix_quick_print(new_state, SIZE_STATE, 1);
		matrix_quick_print(new_covariance, SIZE_STATE, SIZE_STATE);
		
		ukf_update(new_state, measurements[i], new_covariance, &measurement, R, gamma, w_m, w_c, state, covariance, SIZE_STATE, SIZE_MEASUREMENT);
		printf("UPDATE\n");
		matrix_quick_print(state, SIZE_STATE, 1);
		matrix_quick_print(covariance, SIZE_STATE, SIZE_STATE);
	}

	return 0;
}
