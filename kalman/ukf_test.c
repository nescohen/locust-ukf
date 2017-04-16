#include <stdio.h>
#include <math.h>
#include <string.h>
#include "kalman.h"

void process_model(double *curr_state, double *next_state, double delta_t, int n)
{
	next_state[0] = curr_state[0] + curr_state[1]*delta_t;
	next_state[1] = curr_state[1];
	next_state[2] = curr_state[2] + curr_state[3]*delta_t;
	next_state[3] = curr_state[3];
}

void measurement(double *state, double *measurement, int n, int m)
{
	measurement[0] = state[0];
	measurement[1] = state[2];
}

void generate_process_noise(double *Q, double delta_t)
{
	memset(Q, 0, 16*sizeof(double));
	Q[0] = pow(delta_t, 4) / 4 * 0.09;
	Q[1] = pow(delta_t, 3) / 2 * 0.09;
	Q[4] = Q[1];
	Q[5] = pow(delta_t, 2) * 0.09;
	Q[10] = pow(delta_t, 4) / 4 * 0.09;
	Q[11] = pow(delta_t, 3) / 2 * 0.09;
	Q[14] = Q[11];
	Q[15] = pow(delta_t, 2) * 0.09;
}

int main()
{
	double state[4] = {0.0, 0.0, 0.0, 0.0};
	double covariance[16] = {0.0};
	double new_state[4];
	double new_covariance[16];
	double R[4] = {0.09, 0.0, 0.0, 0.09};
	double Q[16];
	double chi[4*(2*4 + 1)];
	double gamma[4*(2*4 + 1)];
	double w_m[9];
	double w_c[9];
	double delta_t = 1;
	double measurements[5][2] = { {1.0, -1.0}, {1.9, -2.1}, {3.05, -2.95}, {4.0, -4.0}, {5.0, -5.0} };

	int i;
	// initialize covariance matrix
	for (i = 0; i < 4; i++) {
		covariance[i + i*4] = 1.0;
	}
	printf("INITIAL\n");
	matrix_quick_print(state, 4, 1);
	matrix_quick_print(covariance, 4, 4);
	for (i = 0; i < 5; i++) {
		vdm_get_all(state, covariance, 4, 0.1, 2.0, -1.0, chi, w_m, w_c);
		generate_process_noise(Q, delta_t);

		ukf_predict(state, covariance, &process_model, Q, delta_t, chi, gamma, w_m, w_c, new_state, new_covariance, 4);
		printf("PREDICT\n");
		matrix_quick_print(new_state, 4, 1);
		matrix_quick_print(new_covariance, 4, 4);
		
		ukf_update(new_state, measurements[i], new_covariance, &measurement, R, gamma, w_m, w_c, state, covariance, 4, 2);
		printf("UPDATE\n");
		matrix_quick_print(state, 4, 1);
		matrix_quick_print(covariance, 4, 4);
	}

	return 0;
}
