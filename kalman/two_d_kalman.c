// just a test file. not for production

#include <string.h>
#include "kalman.h"

void generate_transform(double delta_t, double *result)
{
	result[0] = 1;
	result[1] = delta_t;
	result[2] = 0;
	result[3] = 1;
}

int main()
{
	// TODO: seperate all functions required by the kalman filter to a seperate file and header

	double state[2] = {10, 1}; // x, 2-vector to represent current state with position(0) and velocity(1)
	double state_covariance[4] = {500, 0, 0, 10}; // P, state covariance, 2x2 matrix
	double process_noise[4] = {0, 0, 0, 0}; // Q, for now we have no process noise
	double m_function[2] = {1, 0}; // H, only measuring position
	double m_covariance[1] = {5}; // R
	double state_transition[4]; // F
	double prior[2];
	double posterior[2];
	double prior_covariance[4];
	double posterior_covariance[4];
	double measurements[10] = {10, 11, 12, 13, 14, 15, 16, 17, 18, 19};

	generate_transform(1, state_transition);

	int i;
	for (i = 0; i < 10; i++) {
		predict(state, state_covariance, state_transition, process_noise, prior, prior_covariance, 2);
		update(prior, &measurements[i], prior_covariance, m_function, m_covariance, posterior, posterior_covariance, 2, 1);
		memcpy(state, posterior, sizeof(state));
		memcpy(state_covariance, posterior_covariance, sizeof(state_covariance));
	}

	matrix_quick_print(posterior, 2, 1);
	matrix_quick_print(posterior_covariance, 2, 2);

	return 0;
}
