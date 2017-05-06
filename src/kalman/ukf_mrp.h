#ifndef UKF_MRP_H
#define UKF_MRP_H

#define SIZE_STATE 6
#define SIZE_MEASUREMENT 9

#define ALPHA 0.001
#define BETA 2.0
#define KAPPA 0.0 // (double)(3 - SIZE_STATE)

#define POWER_ITERATIONS 100

typedef struct ukf_parameters {
	double state[SIZE_STATE];
	double covariance[SIZE_STATE*SIZE_STATE];
	double R[SIZE_MEASUREMENT*SIZE_MEASUREMENT];
} Ukf_parameters;

void ukf_reverse_measure(double *state, double *measurement);
//converts a measurement to a state
// DO NOT USE! Not in working stat

void ukf_param_init(Ukf_parameters *parameters);
//performs very basic initialization and gauranntees that no uninitialized memory will slip through

void ukf_run(Ukf_parameters *parameters, double *measurement, double delta_t);

#endif
