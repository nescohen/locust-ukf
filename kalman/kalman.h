#ifndef KALMAN_H
#define KALMAN_H

// Header file implementing the Kalman filter

typedef void (*Ukf_process_model)(double *curr_state, double *next_state, double delta_t, int n);
// curr_state and next_state are n x 1 matrices
typedef void (*Ukf_measurement_f)(double *state, double *measurement, int n, int m);
typedef void (*Ukf_state_mean)(double *points, double *weights, double *mean, int size, int count);
typedef void (*Ukf_measurements_mean)(double *points, double *weights, double *mean, int size, int count);
typedef void (*Ukf_difference_state)(double *point, double *mean, double *difference, int n);
typedef void (*Ukf_difference_measure)(double *point, double *mean, double *difference, int m);
typedef void (*Ukf_add_state)(double *state, double *change, double *result, int n);

void predict(double *x, double *P, double *F, double *Q, double *x_f, double *P_f, int x_dim);
void update(double *x, double *z, double *P, double *H, double *R, double *x_f, double *P_f, int x_dim, int z_dim);

void vdm_scaled_points(double *x, double *P, double *chi, int n, double a, double k);
void vdm_scaled_weights(double *w_m, double *w_c, int n, double a, double b, double k);
void vdm_get_all(double *x, double *P, int n, double a, double b, double k, double *chi, double *w_m, double *w_c);
// chi - (2n+1)x(n)
// w_m - (2n+1)x(1)
// w_c - (2n+1)x(1)

void ukf_predict
(double *x, double *P, Ukf_process_model f, Ukf_state_mean mean, Ukf_difference_state diff, double *Q, double delta_t, double *chi, double *gamma, double *weight_m, double *weight_c,  double *x_f, double *P_f, int n);

void ukf_update
(double *x, double *z, double *P, Ukf_measurement_f h, Ukf_measurements_mean mean, Ukf_difference_state xdiff, Ukf_difference_measure zdiff, Ukf_add_state add_state, double *R, double *gamma, double *weight_m, double *weight_c, double *x_f, double *P_f, int n, int m);

#endif
