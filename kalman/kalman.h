#ifndef KALMAN_H
#define KALMAN_H

// Header file implementing the Kalman filter

typedef void (*Ukf_process_model)(double *chi, double *gamma, double delta_t, int n);
// chi and gamma are both (n)x(2n+1) matrices comprised of the points before and after the process model transform
typedef void (*Ukf_measurement_f)(double *gamma, double *zeta, int n, int m);
// gamma - (n)x(2n+1)
// zeta  - (m)x(2n+1)

void matrix_quick_print(double *matrix, int rows, int columns);
void predict(double *x, double *P, double *F, double *Q, double *x_f, double *P_f, int x_dim);
void update(double *x, double *z, double *P, double *H, double *R, double *x_f, double *P_f, int x_dim, int z_dim);

void vdm_get_all(double *x, double *P, int n, double a, double b, double k, double *chi, double *w_m, double *w_c);
void ukf_predict(double *x, double *P, Ukf_process_model f, double *Q, double delta_t, double *chi, double *weight_m, double *weight_c,  double *x_f, double *P_f, int n);

#endif
