#ifndef KALMAN_H
#define KALMAN_H

// Header file implementing the Kalman filter

void matrix_quick_print(double *matrix, int rows, int columns);
void predict(double *x, double *P, double *F, double *Q, double *x_f, double *P_f, int x_dim);
void update(double *x, double *z, double *P, double *H, double *R, double *x_f, double *P_f, int x_dim, int z_dim);

#endif
