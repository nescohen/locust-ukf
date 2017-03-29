#ifndef KALMAN_H
#define KALMAN_H

// Header file implementing the Kalman filter

void matrix_cross_vector(double *matrix, double *vector, double *result, int rows, int columns);
void matrix_cross_matrix(double *a, double *b, double *result, int m, int n, int p);
void matrix_plus_matrix(double *a, double *b, double *result, int rows, int columns, int sign);
void matrix_quick_print(double *matrix, int rows, int columns);
void matrix_transpose(double *matrix, double *result, int rows, int columns);
void matrix_inverse(double *matrix, double *result, int size);
void matrix_identity(double *matrix, int size);
void predict(double *x, double *P, double *F, double *Q, double *x_f, double *P_f, int x_dim);
void update(double *x, double *z, double *P, double *H, double *R, double *x_f, double *P_f, int x_dim, int z_dim);

#endif
