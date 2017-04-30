#include <stdio.h>
#include <math.h>
#include <string.h>
#include "quaternion_util.h"
#include "matrix_util.h"

void compose_mrp(double mrp_a[3], double mrp_b[3], double mrp_f[3])
// equation from Journal of Astronautical Sciences paper "A Survey of Attitude Representations"
// P" = ((1 - |P|^2)P' + (1 - |P'|^2)P - 2P'xP) / (1 + |P'|^2 * |P|^2 - 2P'*P)
{
	double mag2_a = pow(vector_magnitude(mrp_a), 2);
	double mag2_b = pow(vector_magnitude(mrp_b), 2);
	double denominator;
	denominator = 1;
	denominator += mag2_b * mag2_a;
	denominator -= 2 * dot_product(mrp_b, mrp_a);

	double numerator[3];
	double temp[3];
	scale_matrix(mrp_b, numerator, 1 - mag2_a, 3, 1);
	scale_matrix(mrp_a, temp, 1 - mag2_b, 3, 1);
	matrix_plus_matrix(temp, numerator, numerator, 3, 1, MATRIX_ADD);
	scale_matrix(mrp_b, temp, 2, 3, 1);
	cross_product(temp, mrp_a, temp);
	matrix_plus_matrix(numerator, temp, numerator, 3, 1, MATRIX_SUBTRACT);
	
	scale_matrix(numerator, mrp_f, 1/denominator, 3, 1);
}

void rotate_mrp(double orientation[3], double omega[3], double result[3], double dt)
{
	double temp[3];
	double result_copy[3];
	// equation from NASA paper "Attitude Estimation Using Modified Rodrigues Parameters"
	// 1/4{(1 - |P|^2)w - 2w x P + 2(w*P)P

	// (1 - |P|^2)w
	double factor = 1 - pow(vector_magnitude(orientation), 2);
	scale_matrix(omega, result_copy, factor, 3, 1);

	// -2w x P
	scale_matrix(omega, temp, -2, 3, 1);
	cross_product(temp, orientation, temp);
	matrix_plus_matrix(temp, result_copy, result_copy, 3, 1, MATRIX_ADD);

	// 2(w*P)P
	factor = dot_product(omega, orientation);
	scale_matrix(orientation, temp, factor, 3, 1);
	matrix_plus_matrix(temp, result_copy, result_copy, 3, 1, MATRIX_ADD);

	// 1/4{ .. }
	scale_matrix(result_copy, result_copy, 0.25*dt, 3, 1);
	matrix_plus_matrix(orientation, result_copy, result_copy, 3, 1, MATRIX_ADD);
	memcpy(result, result_copy, sizeof(result_copy));
}

int main()
{
	double angle = 4;
	double axis[3] = {1, 0, 0};
	double start[3] = {0, 0, 0};
	double temp[3];
	double rotate[3];
	double compose[3];

	scale_matrix(axis, temp, angle, 3, 1);
	rotate_mrp(start, temp, rotate, 1);
	printf("Rotate rotation = %f rad\n", atan(vector_magnitude(rotate))*4);

	scale_matrix(axis, temp, tan(angle/4), 3, 1);
	compose_mrp(start, temp, compose);
	printf("Compose rotation = %f rad\n", atan(vector_magnitude(compose))*4);
}
