// Function definitions for quaternions and 3-vectors

#include <math.h>
#include "quaternion_util.h"

void gen_quaternion(double theta, double vect[3], double result[4])
// Generates the equivilent quaternion from a euler axis angle representation. All angles in radians
{
	double sin_theta = sin(theta / 2.0);
	result[0] = cos(theta / 2.0);
	result[1] = sin_theta*vect[0];
	result[2] = sin_theta*vect[1];
	result[3] = sin_theta*vect[2];
}

void mult_quaternion(double op_a[4], double op_b[4], double result[4])
{
	result[0] = op_a[0]*op_b[0] - op_a[1]*op_b[1] - op_a[2]*op_b[2] - op_a[3]*op_b[3];
	result[1] = op_a[1]*op_b[0] + op_a[0]*op_b[1] - op_a[3]*op_b[2] + op_a[2]*op_b[3];
	result[2] = op_a[2]*op_b[0] + op_a[3]*op_b[1] + op_a[0]*op_b[2] - op_a[1]*op_b[3];
	result[3] = op_a[3]*op_b[0] - op_a[2]*op_b[1] + op_a[1]*op_b[2] + op_a[0]*op_b[3];
}

void generate_matrix(double q[4], double matrix[9])
// Converts quaternion q into equivalent post multiplied rotation matrix
{
	matrix[0] = 1 - 2*q[2]*q[2] - 2*q[3]*q[3];
	matrix[1] = 2*q[1]*q[2] - 2*q[3]*q[0];
	matrix[2] = 2*q[1]*q[3] + 2*q[2]*q[0];
	matrix[3] = 2*q[1]*q[2] + 2*q[3]*q[0];
	matrix[4] = 1 - 2*q[1]*q[1] - 2*q[3]*q[3];
	matrix[5] = 2*q[2]*q[3] - 2*q[1]*q[0];
	matrix[6] = 2*q[1]*q[3] - 2*q[2]*q[0];
	matrix[7] = 2*q[2]*q[3] + 2*q[1]*q[0];
	matrix[8] = 1 - 2*q[1]*q[1] - 2*q[2]*q[2];
}

void vector_by_matrix(double v[3], double r[9], double result[3])
// Multiplies vector v by matrix r
{
	int i, j;
	for (i = 0; i < 3; i++){
		result[i] = 0;
		for (j = 0; j < 3; j++){
			result[i] += v[j]*r[i*3 + j];
		}
	}
}

void vector_by_scalar(double v[3], double scalar, double result[3])
{
	int i;
	for (i = 0; i < 3; i++) {
		result[i] = v[i]*scalar;
	}
}

void add_vectors(double v_a[3], double v_b[3], double result[3])
{
	int i;
	for (i = 0; i < 3; i++) {
		result[i] = v_a[i] + v_b[i];
	}
}

