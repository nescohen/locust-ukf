// Function definitions for quaternions and 3-vectors

#include <math.h>
#include <string.h>
#include "quaternion_util.h"

void gen_quaternion(double theta, double vect[3], double result[4])
// Generates the equivilent quaternion from a euler axis angle representation. All angles in radians
{
	if (theta != 0) {
		double sin_theta = sin(theta / 2.0);
		result[0] = cos(theta / 2.0);
		result[1] = sin_theta*vect[0];
		result[2] = sin_theta*vect[1];
		result[3] = sin_theta*vect[2];
	}
	else {
		result[0] = 1;
		result[1] = 0;
		result[2] = 0;
		result[3] = 0;
	}
}

void decomp_quaternion(double quat[4], double vect[3])
{
	double value = sqrt(1 - quat[0]*quat[0]);
	int i;
	for (i = 0; i < 3; i++) {
		vect[i] = quat[i + 1] / value;
	}
	vector_by_scalar(vect, 2*acos(quat[0]), vect);
}

void mult_quaternion(double op_a[4], double op_b[4], double result[4])
{
	double a_copy[4];
	double b_copy[4];
	memcpy(a_copy, op_a, sizeof(a_copy));
	memcpy(b_copy, op_b, sizeof(b_copy));

	result[0] = a_copy[0]*b_copy[0] - a_copy[1]*b_copy[1] - a_copy[2]*b_copy[2] - a_copy[3]*b_copy[3];
	result[1] = a_copy[1]*b_copy[0] + a_copy[0]*b_copy[1] - a_copy[3]*b_copy[2] + a_copy[2]*b_copy[3];
	result[2] = a_copy[2]*b_copy[0] + a_copy[3]*b_copy[1] + a_copy[0]*b_copy[2] - a_copy[1]*b_copy[3];
	result[3] = a_copy[3]*b_copy[0] - a_copy[2]*b_copy[1] + a_copy[1]*b_copy[2] + a_copy[0]*b_copy[3];
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
	double v_copy[3];
	memcpy(v_copy, v, sizeof(v_copy));
	int i, j;
	for (i = 0; i < 3; i++){
		result[i] = 0;
		for (j = 0; j < 3; j++){
			result[i] += v_copy[j]*r[i*3 + j];
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

void cross_product(double v_a[3], double v_b[3], double result[3])
{
	double a_copy[3];
	double b_copy[3];
	memcpy(a_copy, v_a, sizeof(a_copy));
	memcpy(b_copy, v_b, sizeof(b_copy));

	result[0] = a_copy[1]*b_copy[2] - a_copy[2]*b_copy[1];
	result[1] = a_copy[2]*b_copy[0] - a_copy[0]*b_copy[2];
	result[2] = a_copy[0]*b_copy[1] - a_copy[1]*b_copy[0];
}

void add_vectors(double v_a[3], double v_b[3], double result[3])
{
	int i;
	for (i = 0; i < 3; i++) {
		result[i] = v_a[i] + v_b[i];
	}
}

double vector_magnitude(double vector[3])
{
	int i;
	double sum;

	sum = 0;
	for (i = 0; i < 3; i++) {
		sum += vector[i]*vector[i];
	}
	return sqrt(sum);
}

void normalize_vector(double vector[3], double result[3])
{
	double magnitude = vector_magnitude(vector);
	if (magnitude == 0) return;
	int i;
	for (i = 0; i < 3; i++) {
		result[i] = vector[i] / magnitude;
	}
}

void normalize_quaternion(double q[4], double result[4])
{
	int i;
	double sum = 0.0;
	for (i = 0; i < 4; i++) {
		sum += q[i]*q[i];
	}
	double norm = sqrt(sum);
	for (i = 0; i < 4; i++) {
		result[i] = q[i] / norm;
	}
}
