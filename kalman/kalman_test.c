// Just a test file trying to implement the kalman filter
// Nes Cohen
// March 16, 2017

#include <stdio.h>
#include <stdlib.h>

#define DRONE_MASS 1.0 // Mass of the drone in kilograms
#define THROTTLE_FORCE 5.0 // Conversion between throttle and force in newtons per throttle step
#define ARM_LENGTH 0.15 // Distance from center of drone to propeller in meters

typedef struct state
{
	double position[3]; // position in meters
	double velocity[3]; // velocity in meters per second
	double orientation[4]; // represented as a unit quaternion
} State;

typedef struct controls
{
	int motors[4]; // 0 = left front, 1 = right front, 2 = left rear, 3 = right rear
} Controls;

void mult_quaternion(double op_a[4], double op_b[4], double result[4])
{
	result[0] = op_a[0]*op_b[0] - op_a[1]*op_b[1] - op_a[2]*op_b[2] - op_a[3]*op_b[3];
	result[1] = op_a[1]*op_b[0] + op_a[0]*op_b[1] - op_a[3]*op_b[2] + op_a[2]*op_b[3];
	result[2] = op_a[2]*op_b[0] + op_a[3]*op_b[1] + op_a[0]*op_b[2] - op_a[1]*op_b[3];
	result[3] = op_a[3]*op_b[0] - op_a[2]*op_b[1] + op_a[1]*op_b[2] + op_a[0]*op_b[3];
}

void generate_matrix(double q[4], double matrix[9]) // Converts quaternion q into equivalent post multiplied rotation matrix
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

void vector_by_matrix(double v[3], double r[9], double result[3]) // Multiplies vector v by matrix r
{
	int i, j;
	for (i = 0; i < 3; i++){
		result[i] = 0;
		for (j = 0; j < 3; j++){
			result[i] += v[i]*r[i*3 + j];
		}
	}
}

int predict_step(State *current, State *future, Controls *controls)
{
	
}

int main()
{
	return 0;
}


