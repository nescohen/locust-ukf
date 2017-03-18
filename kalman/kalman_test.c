// Just a test file trying to implement the kalman filter
// Nes Cohen
// March 16, 2017

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define DRONE_MASS 1.0 // Mass of the drone in kilograms
#define THROTTLE_FORCE 5.0 // Conversion between throttle and force in newtons per throttle step

// motor_pos is a 4 cell array of 3-vectors, corrosponding to the position of each motor relative to the center of the drone
const double motor_pos[4][3] = {{0.1, 0.1, 0}, {-0.1, 0.1, 0}, {0.1, -0.1, 0}, {-0.1, -0.1, 0}};

typedef struct state
{
	double position[3]; // position in meters
	double linear_velocity[3]; // linear velocity in meters per second
	double angular_velocity[3]; // vector is the euler axis and its magnitude is angular velocity in radians per second
	double orientation[4]; // represented as a unit quaternion
} State;

typedef struct controls
{
	int motors[4]; // 0 = left front, 1 = right front, 2 = left rear, 3 = right rear
} Controls;

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

void predict_cycle(State *current, State *future, Controls *controls, double delta_t)
{
	double drone_rotation[9];
	generate_matrix(current->orientation, drone_rotation);

	//first compute linear acceleration
	double up[3] = {0, 0, 1};
	double force_direction[3];
	vector_by_matrix(up, drone_rotation, force_direction);
	
	double motor_force_scalar = 0;
	int i;
	for (i = 0; i < 4; i++) {
		motor_force_scalar += controls->motors[i]*THROTTLE_FORCE;
	}
	double motor_accel_scalar = motor_force_scalar / DRONE_MASS;
	double motor_accel_vector[3];
	vector_by_scalar(force_direction, motor_accel_scalar, motor_accel_vector);

	double accel_g[3] = {0, 0, -9.8};
	double net_accel[3];
	add_vectors(accel_g, motor_accel_vector, net_accel); // add in acceleration due to gravity
}

void predict_step(State *current, State *future, Controls *controls, double delta_t, double timestep)
{
	State temp_curr = *current;
	State temp_next;
	while (delta_t > timestep) {
		predict_cycle(&temp_curr, &temp_next, controls, timestep);
		temp_curr = temp_next;
		delta_t -= timestep;
	}
	predict_cycle(&temp_curr, future, controls, delta_t);
}

int main()
{
	
	return 0;
}

