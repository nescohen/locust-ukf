// Just a test file trying to implement the kalman filter
// Nes Cohen
// March 16, 2017

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "quaternion_util.h"

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
