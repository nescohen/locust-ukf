// Just a test file trying to implement the kalman filter
// Nes Cohen
// March 16, 2017

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "quaternion_util.h"

#define DRONE_MASS 1.0 // Mass of the drone in kilograms
#define THROTTLE_FORCE 0.05 // Conversion between throttle and force in newtons per throttle step

// motor_pos is a 4 cell array of 3-vectors, corrosponding to the position of each motor relative to the center of the drone
const double motor_pos[4][3] = {{0.1, 0.1, 0}, {-0.1, 0.1, 0}, {0.1, -0.1, 0}, {-0.1, -0.1, 0}};

typedef struct state
{
	double position[3]; // Position in meters
	double linear_velocity[3]; // Linear velocity in meters per second
	double angular_velocity[3]; // Vector is the euler axis and its magnitude is angular velocity in radians per second
	double orientation[4]; // Represented as a unit quaternion
} State;

typedef struct controls
{
	int motors[4]; // 0 = left front, 1 = right front, 2 = left rear, 3 = right rear
} Controls;

void initialize_state(State *state)
{
	double axis[3] = {1, 0, 0};
	int i;
	for (i = 0; i < 3; i++) {
		state->position[i] = 0.0;
		state->linear_velocity[i] = 0.0;
		state->angular_velocity[i] = 0.0;
	}
	gen_quaternion(0.0, axis, state->orientation);
}

void predict_cycle(State *current, State *future, Controls *controls, double delta_t)
{
	int i;
	// unit vector representations for each axis
	// double right[3] = {1, 0, 0}; // x axis
	// double forward[3] = {0, 1, 0}; // y axis
	double up[3] = {0, 0, 1}; // z axis

	// Initialize drone rotation matrix
	double drone_rotation[9];
	generate_matrix(current->orientation, drone_rotation);
	double drone_up[3];
	vector_by_matrix(up, drone_rotation, drone_up);

	// First compute linear acceleration
	double motor_force_scalar = 0;
	for (i = 0; i < 4; i++) {
		motor_force_scalar += controls->motors[i]*THROTTLE_FORCE;
	}
	double motor_accel_scalar = motor_force_scalar / DRONE_MASS;
	double motor_accel_vector[3];
	vector_by_scalar(drone_up, motor_accel_scalar, motor_accel_vector);

	double accel_g[3] = {0, 0, -9.8};
	double net_accel[3];
	add_vectors(accel_g, motor_accel_vector, net_accel); // add in acceleration due to gravity

	// Determine and save final velocity from current acceleration and past velocity
	double cycle_velocity[3];
	vector_by_scalar(net_accel, delta_t, cycle_velocity);
	add_vectors(cycle_velocity, current->linear_velocity, future->linear_velocity);
	
	// Determine and save final position from current velocity and past position
	double delta_p[3];
	vector_by_scalar(current->linear_velocity, delta_t, delta_p);
	add_vectors(delta_p, current->position, future->position);

	// Now compute angular acceleration
	// First torque due to relative torque between motors. EXTREMELY PRELIMINARY
	int delta_diagonal = controls->motors[0] + controls->motors[3] - controls->motors[1] - controls->motors[2];
	double spin_torque[3];
	vector_by_scalar(up, (double)delta_diagonal*THROTTLE_FORCE*2.0, spin_torque);

	// Second torque due to relative linear force between motors
	double net_torque_ln[3]; // net torque vector due to linear force
	memset(net_torque, 0, sizeof(net_torque));
	for (i = 0; i < 4; i++) {
		double v_mf[3]; // vector of motor force
		double torque[3]; // vector of motor torque
		
		vector_by_scalar(drone_up, controls->motors[i]*THROTTLE_FORCE, v_mf); 
		cross_product(v_mf, motor_pos[i], torque);
		add_vectors(torque, net_torque, net_torque);
	}

	// Combine to find the net torque
	double net_torque[3];
	add_vectors(net_torque_ln, spin_torque, net_torque);

	// Calculate angular acceleration and velocity
	double angular_accel[3];
	vector_by_scalar(net_torque, 1.0 / DRONE_MASS, angular_accel);
	/* 
	 * This calculation assumes the drone is a point-mass. In future a more complex
	 * and accurate determination of the drone's moment of inertia may be used
	 */
	double angular_velocity[3];
	vector_by_scalar(angular_accel, delta_t, angular_velocity);
	add_vectors(angular_velocity, current->angular_velocity, future->angular_velocity);
	
	// Apply total angular velocity as change in rotation
	double e_axis[3]; // Euler axis for rotation
	normalize_vector(future->angular_velocity, e_axis);
	double theta = vector_magnitude(future->angular_velocity)*delta_t;
	double rotation[4];
	gen_quaternion(theta, e_axis, rotation);
	mult_quaternion(rotation, current->orientation, future->orientation);
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
	int i;
	State starting;
	State ending;
	Controls controls;
	
	initialize_state(&starting);
	for (i = 0; i < 4; i++) {
		controls.motors[i] = 100;
	}

	predict_step(&starting, &ending, &controls, 1, 0.1);
	
	printf("position = {%f, %f, %f}\n", ending.position[0], ending.position[1], ending.position[2]);
	
	return 0;
}
