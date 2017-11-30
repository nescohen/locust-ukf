#include "navigation.h"
#include "../hardware/boardutil.h"
#include "../error/error_log.h"
#include "../pid/pid.h"
#include "../math/matrix_util.h"
#include "../math/quaternion_util.h"

#include <time.h>
#include <pthread.h>

#define NAV_PRINT

Pidhist g_hist_x;
Pidhist g_hist_y;

void mrp_to_euler(double *euler_angles, double *mrp)
//expects 3x1 array to write and 3x1 mrp, returns equivelent euler_angle representation from mrp
{
	double angle;
	double axis[3];
	double matrix[9];
	double result[3];
	
	// First convert the mrp into a rotation matrix
	angle = 4*atan(vector_magnitude(mrp));
	normalize_vector(mrp, axis);
	axis_angle_matrix(axis, angle, matrix);

	// Get relevant information from rotation matrix
	result[0] = atan2(matrix[3*1 + 0], matrix[3*0 + 0]);
	result[1] = atan2(-1*matrix[3*2 + 0], sqrt(pow(matrix[3*2 + 1], 2) + pow(matrix[3*2 + 2], 2)));
	result[2] = atan2(matrix[3*2 + 1], matrix[3*2 + 2]);

	memcpy(euler_angles, result, sizeof(result));
}

void sensor_to_array(double array[3], Vector3 sensor, double sensativity)
{
	array[0] = (double)sensor.x / (double)SIGNED_16_MAX * sensativity;
	array[1] = (double)sensor.y / (double)SIGNED_16_MAX * sensativity;
	array[2] = (double)sensor.z / (double)SIGNED_16_MAX * sensativity;
}

void init_drone_state(Drone_state *state)
{
	int i;
	for (i = 0; i < 4; i++) {
		state->motors[i] = 0;
	}
	ukf_param_init(&(state->ukf_params));
}

int start_sensors()
{
	if (open_bus(DEVICE_FILE) != 0) {
		log_error("Failed to open i2c device file.");
		return 1;
	}

	int failure = 0;
	failure = failure || gyro_power_on() < 0;
	failure = failure || accl_power_on() < 0;
	failure = failure || comp_power_on() < 0;
	if (failure) {
		log_error("Failed to power on one or more sensors.");
		return 2;
	}

	return 0;
}

int init_nav(Drone_state *state)
{
	// turn on sensors
	int result = start_sensors();
	if (result != 0) {
		return 1;
	}

	// create and start sensor polling thread
	pthread_t sensor_poll_thread;
	pthread_create(&sensor_poll_thread, NULL, &poll_loop, NULL);

	// create and start motor controlling thread
	pthread_t motor_update_thread;
	pthread_create(&motor_update_thread, NULL, &motor_loop, NULL);

	init_drone_state(state);

	return 0;
}

int align_nav(Drone_state *state)
{
	int align = 1;
	double align_mean[SIZE_MEASUREMENT];
	double align_var[SIZE_MEASUREMENT];
	double weight_sum = 0;

	struct timespec curr_clock;
	struct timespec last_clock;

	if (clock_gettime(CLOCK_REALTIME, &curr_clock) < 0) {
		log_error("Failed to get system time during alignment");
		return 1;
	}
	while (align == 1) {
		// calculate time elapsed during last loop iteration
		last_clock = curr_clock;
		if (clock_gettime(CLOCK_REALTIME, &curr_clock) < 0) {
			log_error("Failed to get system time during alignment");
			return 1;
		}
		double elapsed = (double)(curr_clock.tv_nsec - last_clock.tv_nsec)*NSEC_TO_SEC + (double)(curr_clock.tv_sec - last_clock.tv_sec); 

		// get data from sensors
		Vector3 gyro;
		Vector3 accel;
		Vector3 north;
		double measurement[SIZE_MEASUREMENT];
		get_sensor_data(&gyro, &accel);
		get_compass_data(&north);
		sensor_to_array(measurement, accel, ACCL_SENSATIVITY);
		sensor_to_array(measurement + 3, north, 1);
		sensor_to_array(measurement + 6, gyro, GYRO_SENSATIVITY);

		// calculate mean and variance incrementally
		int i;
		for (i = 0; i < SIZE_MEASUREMENT; i++) {
			double old_mean = align_mean[i];
			align_mean[i] = (align_mean[i]*weight_sum + measurement[i]*elapsed) / (weight_sum + elapsed);
			align_var[i] += elapsed * (measurement[i] - old_mean) * (measurement[i] - align_mean[i]);
		}
		weight_sum += elapsed;
		if(weight_sum >= ALIGN_TIME) align = 0;
	}
	matrix_scale(align_var, align_var, 1 / weight_sum, SIZE_MEASUREMENT, 1);
	memcpy(g_down, align_mean, sizeof(g_down));
	memcpy(g_north, align_mean + 3, sizeof(g_north));

	memset(state->ukf_params.state, 0, 3*sizeof(double));
	memcpy(state->ukf_params.state + 3, align_mean + 6, 3*sizeof(double));

#ifdef NAV_PRINT
	printf("Align State:\n");
	matrix_quick_print(state->ukf_params.covariance, SIZE_STATE, SIZE_STATE);
#endif

	return 0;
}

int update_nav(Drone_state *state, double delta_t)
{
	
}

void stop_nav()
{
	stop_hardware_loop();
	pthread_join(sensor_poll_thread, NULL);
	gyro_power_off();
	accl_power_off();
	comp_power_off();
	pthread_join(motor_update_thread, NULL);
}

void recovery_pid(double x, double y, Controls *controls, int *motors, Pidhist *hist_x, Pidhist *hist_y, double delta_t)
{
	double error_x = x - controls->roll;
	double error_y = y - controls->pitch;

	double correct_x = PID_SCALE*pid(hist_x, error_x, delta_t);
	double correct_y = PID_SCALE*pid(hist_y, error_y, delta_t);

	motors[0] = controls->throttle + (int)(0.5*correct_x) + (int)(0.5*correct_y);
	motors[1] = controls->throttle + (int)(0.5*correct_x) + (int)(-0.5*correct_y);
	motors[2] = controls->throttle + (int)(-0.5*correct_x) + (int)(0.5*correct_y);
	motors[3] = controls->throttle + (int)(-0.5*correct_x) + (int)(-0.5*correct_y);

	int i;
	for (i = 0; i < 4; i++) {
		if (motors[i] < 0) {
			motors[i] = 0;
		}
		if (motors[i] > 200) {
			motors[i] = 200;
		}
	}

#ifdef PID_PRINT
	static int cap = 0;
	if (cap > 1000) {
		printf("[%f, %f] => [%f, %f] => [%d, %d, %d, %d]\n", error_x, error_y, correct_x, correct_y, motors[0], motors[1], motors[2], motors[3]);
		cap = 0;
	}
	else {
		cap += 1;
	}
#endif
}
