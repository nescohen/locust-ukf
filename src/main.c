/* Nes Cohen */
/* 6/28/2016 */

#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <float.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>
#include <errno.h>

#include "hardware/boardutil.h"
#include "hardware/flight-input.h"
#include "error/error_log.h"
#include "pid/pid.h"
#include "kalman/ukf_mrp.h"
#include "math/matrix_util.h"
#include "math/quaternion_util.h"

#define SIGNED_16_MAX 0x7FFF
#define NSEC_TO_SEC 1e-9
#define SEC_TO_NSEC (long)1e9

#define GYRO_SENSATIVITY 4.36332f
#define ACCL_SENSATIVITY 2.f * GRAVITY // 2gs max converted to ms^-2
#define GYRO_VARIANCE 0.193825 // 11.111... degress in radians
#define ACCL_VARIANCE 1.1772
#define COMP_VARIANCE 1

#define GRAVITY 9.81f
#define EPSILON 1.0f

#define ALIGN_TIME 2.5 // in seconds

#define PID_SCALE 30 // 100 / 2 * M_PI

volatile sig_atomic_t stop;
double g_north[3];
double g_down[3];

void inthand(int signum)
{
	stop = 1;
}

void sensor_to_array(double array[3], Vector3 sensor, double sensativity)
{
	array[0] = (double)sensor.x / (double)SIGNED_16_MAX * sensativity;
	array[1] = (double)sensor.y / (double)SIGNED_16_MAX * sensativity;
	array[2] = (double)sensor.z / (double)SIGNED_16_MAX * sensativity;
}

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
	//printf("[%f, %f] => [%f, %f] => [%d, %d, %d, %d]\n", error_x, error_y, correct_x, correct_y, motors[0], motors[1], motors[2], motors[3]);
}

int detect_nans(double *array, int size)
{
	int i;
	int nan = 0;
	for (i = 0; i < size; i++) {
		if (array[i] != array[i]) nan = 1;
	}
	return nan;
}

int main(int argc, char **argv)
{
	struct timespec curr_clock;
	struct timespec last_clock;

	signal(SIGINT, inthand);
	signal(SIGTSTP, inthand);

	int user_throttle = -1;
	if (argc == 3) {
		char *end = NULL;
		long input = strtol(argv[2], &end, 10);
		if (end != argv[2] && !strcmp(argv[1], "throttle") && input >= 0 && input <= 200) {
			user_throttle = (int) input;
		}
		else {
			printf("Invalid arguments. Exiting.\n");
			return 1;
		}
	}
	else if (argc != 1) {
		printf("Invalid arguments. Exiting.\n");
		return 1;
	}

	if (open_bus(DEVICE_FILE) != 0) {
		log_error("Failed to open i2c device file.");
		stop = 1;
	}
	int failure = 0;
	failure = failure || gyro_power_on() < 0;
	failure = failure || accl_power_on() < 0;
	failure = failure || comp_power_on() < 0;
	if (failure) {
		log_error("Failed to power on one or more sensors.");
		stop = 1;
	}

	pthread_t sensor_poll_thread;
	pthread_create(&sensor_poll_thread, NULL, &poll_loop, NULL);
	pthread_t motor_update_thread;
	pthread_create(&motor_update_thread, NULL, &motor_loop, NULL);

	//TODO: have the alignment step also run a self-test on the gyro and make corrections
	// Addendum: the filter seems to be performing quite well with out this, it may not be necessary
	//TODO: find the source of the NaNs. In the final program it should be impossible to generate a NaN

	// array containing current motor throttle values
	int motors[4];

	// int i;
	// for(i = 0; i < 4; i++) {
	// 	motors[i] = 30;
	// }
	// if (user_throttle > 0) update_motors(motors);

	Ukf_parameters ukf;
	if (!stop) {
		int align = 1;
		double align_mean[SIZE_MEASUREMENT];
		double align_var[SIZE_MEASUREMENT];
		double weight_sum = 0;
		if (clock_gettime(CLOCK_REALTIME, &curr_clock) < 0) stop = 1;
		while (align == 1 && !stop) {
			// calculate time elapsed during last loop iteration
			last_clock = curr_clock;
			if (clock_gettime(CLOCK_REALTIME, &curr_clock) < 0) stop = 1;
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

		ukf_param_init(&ukf);

		memset(ukf.state, 0, 3*sizeof(double));
		memcpy(ukf.state + 3, align_mean + 6, 3*sizeof(double));

		printf("Align State:\n");
		matrix_quick_print(ukf.covariance, SIZE_STATE, SIZE_STATE);
	}

	log_error("END ALIGNMENT");

	// for (i = 0; i < 4; i++) {
	// 	motors[i] = 0;
	// }
	// if (user_throttle > 0) update_motors(motors);

	// allocate and initialize history for pid
	Pidhist hist_x;
	Pidhist hist_y;
	init_hist(&hist_x);
	init_hist(&hist_y);

	double measurement[SIZE_MEASUREMENT];
	if (clock_gettime(CLOCK_REALTIME, &curr_clock) < 0) stop = 1;
	while(!stop) {
		// declarations
		Vector3 gyro;
		Vector3 accel;
		Vector3 north;

		// calculate time elapsed during last loop iteration
		last_clock = curr_clock;
		if (clock_gettime(CLOCK_REALTIME, &curr_clock) < 0) stop = 1;
		double elapsed = (double)(curr_clock.tv_nsec - last_clock.tv_nsec)*NSEC_TO_SEC + (double)(curr_clock.tv_sec - last_clock.tv_sec); 

		get_sensor_data(&gyro, &accel);
		get_compass_data(&north);

		double accel_var[3];
		double comp_var[3];
		double gyro_var[3];
		double acceleration[3];
		sensor_to_array(acceleration, accel, ACCL_SENSATIVITY);
		double accel_mag = vector_magnitude(acceleration);
		if (accel_mag + EPSILON > GRAVITY && accel_mag - EPSILON < GRAVITY) {
			normalize_vector(acceleration, measurement + 0);
			matrix_init(accel_var, ACCL_VARIANCE, 3, 1);
		}
		else {
			matrix_init(accel_var, DBL_MAX, 3, 1);
		}
		sensor_to_array(measurement + 3, north, 1);
		normalize_vector(measurement + 3, measurement + 3);
		matrix_init(comp_var, COMP_VARIANCE, 3, 1);
		sensor_to_array(measurement + 6, gyro, GYRO_SENSATIVITY);
		matrix_init(gyro_var, GYRO_VARIANCE, 3, 1);

		int i;
		for (i = 0; i < 3; i++) {
			ukf.R[i*SIZE_MEASUREMENT + i] = accel_var[i];
		}
		for (i = 3; i < 6; i++) {
			ukf.R[i*SIZE_MEASUREMENT + i] = comp_var[i - 3];
		}
		for (i = 6; i < 9; i++) {
			ukf.R[i*SIZE_MEASUREMENT + i] = gyro_var[i - 6];
		}

		ukf_run(&ukf, measurement, elapsed);
		//printf("elapsed - %f\n", elapsed);

		// convert mrp to euler angles for pid
		double euler_angles[3];
		mrp_to_euler(euler_angles, ukf.state);

		if (detect_nans(measurement, SIZE_MEASUREMENT) ||
			detect_nans(euler_angles, 3))
		{
			printf("NAN detected\n");
			stop = 1;
			break;
		}

		Controls controls;
		controls.throttle = (user_throttle == -1) ? 100 : user_throttle;
		controls.roll = 0;
		controls.pitch = 0;
		recovery_pid(euler_angles[2], euler_angles[1], &controls, motors, &hist_x, &hist_y, elapsed);
		
		if (user_throttle > 0) set_throttle(motors);

		// printf("Attitude MRP = [%f, %f, %f] ->\t[%f, %f, %f]^t\t (%f, %f, %f)\n", measurement[6], measurement[7], measurement[8], ukf.state[0], ukf.state[1], ukf.state[2], euler_angles[0], euler_angles[1], euler_angles[2]);
	}

	// pthread_kill(sensor_poll_thread, SIGKILL);
	stop_loop();

	// memset(motors, 0, sizeof(motors));
	// set_throttle(motors);

	pthread_join(sensor_poll_thread, NULL);

	gyro_power_off();
	accl_power_off();
	comp_power_off();

	log_complete();

	pthread_join(motor_update_thread, NULL);

	return 0;
}
