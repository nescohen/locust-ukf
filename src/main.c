/* Nes Cohen */
/* 6/28/2016 */

#include "nav/navigation.h"
#include "hardware/boardutil.h"
#include "hardware/flight-input.h"
#include "error/error_log.h"
#include "pid/pid.h"
#include "kalman/ukf_mrp.h"
#include "math/matrix_util.h"
#include "math/quaternion_util.h"
#include "client/client.h"

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
#include <fcntl.h>

#define GYRO_SENSATIVITY 4.36332f
#define ACCL_SENSATIVITY (2.f * GRAVITY) // 2gs max converted to ms^-2
#define GYRO_VARIANCE 0.193825 // 11.111... degress in radians
#define ACCL_VARIANCE 1.1772
#define COMP_VARIANCE 1.1772

#define SIGNED_16_MAX 0x7FFF
#define NSEC_TO_SEC 1e-9
#define SEC_TO_NSEC ((long)1e9)

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

int detect_nans(double *array, int size)
{
	int i;
	int nan = 0;
	for (i = 0; i < size; i++) {
		if (array[i] != array[i]) nan = 1;
	}
	return nan;
}

int decode_int(char *buffer)
// WARNING: assumes int is at least 32 bits
// decodes assuming little endian
{
	int result = 0;
	int i;
	for (i = 0; i < 4; i++) {
		result |= (int)buffer[i] << i*8;
	}

	return result;
}

int main(int argc, char **argv)
{
	// signal handlers to prevent motors from staying on
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

	int sock = -1;
	if (user_throttle < 0) {
		sock = establish_connection();
	}

	// create and start network handling thread
	pthread_t network_client_thread;
	pthread_create(&network_client_thread, NULL, &network_client_start);

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

	// for (i = 0; i < 4; i++) {
	// 	motors[i] = 0;
	// }
	// if (user_throttle > 0) update_motors(motors);

	Drone_state drone_state;

	// allocate and initialize history for pid
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
		
		if (user_throttle >= 0) set_throttle(motors);

		//printf("Attitude MRP = [%f, %f, %f] ->\t[%f, %f, %f]^t\t (%f, %f, %f)\r", measurement[6], measurement[7], measurement[8], ukf.state[0], ukf.state[1], ukf.state[2], euler_angles[0], euler_angles[1], euler_angles[2]);
	}

	log_complete();

	return 0;
}
