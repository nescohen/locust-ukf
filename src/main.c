/* Nes Cohen */
/* 6/28/2016 */

#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>

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
#define EPSILON 0.5f

#define ALIGN_TIME 2.5 // in seconds

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

void recovery_pid(double x, double y, Controls *controls, int *motors, Pidhist *hist_x, Pidhist *hist_y, double delta_t)
{
	double error_x = x - controls->roll;
	double error_y = y - controls->pitch;

	double correct_x = pid(hist_x, error_x, delta_t);
	double correct_y = pid(hist_y, error_y, delta_t);

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
}

int main(int argc, char **argv)
{
	struct timespec curr_clock;
	struct timespec last_clock;

	signal(SIGINT, inthand);
	signal(SIGTSTP, inthand);

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

	//TODO: have the alignment step also run a self-test on the gyro and make corrections
	//TODO: find the source of the NaNs. In the final program it should be impossible to generate a NaN

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
			failure = 0;
			failure = failure || gyro_poll(&gyro) < 0;
			failure = failure || accl_poll(&accel) < 0;
			failure = failure || comp_poll(&north) < 0;
			if (failure) {
				log_error("Failed to get measurements from one or more sensors (during alignment).");
				stop = 1;
			}
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

		failure = 0;
		failure = failure || gyro_poll(&gyro) < 0;
		failure = failure || accl_poll(&accel) < 0;
		failure = failure || comp_poll(&north) < 0;
		if (failure) {
			log_error("Failed to get measurements from one or more sensors.");
			stop = 1;
		}

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
		printf("Attitude MRP = [%f, %f, %f]^t\n", ukf.state[0], ukf.state[1], ukf.state[2]);
	}

	gyro_power_off();
	accl_power_off();
	comp_power_on();

	log_complete();

	return 0;
}
