/* Nes Cohen */
/* 6/28/2016 */

#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>

#include "hardware/boardutil.h"
#include "hardware/flight-input.h"
#include "error/error_log.h"
#include "pid/pid.h"

#define SIGNED_16_MAX 0x7FFF

#define GYRO_SENSATIVITY 4.36332f
#define ACCL_SENSATIVITY 2.f
#define GRAVITY 9.81f
#define EPSILON 0.5f

#define MAX_CORRECT 0.1f // expressed as fraction of power setting

volatile sig_atomic_t stop;

void inthand(int signum)
{
	stop = 1;
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
	int test = 0;
	test = test || gyro_power_on();
	test = test || accl_power_on();
	test = test || comp_power_on();
	if (test != 0) {
		log_error("Failed to power on one or more sensors.");
		stop = 1;
	}

	if (clock_gettime(CLOCK_REALTIME, &curr_clock) < 0) stop = 1;
	while(!stop) {
		last_clock = curr_clock;
		if (clock_gettime(CLOCK_REALTIME, &curr_clock) < 0) stop = 1;
		long elapsed = curr_clock.tv_nsec - last_clock.tv_nsec + (curr_clock.tv_sec - last_clock.tv_sec)*1000000000;

		Vector3 gyro;
		Vector3 accel;
		Vector3 north;
		gyro_poll(&gyro);
		accl_poll(&accel);
		comp_poll(&north);

		double omega[3];
		omega[0] = ((double)gyro.x/(double)SIGNED_16_MAX) * GYRO_SENSATIVITY;
		omega[1] = ((double)gyro.y/(double)SIGNED_16_MAX) * GYRO_SENSATIVITY;
		omega[2] = ((double)gyro.z/(double)SIGNED_16_MAX) * GYRO_SENSATIVITY;
	}

	gyro_power_off();
	accl_power_off();
	comp_power_on();

	log_complete();

	return 0;
}
