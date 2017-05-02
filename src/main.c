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

#define GYRO_SENSATIVITY 250.f
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

		gyro_poll(&gyro);
		accl_poll(&accel);
		comp_poll(&north);

		double v_ang_x = ((double)gyro.x/(double)SIGNED_16_MAX) * GYRO_SENSATIVITY;
		double v_ang_y = ((double)gyro.y/(double)SIGNED_16_MAX) * GYRO_SENSATIVITY;
		double v_ang_z = ((double)gyro.z/(double)SIGNED_16_MAX) * GYRO_SENSATIVITY;
		deg_x += v_ang_x * (elapsed/1e9);
		deg_y += v_ang_y * (elapsed/1e9);
		deg_z += v_ang_z * (elapsed/1e9);

		grav.x =    ((double)(accel.x)/(double)SIGNED_16_MAX*ACCL_SENSATIVITY*GRAVITY)*0.3 + 0.7*last_grav.x;
		grav.y = (-1*(double)(accel.y)/(double)SIGNED_16_MAX*ACCL_SENSATIVITY*GRAVITY)*0.3 + 0.7*last_grav.y;
		grav.z =    ((double)(accel.z)/(double)SIGNED_16_MAX*ACCL_SENSATIVITY*GRAVITY)*0.3 + 0.7*last_grav.z;
		last_grav.x = grav.x;
		last_grav.y = grav.y;
		last_grav.z = grav.z;

		double vect_pair[4];
		vect_pair[0] = (double)north.x;
		vect_pair[1] = (double)north.y;
		vect_pair[2] = (double)init_north.x;
		vect_pair[3] = (double)init_north.y;
		double angle = angle_between(vect_pair);

		double accel_mag = magnitude(&grav);
		if( accel_mag < GRAVITY + EPSILON && accel_mag > GRAVITY - EPSILON ) {
			double pitch = atan2(grav.y, grav.z) * 180 / M_PI;
			// deg_x = pitch*0.8 + deg_x*0.2;
			double roll = atan2(grav.x, grav.z) * 180 / M_PI;
			// deg_y = roll*0.8 + deg_y*0.2;
		}

		if (total > 10000000) {
			/*if (check_update()) {
				get_controls(&controls);
			}*/
			recovery_pid(deg_x, deg_y, &controls, motors, &hist_x, &hist_y, 0.01); // dt is one one-hundredth of a second
			// update_motors(motors);
			total = total % 10000000;
		}
		//printf("\r%f|%f|%f(%f) ... %f|%f|%f[%f] ... %f|%f|%f| ... %d|%d|%d|%d     ", deg_x, deg_y, deg_z, angle, grav.x, grav.y, grav.z, accel_mag, v_ang_x, v_ang_y, v_ang_z, motors[0], motors[1], motors[2], motors[3]);
		printf("\r%f|%f|%f || %f|%f|%f || %f         ", deg_x, deg_y, deg_z, v_ang_x, v_ang_y, v_ang_z, percentage);
	}
	printf("\n");

	int i;
	for (i = 0; i < 4; i++)
	{
		motors[i] = 0;
	}
	update_motors(motors);
	gyro_power_off();
	accl_power_off();
	comp_power_on();

	log_complete();

	return 0;
}
