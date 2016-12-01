/* Nes Cohen */
/* 6/28/2016 */

/* TODO: look for divide by zero error */

#include "boardutil.h"
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>

#define SIGNED_16_MAX 0x7FFF

#define GYRO_SENSATIVITY 2000.f
#define ACCL_SENSATIVITY 2.f
#define GRAVITY 9.81f
#define EPSILON 0.5f

typedef struct realvect3
{
	double x;
	double y;
	double z;
} Realvect3;

volatile int stop;
struct timespec curr_clock;
struct timespec last_clock;

void inthand(int signum)
{
	stop = 1;
}

inline double magnitude(Realvect3 *vector)
{
	double sum = vector->x*vector->x + vector->y*vector->y + vector->z*vector->z;
	return sqrt(sum);
}

void recovery(double x, double y, int *motors)
{
	/*   front
		0     1
	left       right
		2     3
		 rear
	*/

	int i;
	for (i = 0; i < 4; i++) {
		motors[i] = 100;
	}

	if (x > 45) {
		motors[0] -= 50;
		motors[1] -= 50;
		motors[2] += 50;
		motors[3] += 50;
	}
	else if (x < -45) {
		motors[0] += 50;
		motors[1] += 50;
		motors[2] -= 50;
		motors[3] -= 50;
	}
	else {
		motors[0] -= (int)x;
		motors[1] -= (int)x;
		motors[2] += (int)x;
		motors[3] += (int)x;
	}

	if (y > 45) {
		motors[0] -= 50;
		motors[2] -= 50;
		motors[1] += 50;
		motors[3] += 50;
	}
	else if (y < -45) {
		motors[0] += 50;
		motors[2] += 50;
		motors[1] -= 50;
		motors[3] -= 50;
	}
	else {
		motors[0] -= (int)y;
		motors[2] -= (int)y;
		motors[1] += (int)y;
		motors[3] += (int)y;
	}
}

void recovery_debug(double x, double y, int *motors)
{
	/*   front
		0     1
	left       right
		2     3
		 rear
	*/

	int i;
	for (i = 0; i < 4; i++) {
		motors[i] = 50;
	}

	if (x > 45) {
		motors[0] -= 25;
		motors[1] -= 25;
		motors[2] += 25;
		motors[3] += 25;
	}
	else if (x < -45) {
		motors[0] += 25;
		motors[1] += 25;
		motors[2] -= 25;
		motors[3] -= 25;
	}
	else {
		motors[0] -= (int)(x / 2);
		motors[1] -= (int)(x / 2);
		motors[2] += (int)(x / 2);
		motors[3] += (int)(x / 2);
	}

	if (y > 45) {
		motors[0] -= 25;
		motors[2] -= 25;
		motors[1] += 25;
		motors[3] += 25;
	}
	else if (y < -45) {
		motors[0] += 25;
		motors[2] += 25;
		motors[1] -= 25;
		motors[3] -= 25;
	}
	else {
		motors[0] -= (int)(y / 2);
		motors[2] -= (int)(y / 2);
		motors[1] += (int)(y / 2);
		motors[3] += (int)(y / 2);
	}
}
void trim(int *current_motor_speed, int *trim_amount){
	for(int i = 0, i < 4, i++){
		current_motor_speed[i] += trim_amount[i];
	}
}

int main()
{
	signal(SIGINT, inthand);

	open_bus(DEVICE_FILE);
	gyro_power_on();
	accl_power_on();

	double deg_x = 0;
	double deg_y = 0;
	double deg_z = 0;

	int motors[4] = {0,0,0,0};
	long total = 0;
	if (clock_gettime(CLOCK_REALTIME, &curr_clock) < 0) stop = 1;
	while(!stop) {
		Vector3 gyro;
		Vector3 accel;
		Realvect3 grav;

		last_clock = curr_clock;
		if (clock_gettime(CLOCK_REALTIME, &curr_clock) < 0) stop = 1;
		gyro_poll(&gyro);
		accl_poll(&accel);
		long elapsed = curr_clock.tv_nsec - last_clock.tv_nsec + (curr_clock.tv_sec - last_clock.tv_sec)*1000000000;
		total += elapsed;

		deg_x += (double)gyro.x / (double)SIGNED_16_MAX * GYRO_SENSATIVITY * elapsed / 1000000000.f;
		deg_y += (double)gyro.y / (double)SIGNED_16_MAX * GYRO_SENSATIVITY * elapsed / 1000000000.f;
		deg_z += (double)gyro.z / (double)SIGNED_16_MAX * GYRO_SENSATIVITY * elapsed / 1000000000.f;

		grav.x =    (double)(accel.x) / (double)SIGNED_16_MAX * ACCL_SENSATIVITY * GRAVITY;
		grav.y = -1*(double)(accel.y) / (double)SIGNED_16_MAX * ACCL_SENSATIVITY * GRAVITY;
		grav.z =    (double)(accel.z) / (double)SIGNED_16_MAX * ACCL_SENSATIVITY * GRAVITY;

		double accel_mag = magnitude(&grav);
		if( accel_mag < GRAVITY + EPSILON && accel_mag > GRAVITY - EPSILON ) {
			double pitch = atan2(grav.y, grav.z) * 180 / M_PI;
			deg_x = pitch*0.02 + deg_x*0.98;
			double roll = atan2(grav.x, grav.z) * 180 / M_PI;
			deg_y = roll*0.02 + deg_y*0.98;
		}

		if (total > 100000000) {
			recovery_debug(deg_x, deg_y, motors);
			update_motors(motors);
			total = total % 100000000;
		}
		if (deg_x > 45 || deg_x < -45 || deg_y > 45 || deg_y < -45) {
			stop = 1;
		}

		/* printf("\r%f|%f|%f ... %f|%f|%f[%ld] ... %d|%d|%d|%d     ", deg_x, deg_y, deg_z, grav.x, grav.y, grav.z, elapsed, motors[0], motors[1], motors[2], motors[3]); */
	}
	/* printf("\n"); */

	int i;
	for (i = 0; i < 4; i++)
	{
		motors[i] = 0;
	}
	update_motors(motors);
	gyro_power_off();
	accl_power_off();

	return 0;
}
