/* Nes Cohen */
/* 6/28/2016 */

#include "boardutil.h"
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>

#define SIGNED_16_MAX 0x7FFF

#define GYRO_SENSATIVITY 2000.f
#define ACCL_SENSATIVITY 2.f
#define GRAVITY 9.81f
#define EPSILON 0.5f

#define MAX_CORRECT 0.1f // expressed as fraction of power setting

typedef struct realvect3
{
	double x;
	double y;
	double z;
} Realvect3;

int stop;
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

double angle_between(const double *array)
/* array is size 4: (x,y) of first vector followed by (x,y) of second vector */
{
	double dot = array[0]*array[2] + array[1]*array[3];
	double mag = sqrt(array[0]*array[0] + array[1]*array[1]) * sqrt(array[2]*array[2] + array[3]*array[3]);
	double cosine = dot/mag;
	return acos(cosine)*180/M_PI;
}

void recovery(double x, double y, int power, int *motors)
{
	/*   front
		0     1
	left       right
		2     3
		 rear
	*/

	int i;
	for (i = 0; i < 4; i++) {
		motors[i] = power;
	}

	if (x > 45) {
		motors[0] += power*MAX_CORRECT;
		motors[1] += power*MAX_CORRECT;
		motors[2] -= power*MAX_CORRECT;
		motors[3] -= power*MAX_CORRECT;
	}
	else if (x < -45) {
		motors[0] -= power*MAX_CORRECT;
		motors[1] -= power*MAX_CORRECT;
		motors[2] += power*MAX_CORRECT;
		motors[3] += power*MAX_CORRECT;
	}
	else {
		motors[0] += (int)((float)x/45.f*MAX_CORRECT*power);
		motors[1] += (int)((float)x/45.f*MAX_CORRECT*power);
		motors[2] -= (int)((float)x/45.f*MAX_CORRECT*power);
		motors[3] -= (int)((float)x/45.f*MAX_CORRECT*power);
	}

	if (y > 45) {
		motors[0] -= power*MAX_CORRECT;
		motors[2] -= power*MAX_CORRECT;
		motors[1] += power*MAX_CORRECT;
		motors[3] += power*MAX_CORRECT;
	}
	else if (y < -45) {
		motors[0] += power*MAX_CORRECT;
		motors[2] += power*MAX_CORRECT;
		motors[1] -= power*MAX_CORRECT;
		motors[3] -= power*MAX_CORRECT;
	}
	else {
		motors[0] -= (int)((float)y/45.f*MAX_CORRECT*power);
		motors[2] -= (int)((float)y/45.f*MAX_CORRECT*power);
		motors[1] += (int)((float)y/45.f*MAX_CORRECT*power);
		motors[3] += (int)((float)y/45.f*MAX_CORRECT*power);
	}
}
void trim(int *current_motor_speed, const int *trim_amount){
	int i;
	for(i = 0; i < 4; i++){
		current_motor_speed[i] += trim_amount[i];
	}
}

int main(int argc, char **argv)
{
	signal(SIGINT, inthand);
	signal(SIGTSTP, inthand);

	open_bus(DEVICE_FILE);
	gyro_power_on();
	accl_power_on();
	comp_power_on();
	
	int power;
	if (argc > 1) {
		long input;
		if ((input = strtol(argv[1], NULL, 0)) != 0) {
			power = (int)input;
			printf("Power=%d, input=%ld, argv=\"%s\"\n", power, input, argv[1]);
		}
		else return 1;
	}
	else return 1;
	if (power > 190) power = 190;
	else if (power < 10) power = 10;

	double deg_x = 0;
	double deg_y = 0;
	double deg_z = 0;

	Vector3 last_grav;
	last_grav.x = 0;
	last_grav.y = 0;
	last_grav.z = 0;

	Vector3 init_north;
	comp_poll(&init_north);

	int motors[4] = {0,0,0,0};
	long total = 0;
	if (clock_gettime(CLOCK_REALTIME, &curr_clock) < 0) stop = 1;
	while(!stop) {
		Vector3 gyro;
		Vector3 accel;
		Vector3 north;
		Realvect3 grav;

		last_clock = curr_clock;
		if (clock_gettime(CLOCK_REALTIME, &curr_clock) < 0) stop = 1;
		gyro_poll(&gyro);
		accl_poll(&accel);
		comp_poll(&north);
		long elapsed = curr_clock.tv_nsec - last_clock.tv_nsec + (curr_clock.tv_sec - last_clock.tv_sec)*1000000000;
		total += elapsed;

		deg_x += (double)gyro.x/(double)SIGNED_16_MAX*GYRO_SENSATIVITY*elapsed/1000000000.f;
		deg_y += (double)gyro.y/(double)SIGNED_16_MAX*GYRO_SENSATIVITY*elapsed/1000000000.f;
		deg_z += (double)gyro.z/(double)SIGNED_16_MAX*GYRO_SENSATIVITY*elapsed/1000000000.f;

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
			deg_x = pitch*0.02 + deg_x*0.98;
			double roll = atan2(grav.x, grav.z) * 180 / M_PI;
			deg_y = roll*0.02 + deg_y*0.98;
		}

		if (total > 10000000) {
			recovery(deg_x, deg_y, power, motors);
			update_motors(motors);
			total = total % 10000000;
		}
		if (deg_x > 45 || deg_x < -45 || deg_y > 45 || deg_y < -45) {
			stop = 1;
		}

		/* printf("\r%f|%f|%f(%f) ... %f|%f|%f[%f] ... %d|%d|%d| ... %d|%d|%d|%d     ", deg_x, deg_y, deg_z, angle, grav.x, grav.y, grav.z, accel_mag, north.x, north.y, north.z, motors[0], motors[1], motors[2], motors[3]); */
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
	comp_power_on();

	return 0;
}
