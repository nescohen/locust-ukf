/* Nes Cohen */
/* 6/28/2016 */
#include "gyroutil.h"
#include <time.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>

volatile int stop;
struct timespec curr_clock;
struct timespec last_clock;

void inthand(int signum)
{
	stop = 1;
}

int main()
{
	signal(SIGINT, inthand);

	open_bus(DEVICE_FILE);
	gyro_power_on();

	double deg_x = 0;
	double deg_y = 0;
	double deg_z = 0;
	if (clock_gettime(CLOCK_REALTIME, &curr_clock) < 0) stop = 1;
	while(!stop) {
		Vector3 gyro;
		last_clock = curr_clock;
		if (clock_gettime(CLOCK_REALTIME, &curr_clock) < 0) stop = 1;
		gyro_poll(&gyro);
		long elapsed = curr_clock.tv_nsec - last_clock.tv_nsec + (curr_clock.tv_sec - last_clock.tv_sec)*1000000000;
		deg_x += (double)gyro.x / 32768.f * 250.f * elapsed / 1000000000.f;
		deg_y += (double)gyro.y / 32768.f * 250.f * elapsed / 1000000000.f;
		deg_z += (double)gyro.z / 32768.f * 250.f * elapsed / 1000000000.f;
		printf("\r%f|%f|%f - %d", deg_x, deg_y, deg_z, elapsed);
	}
	printf("\n");
	gyro_power_off();

	return 0;
}
