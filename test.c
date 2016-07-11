/* Nes Cohen */
/* 6/28/2016 */
#include "sensorutil.h"
#include <time.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>

#define SIGNED_16_MAX 0x7FFF
#define ALIGNMENT_ROUNDS 1000

volatile int stop;
struct timespec curr_clock;
struct timespec last_clock;

void inthand(int signum)
{
	stop = 1;
}

int align(Vector3 *gravity)
/* only call after the bus has been opened */
/* returns the vector for the gravitational force */
{
	Vector3 data;
	int32_t avg_x = 0;
	int32_t avg_y = 0;
	int32_t avg_z = 0;

	int i;
	for (i = 0; i < ALIGNMENT_ROUNDS; i++) {
		accl_poll(&data);
		avg_x += (int32_t)data.x;
		avg_y += (int32_t)data.y;
		avg_z += (int32_t)data.z;
	}

	avg_x /= ALIGNMENT_ROUNDS;
	avg_y /= ALIGNMENT_ROUNDS;
	avg_z /= ALIGNMENT_ROUNDS;
	gravity->x = avg_x;
	gravity->y = avg_y;
	gravity->z = avg_z;
}

int main()
{
	signal(SIGINT, inthand);

	open_bus(DEVICE_FILE);
	gyro_power_on();
	accl_power_on();

	Vector3 gravity;
	align(&gravity);

	double deg_x = 0;
	double deg_y = 0;
	double deg_z = 0;
	double vel_x = 0;
	double vel_y = 0;
	double vel_z = 0;
	double pos_x = 0;
	double pos_y = 0;
	double pos_z = 0;
	if (clock_gettime(CLOCK_REALTIME, &curr_clock) < 0) stop = 1;
	while(!stop) {
		Vector3 gyro;
		Vector3 accel;
		last_clock = curr_clock;
		if (clock_gettime(CLOCK_REALTIME, &curr_clock) < 0) stop = 1;
		gyro_poll(&gyro);
		accl_poll(&accel);
		long elapsed = curr_clock.tv_nsec - last_clock.tv_nsec + (curr_clock.tv_sec - last_clock.tv_sec)*1000000000;

		deg_x += (double)gyro.x / 32768.f * 250.f * elapsed / 1000000000.f;
		deg_y += (double)gyro.y / 32768.f * 250.f * elapsed / 1000000000.f;
		deg_z += (double)gyro.z / 32768.f * 250.f * elapsed / 1000000000.f;

		vel_x += (double)(accel.x - gravity.x) / (double)SIGNED_16_MAX * 2.f * 9.81f * elapsed / 1000000000.f;
		vel_y += (double)(accel.y - gravity.y) / (double)SIGNED_16_MAX * 2.f * 9.81f * elapsed / 1000000000.f;
		vel_z += (double)(accel.z - gravity.z) / (double)SIGNED_16_MAX * 2.f * 9.81f * elapsed / 1000000000.f;

		pos_x += vel_x * elapsed / 1000000000.f;
		pos_y += vel_y * elapsed / 1000000000.f;
		pos_z += vel_z * elapsed / 1000000000.f;

		printf("\r%f|%f|%f ... %f|%f|%f", deg_x, deg_y, deg_z, pos_x, pos_y, pos_z);
	}
	printf("\n");
	gyro_power_off();
	accl_power_off();

	return 0;
}
