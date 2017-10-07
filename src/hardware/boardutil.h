#ifndef BOARDUTIL_H
#define BOARDUTIL_H

#include <stdint.h>

#define DEVICE_FILE "/dev/i2c-1"

typedef struct vector3
{
	int16_t x;
	int16_t y;
	int16_t z;
} Vector3;

int open_bus(char *filename);
int gyro_power_on();
int accl_power_on();
int comp_power_on();
int gyro_power_off();
int accl_power_off();
int comp_power_off();
int gyro_poll(Vector3 *output);
int accl_poll(Vector3 *output);
int comp_poll(Vector3 *output);

int update_motors(int *settings); /* settings points to a 4 size integer array */

void get_sensor_data(Vector3 *gyro, Vector3 *accel);
void get_compass_data(Vector3 *compass);
void stop_loop();
void *poll_loop(void *arg);

#endif
