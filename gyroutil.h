#ifndef GYROUTIL_H
#define GYROUTIL_H

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
int gyro_power_off();
int gyro_poll(Vector3 *output);

#endif
