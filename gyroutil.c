/* Nes Cohen */
/* 6/28/2016 */

#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdio.h>
#include "gyroutil.h"

#define GYRO_ADDR 0x69
#define POWER_ON 0xCF /* sets output rate to 800 hz and the low pass cut off to 30 */
#define POWER_OFF 0x00
#define HIGH_PASS_MAX 0x29
#define HIGH_PASS_ON 0x10

static int g_bus = -1; /* used to store the i2c bus file descriptor */

int open_bus(char *filename)
{
	int ref = open(filename, GYRO_ADDR);
	if (ref < 0) {
		return 1;
	}
	g_bus = ref;
	if (ioctl(g_bus, I2C_SLAVE, GYRO_ADDR) < 0) {
		return 2;
	}
	return 0;
}

int gyro_power_on()
{
	int32_t error = i2c_smbus_write_byte_data(g_bus, 0x20, POWER_ON);
	if (error < 0) return 1;
	error = i2c_smbus_write_byte_data(g_bus, 0x21, HIGH_PASS_MAX);
	if (error < 0) return 1;
	error = i2c_smbus_write_byte_data(g_bus, 0x24, HIGH_PASS_ON);
	if (error < 0) return 1;
	return 0;
}

int gyro_power_off()
{
	int32_t error = i2c_smbus_write_byte_data(g_bus, 0x20, POWER_OFF);
	if (error < 0) return 1;
	return 0;
}

int gyro_poll(Vector3 *output)
{
	int32_t result;

	result = i2c_smbus_read_byte_data(g_bus, 0x28);
	if (result < 0) return 1;
	output->x = (int16_t)result;
	result = i2c_smbus_read_byte_data(g_bus, 0x29);
	if (result < 0) return 1;
	output->x = (int16_t)result << 8;
	output->x += 250;

	result = i2c_smbus_read_byte_data(g_bus, 0x2A);
	if (result < 0) return 1;
	output->y = (int16_t)result;
	result = i2c_smbus_read_byte_data(g_bus, 0x2B);
	if (result < 0) return 1;
	output->y = (int16_t)result << 8;

	result = i2c_smbus_read_byte_data(g_bus, 0x2C);
	if (result < 0) return 1;
	output->z = (int16_t)result;
	result = i2c_smbus_read_byte_data(g_bus, 0x2D);
	if (result < 0) return 1;
	output->z = (int16_t)result << 8;

	return 0;
}
