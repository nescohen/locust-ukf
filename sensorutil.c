/* Nes Cohen */
/* 6/28/2016 */

#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdio.h>
#include "sensorutil.h"

#define GYRO_ADDR 0x69
#define ACCL_ADDR 0x19
#define GYRO_POWER_ON 0xCF /* sets output rate to 800 hz and the low pass cut off to 30 */
#define GYRO_POWER_OFF 0x00
#define ACCL_POWER_ON 0x57
#define ACCL_POWER_OFF 0x07
#define HIGH_PASS_MAX 0x29
#define HIGH_PASS_ON 0x10

static int g_bus = -1; /* used to store the i2c bus file descriptor */

int open_bus(char *filename)
{
	int ref = open(filename, O_RDWR);
	if (ref < 0) {
		return 1;
	}
	g_bus = ref;
	return 0;
}

int gyro_power_on()
{
	if (ioctl(g_bus, I2C_SLAVE, GYRO_ADDR) < 0) {
		return 2;
	}
	int32_t error = i2c_smbus_write_byte_data(g_bus, 0x20, GYRO_POWER_ON);
	if (error < 0) return 1;
	error = i2c_smbus_write_byte_data(g_bus, 0x21, HIGH_PASS_MAX);
	if (error < 0) return 1;
	error = i2c_smbus_write_byte_data(g_bus, 0x24, HIGH_PASS_ON);
	if (error < 0) return 1;
	return 0;
}

int accl_power_on()
{
	if (ioctl(g_bus, I2C_SLAVE, ACCL_ADDR) < 0) {
		/* some sort of error */
		return 2;
	}
	
	int32_t result;
	result = i2c_smbus_write_byte_data(g_bus, 0x20, ACCL_POWER_ON);
	if (result < 0) {
		return 3;
	}
	return 0;
}

int gyro_power_off()
{
	if (ioctl(g_bus, I2C_SLAVE, GYRO_ADDR) < 0) {
		return 2;
	}
	int32_t error = i2c_smbus_write_byte_data(g_bus, 0x20, GYRO_POWER_OFF);
	if (error < 0) return 1;
	return 0;
}

int accl_power_off()
{
	if (ioctl(g_bus, I2C_SLAVE, ACCL_ADDR) < 0) {
		/* some sort of error */
		return 2;
	}
	int32_t result;
	result = i2c_smbus_write_byte_data(g_bus, 0x20, ACCL_POWER_OFF);
	if (result < 0) {
		return 3;
	}
	return 0;
}

int gyro_poll(Vector3 *output)
{
	if (ioctl(g_bus, I2C_SLAVE, GYRO_ADDR) < 0) {
		return 2;
	}
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

int accl_poll(Vector3 *output)
{
	if (ioctl(g_bus, I2C_SLAVE, ACCL_ADDR) < 0) {
		/* some sort of error */
		return 2;
	}
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
