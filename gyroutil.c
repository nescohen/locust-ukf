/* Nes Cohen */
/* 6/28/2016 */

#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>

#define DEVICE_FILE "/dev/i2c-1"
#define GYRO_ADDR 0x69
#define POWER_ON 0xCF
#define POWER_OFF 0x00

static int g_bus == -1;

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
	int32_t result = i2c_smbus_write_byte_data(bus, 0x20, POWER_ON);
	if (result < 0) return 1;
	return 0;
}

int gyro_power_off()
{
	int32_t result = i2c_smbus_write_byte_data(bus, 0x20, POWER_OFF);
	if (result < 0) return 1;
	return 0;
}
