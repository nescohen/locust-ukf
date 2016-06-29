/* Nes Cohen */
/* 6/28/2016 */

#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>

#define DEVICE_FILE "/dev/i2c-1"
#define GYRO_ADDR 0x69

int test()
{
	int bus = open(DEVICE_FILE, O_RDWR);
	if (bus < 0) { /* error opening file */
		return 1;
	}

	if (ioctl(bus, I2C_SLAVE, GYRO_ADDR) < 0) {
		/* some sort of error */
		return 1;
	}
	
	uint8_t reg = 0x0F;
	int32_t result;
	result = i2c_smbus_read_byte_data(bus, reg);
	if (result < 0) {
		/* register read error */
		return 1;
	}

	printf("%X\n", result);
	return 0;
}

int main()
{
	int result = test();
	if (result) {
		printf("%d\n", result);
		return 1;
	}
	else return 0;
}
