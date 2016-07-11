/* Nes Cohen */
/* 6/28/2016 */

#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>

#define DEVICE_FILE "/dev/i2c-1"
#define GYRO_ADDR 0x69
#define ACCL_ADDR 0x19
#define ACCL_POWER_ON 0x57
#define ACCL_POWER_OFF 0x07
#define SIGNED_16_MAX 0x7FFF

typedef struct vector3 {
	int16_t x;
	int16_t y;
	int16_t z;
} Vector3;

int g_bus;

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

int accl_poll(Vector3 *output)
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

int main()
{
	int result = on();
	if (result) {
		printf("%d\n", result);
		return 1;
	}

	Vector3 accel;
	Vector3 avg;

	int i;
	for (i = 0; i < 1000; i++) {
		if (get_accel(&accel)) return 1; /* read error */

		avg.x = ( avg.x*i + accel.x ) / (i + 1);
		avg.y = ( avg.y*i + accel.y ) / (i + 1);
		avg.z = ( avg.z*i + accel.z ) / (i + 1);
	}

	double accel_x = (double)avg.x / (double)SIGNED_16_MAX * 2.f * 9.81f;
	double accel_y = (double)avg.y / (double)SIGNED_16_MAX * 2.f * 9.81f;
	double accel_z = (double)avg.z / (double)SIGNED_16_MAX * 2.f * 9.81f;

	printf("x=%f, y=%f, z=%f\n", accel_x, accel_y, accel_z);

	if( (result = off()) != 0 ) return result;

	return 0;
}
