/* Nes Cohen */
/* 6/28/2016 */

#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include "boardutil.h"

#define GYRO_ADDR 0x69
#define ACCL_ADDR 0x19
#define GYRO_POWER_ON 0xCF /* sets output rate to 800 hz and the low pass cut off to 30 */
#define GYRO_POWER_OFF 0x00
#define GYRO_HIGH_PASS_MAX 0x29
#define GYRO_HIGH_PASS_ON 0x10
#define GYRO_FULL_SCALE 0x30
#define ACCL_POWER_ON 0x57
#define ACCL_POWER_OFF 0x07
#define ACCL_HIGH_REZ 0x08

#define FORWARD_CONTROL 0x44
#define REAR_CONTROL 0x45
#define MOTOR_PORT 0x01
#define MOTOR_STARBOARD 0x02

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
	error = i2c_smbus_write_byte_data(g_bus, 0x23, GYRO_FULL_SCALE);
	if (error < 0) return 1;
	/*
	error = i2c_smbus_write_byte_data(g_bus, 0x21, GYRO_HIGH_PASS_MAX);
	if (error < 0) return 1;
	error = i2c_smbus_write_byte_data(g_bus, 0x24, GYRO_HIGH_PASS_ON);
	if (error < 0) return 1;
	*/
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
		return 1;
	}
	result = i2c_smbus_write_byte_data(g_bus, 0x23, ACCL_HIGH_REZ);
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
	output->x |= (int16_t)result << 8;

	result = i2c_smbus_read_byte_data(g_bus, 0x2A);
	if (result < 0) return 1;
	output->y = (int16_t)result;
	result = i2c_smbus_read_byte_data(g_bus, 0x2B);
	if (result < 0) return 1;
	output->y |= (int16_t)result << 8;

	result = i2c_smbus_read_byte_data(g_bus, 0x2C);
	if (result < 0) return 1;
	output->z = (int16_t)result;
	result = i2c_smbus_read_byte_data(g_bus, 0x2D);
	if (result < 0) return 1;
	output->z |= (int16_t)result << 8;

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
	output->x |= (int16_t)result << 8;

	result = i2c_smbus_read_byte_data(g_bus, 0x2A);
	if (result < 0) return 1;
	output->y = (int16_t)result;
	result = i2c_smbus_read_byte_data(g_bus, 0x2B);
	if (result < 0) return 1;
	output->y |= (int16_t)result << 8;

	result = i2c_smbus_read_byte_data(g_bus, 0x2C);
	if (result < 0) return 1;
	output->z = (int16_t)result;
	result = i2c_smbus_read_byte_data(g_bus, 0x2D);
	if (result < 0) return 1;
	output->z |= (int16_t)result << 8;

	return 0;
}

static int send_update(int device, int motor, int8_t throttle)
{
	printf("Update: device=%d, motor=%d, throttle=%d\n", device, motor, throttle);
	if ( ioctl(g_bus, I2C_SLAVE, device) < 0 ) {
		return 1;
	}
	return i2c_smbus_write_byte_data(g_bus, motor, throttle);
}

int update_motors(int *settings)
/* settings points to a 4-length array of integers */
{
	send_update( FORWARD_CONTROL, MOTOR_PORT,      (int8_t)settings[0] );
	send_update( FORWARD_CONTROL, MOTOR_STARBOARD, (int8_t)settings[1] );
	send_update( REAR_CONTROL,    MOTOR_PORT,      (int8_t)settings[2] );
	send_update( REAR_CONTROL,    MOTOR_STARBOARD, (int8_t)settings[3] );
	return 0;
}
