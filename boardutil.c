/* Nes Cohen */
/* 6/28/2016 */

#include "boardutil.h"
#include "error_log.h"
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>

#define GYRO_ADDR 0x69
#define ACCL_ADDR 0x19
#define COMP_ADDR 0x1E
#define GYRO_POWER_ON 0x0F
#define GYRO_POWER_OFF 0x00
#define GYRO_HIGH_PASS_SET 0x23
#define GYRO_HIGH_PASS_ON 0x10
#define GYRO_FULL_SCALE 0x00
#define ACCL_POWER_ON 0x57
#define ACCL_POWER_OFF 0x07
#define ACCL_HIGH_REZ 0x08
#define COMP_POWER_ON 0x00
#define COMP_POWER_OFF 0x03
#define COMP_SET_GAIN 0x80

#define FORWARD_CONTROL 0x44
#define REAR_CONTROL 0x45
#define MOTOR_PORT 0x01
#define MOTOR_STARBOARD 0x02

#define STATUS_NEW_BIT_X 0
#define STATUS_NEW_BIT_Y 1
#define STATUS_NEW_BIT_Z 2
#define STATUS_NEW_BIT_ALL 3

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
	
	error = i2c_smbus_write_byte_data(g_bus, 0x21, GYRO_HIGH_PASS_SET);
	if (error < 0) return 1;
	error = i2c_smbus_write_byte_data(g_bus, 0x24, GYRO_HIGH_PASS_ON);
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
		return 1;
	}
	result = i2c_smbus_write_byte_data(g_bus, 0x23, ACCL_HIGH_REZ);
	if (result < 0) {
		return 3;
	}
	return 0;
}

int comp_power_on()
{
	if (ioctl(g_bus, I2C_SLAVE, COMP_ADDR) < 0) {
		return 2;
	}
	
	int32_t result;
	result = i2c_smbus_write_byte_data(g_bus, 0x02, COMP_POWER_ON);
	if (result < 0) {
		return 1;
	}
	result = i2c_smbus_write_byte_data(g_bus, 0x01, COMP_SET_GAIN);
	if (result < 0) {
		return 1;
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

int comp_power_off()
{
	if (ioctl(g_bus, I2C_SLAVE, COMP_ADDR) < 0) {
		return 2;
	}
	
	int32_t result;
	result = i2c_smbus_write_byte_data(g_bus, 0x02, COMP_POWER_OFF);
	if (result < 0) {
		return 1;
	}
	return 0;
}

int gyro_poll(Vector3 *output)
// only updates value if new data is available
{
	if (ioctl(g_bus, I2C_SLAVE, GYRO_ADDR) < 0) {
		return 2;
	}
	int32_t result;
	int32_t status;

	status = i2c_smbus_read_byte_data(g_bus, 0x27);
	if (status & (1 << STATUS_NEW_BIT_ALL))
	{
		if (status & (1 << STATUS_NEW_BIT_X))
		{
			result = i2c_smbus_read_byte_data(g_bus, 0x28);
			if (result < 0) return 1;
			output->x = (int16_t)result;
			result = i2c_smbus_read_byte_data(g_bus, 0x29);
			if (result < 0) return 1;
			output->x |= (int16_t)result << 8;
		}

		if (status & (1 << STATUS_NEW_BIT_Y))
		{
			result = i2c_smbus_read_byte_data(g_bus, 0x2A);
			if (result < 0) return 1;
			output->y = (int16_t)result;
			result = i2c_smbus_read_byte_data(g_bus, 0x2B);
			if (result < 0) return 1;
			output->y |= (int16_t)result << 8;
		}

		if (status & (1 << STATUS_NEW_BIT_Z))
		{
			result = i2c_smbus_read_byte_data(g_bus, 0x2C);
			if (result < 0) return 1;
			output->z = (int16_t)result;
			result = i2c_smbus_read_byte_data(g_bus, 0x2D);
			if (result < 0) return 1;
			output->z |= (int16_t)result << 8;
		}
		return 1;
	}

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

int comp_poll(Vector3 *output)
{
	
	if (ioctl(g_bus, I2C_SLAVE, COMP_ADDR) < 0) {
		return 2;
	}
	int32_t result;

	result = i2c_smbus_read_byte_data(g_bus, 0x04);
	if (result < 0) return 1;
	output->x = (int16_t)result;
	result = i2c_smbus_read_byte_data(g_bus, 0x03);
	if (result < 0) return 1;
	output->x |= (int16_t)result << 8;

	result = i2c_smbus_read_byte_data(g_bus, 0x06);
	if (result < 0) return 1;
	output->y = (int16_t)result;
	result = i2c_smbus_read_byte_data(g_bus, 0x05);
	if (result < 0) return 1;
	output->y |= (int16_t)result << 8;

	result = i2c_smbus_read_byte_data(g_bus, 0x08);
	if (result < 0) return 1;
	output->z = (int16_t)result;
	result = i2c_smbus_read_byte_data(g_bus, 0x07);
	if (result < 0) return 1;
	output->z |= (int16_t)result << 8;

	return 0;
}

static int send_update(int device, int motor, int8_t throttle)
{
	int error;

	error = ioctl(g_bus, I2C_SLAVE, device);
	if ( error < 0 ) {
		return errno;
	}
	return i2c_smbus_write_byte_data(g_bus, motor, throttle);
}

int update_motors(int *settings)
/* settings points to a 4-length array of integers */
{
	char buffer[100];
	int err[4];
	int i;

	err[0] = send_update( FORWARD_CONTROL, MOTOR_PORT,      (int8_t)settings[0] );
	err[1] = send_update( FORWARD_CONTROL, MOTOR_STARBOARD, (int8_t)settings[1] );
	err[2] = send_update( REAR_CONTROL,    MOTOR_PORT,      (int8_t)settings[2] );
	err[3] = send_update( REAR_CONTROL,    MOTOR_STARBOARD, (int8_t)settings[3] );

	for (i = 0; i < 4; i++) {
		snprintf(buffer, 100, "motor=%d throttle=%d error=%d", i, settings[i], err[i]);
		log_error(buffer);
	}
	return 0;
}
