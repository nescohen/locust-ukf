/* Nes Cohen */
/* 6/28/2016 */

#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include "boardutil.h"
#include "../error/error_log.h"

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
static pthread_mutex_t bus_lock = (pthread_mutex_t) PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t output_lock = (pthread_mutex_t) PTHREAD_MUTEX_INITIALIZER;

static pthread_mutex_t motor_lock = (pthread_mutex_t) PTHREAD_MUTEX_INITIALIZER;
static volatile int curr_throttle[4] = {0, 0, 0, 0};

static Vector3 curr_gyro;
static Vector3 curr_accel;
static Vector3 curr_compass;
static int curr_gyro_count;

static volatile sig_atomic_t stop = 0;

int open_bus(char *filename)
// must be called before multi-threading starts
{
	pthread_mutex_lock(&bus_lock);

	int ref = open(filename, O_RDWR);
	if (ref < 0) {
		//printf("%d\n", errno);
		pthread_mutex_unlock(&bus_lock);
		return 1;
	}
	g_bus = ref;

	pthread_mutex_unlock(&bus_lock);
	return 0;
}

int gyro_power_on()
{
	// TODO: try powering the gyro in self test mode and make corrections

	pthread_mutex_lock(&bus_lock);
	if (ioctl(g_bus, I2C_SLAVE, GYRO_ADDR) < 0) {
		pthread_mutex_unlock(&bus_lock);
		return -2;
	}
	int result = 0;
	int32_t error = i2c_smbus_write_byte_data(g_bus, 0x20, GYRO_POWER_ON);
	if (error < 0) result = -1;
	error = i2c_smbus_write_byte_data(g_bus, 0x23, GYRO_FULL_SCALE);
	if (error < 0) result = -1;
	
	error = i2c_smbus_write_byte_data(g_bus, 0x21, GYRO_HIGH_PASS_SET);
	if (error < 0) result = -1;
	error = i2c_smbus_write_byte_data(g_bus, 0x24, GYRO_HIGH_PASS_ON);
	if (error < 0) result = -1;

	pthread_mutex_unlock(&bus_lock);
	return result;
}

int accl_power_on()
{
	pthread_mutex_lock(&bus_lock);
	if (ioctl(g_bus, I2C_SLAVE, ACCL_ADDR) < 0) {
		/* some sort of error */
		pthread_mutex_unlock(&bus_lock);
		return -2;
	}
	
	int32_t result;
	result = i2c_smbus_write_byte_data(g_bus, 0x20, ACCL_POWER_ON);
	if (result < 0) {
		pthread_mutex_unlock(&bus_lock);
		return -1;
	}
	result = i2c_smbus_write_byte_data(g_bus, 0x23, ACCL_HIGH_REZ);

	pthread_mutex_unlock(&bus_lock);
	if (result < 0) {
		return -3;
	}
	return 0;
}

int comp_power_on()
{
	pthread_mutex_lock(&bus_lock);
	if (ioctl(g_bus, I2C_SLAVE, COMP_ADDR) < 0) {
		pthread_mutex_unlock(&bus_lock);
		return -2;
	}
	
	int32_t result;
	result = i2c_smbus_write_byte_data(g_bus, 0x02, COMP_POWER_ON);
	if (result < 0) {
		pthread_mutex_unlock(&bus_lock);
		return -1;
	}
	result = i2c_smbus_write_byte_data(g_bus, 0x01, COMP_SET_GAIN);

	pthread_mutex_unlock(&bus_lock);
	if (result < 0) {
		return -1;
	}
	return 0;
}

int gyro_power_off()
{
	pthread_mutex_lock(&bus_lock);
	if (ioctl(g_bus, I2C_SLAVE, GYRO_ADDR) < 0) {
		pthread_mutex_unlock(&bus_lock);
		return -2;
	}
	int32_t error = i2c_smbus_write_byte_data(g_bus, 0x20, GYRO_POWER_OFF);

	pthread_mutex_unlock(&bus_lock);
	if (error < 0) return -1;
	return 0;
}

int accl_power_off()
{
	pthread_mutex_lock(&bus_lock);
	if (ioctl(g_bus, I2C_SLAVE, ACCL_ADDR) < 0) {
		/* some sort of error */
		pthread_mutex_unlock(&bus_lock);
		return -2;
	}

	int32_t result;
	int return_val;
	result = i2c_smbus_write_byte_data(g_bus, 0x20, ACCL_POWER_OFF);
	if (result < 0) {
		return_val = -1;
	}
	else {
		return_val = 0;
	}
	pthread_mutex_unlock(&bus_lock);
	return return_val;
}

int comp_power_off()
{
	pthread_mutex_lock(&bus_lock);
	if (ioctl(g_bus, I2C_SLAVE, COMP_ADDR) < 0) {
		pthread_mutex_unlock(&bus_lock);
		return -2;
	}
	
	int32_t result;
	int return_val;
	result = i2c_smbus_write_byte_data(g_bus, 0x02, COMP_POWER_OFF);
	if (result < 0) {
		return_val = -1;
	}
	else {
		return_val = 0;
	}

	pthread_mutex_unlock(&bus_lock);
	return return_val;
}

int gyro_poll(Vector3 *output)
// only updates value if new data is available
{
	pthread_mutex_lock(&bus_lock);
	if (ioctl(g_bus, I2C_SLAVE, GYRO_ADDR) < 0) {
		pthread_mutex_unlock(&bus_lock);
		return -1;
	}
	int32_t result;
	int32_t status;
	int error = 0;

	int16_t read_x;
	int16_t read_y;
	int16_t read_z;

	int count = 0;
	status = i2c_smbus_read_byte_data(g_bus, 0x27);
	if (status & (1 << STATUS_NEW_BIT_ALL)) {
		output->x = 0;
		output->y = 0;
		output->z = 0;
	}
	while (status & (1 << STATUS_NEW_BIT_ALL)) {
		result = i2c_smbus_read_byte_data(g_bus, 0x28);
		if (result < 0) error = -1;
		read_x = (int16_t)result;
		result = i2c_smbus_read_byte_data(g_bus, 0x29);
		if (result < 0) error = -1;
		read_x |= (int16_t)result << 8;

		result = i2c_smbus_read_byte_data(g_bus, 0x2A);
		if (result < 0) error = -1;
		read_y = (int16_t)result;
		result = i2c_smbus_read_byte_data(g_bus, 0x2B);
		if (result < 0) error = -1;
		read_y |= (int16_t)result << 8;
		
		result = i2c_smbus_read_byte_data(g_bus, 0x2C);
		if (result < 0) error = -1;
		read_z = (int16_t)result;
		result = i2c_smbus_read_byte_data(g_bus, 0x2D);
		if (result < 0) error = -1;
		read_z |= (int16_t)result << 8;

		output->x = (count*output->x + read_x) / (count + 1);
		output->y = (count*output->y + read_y) / (count + 1);
		output->z = (count*output->z + read_z) / (count + 1);
		status = i2c_smbus_read_byte_data(g_bus, 0x27);
		count++;
	}

	pthread_mutex_unlock(&bus_lock);
	if (error == 0) return count;
	else return error;
}

int accl_poll(Vector3 *output)
{
	pthread_mutex_lock(&bus_lock);
	if (ioctl(g_bus, I2C_SLAVE, ACCL_ADDR) < 0) {
		/* some sort of error */
		pthread_mutex_unlock(&bus_lock);
		return -2;
	}
	int32_t result;
	int error = 0;

	result = i2c_smbus_read_byte_data(g_bus, 0x28);
	if (result < 0) error = result;
	output->x = (int16_t)result;
	result = i2c_smbus_read_byte_data(g_bus, 0x29);
	if (result < 0) error = result;
	output->x |= (int16_t)result << 8;

	result = i2c_smbus_read_byte_data(g_bus, 0x2A);
	if (result < 0) error = result;
	output->y = (int16_t)result;
	result = i2c_smbus_read_byte_data(g_bus, 0x2B);
	if (result < 0) error = result;
	output->y |= (int16_t)result << 8;

	result = i2c_smbus_read_byte_data(g_bus, 0x2C);
	if (result < 0) error = result;
	output->z = (int16_t)result;
	result = i2c_smbus_read_byte_data(g_bus, 0x2D);
	if (result < 0) error = result;
	output->z |= (int16_t)result << 8;

	pthread_mutex_unlock(&bus_lock);
	return error;
}

int comp_poll(Vector3 *output)
{
	pthread_mutex_lock(&bus_lock);
	if (ioctl(g_bus, I2C_SLAVE, COMP_ADDR) < 0) {
		pthread_mutex_unlock(&bus_lock);
		return -2;
	}
	int32_t result;
	int error = 0;

	result = i2c_smbus_read_byte_data(g_bus, 0x04);
	if (result < 0) error = -1;
	output->x = (int16_t)result;
	result = i2c_smbus_read_byte_data(g_bus, 0x03);
	if (result < 0) error = -1;
	output->x |= (int16_t)result << 8;

	result = i2c_smbus_read_byte_data(g_bus, 0x06);
	if (result < 0) error = -1;
	output->y = (int16_t)result;
	result = i2c_smbus_read_byte_data(g_bus, 0x05);
	if (result < 0) error = -1;
	output->y |= (int16_t)result << 8;

	result = i2c_smbus_read_byte_data(g_bus, 0x08);
	if (result < 0) error = -1;
	output->z = (int16_t)result;
	result = i2c_smbus_read_byte_data(g_bus, 0x07);
	if (result < 0) error = -1;
	output->z |= (int16_t)result << 8;

	pthread_mutex_unlock(&bus_lock);
	return error;
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

void get_sensor_data(Vector3 *gyro, Vector3 *accel)
{
	pthread_mutex_lock(&output_lock);

	if (curr_gyro_count > 0) {
		memcpy(gyro, &curr_gyro, sizeof(Vector3));
		curr_gyro_count = 0;
	}
	else memset(gyro, 0, sizeof(Vector3));
	memcpy(accel, &curr_accel, sizeof(Vector3));

	pthread_mutex_unlock(&output_lock);
}

void get_compass_data(Vector3 *compass)
{
	pthread_mutex_lock(&output_lock);
	memcpy(compass, &curr_compass, sizeof(Vector3));
	pthread_mutex_unlock(&output_lock);
}

void stop_hardware_loop()
{
	stop = 1;
}

void *poll_loop(void *arg)
// Starting point for sensor polling thread
{
	pthread_mutex_lock(&output_lock);
	curr_gyro_count = 0;
	memset(&curr_gyro, 0, sizeof(Vector3));
	memset(&curr_accel, 0, sizeof(Vector3));
	memset(&curr_compass, 0, sizeof(Vector3));
	pthread_mutex_unlock(&output_lock);

	while (!stop) {
		int error;
		int gyro_count = 0;
		Vector3 temp_gyro;
		Vector3 temp_accel;
		Vector3 temp_compass;

		char error_buffer[30];
		error = accl_poll(&temp_accel);
		if (error < 0) {
			snprintf(error_buffer, 30*sizeof(char), "Error \"%d\" reading from accelerometer'", error);
			log_error(error_buffer);
		}
		error = comp_poll(&temp_compass);
		if (error < 0) {
			snprintf(error_buffer, 30*sizeof(char), "Error \"%d\" reading from compass'", error);
			log_error(error_buffer);
		}
		error = gyro_poll(&temp_gyro);
		if (error < 0) {
			snprintf(error_buffer, 30*sizeof(char), "Error \"%d\" reading from gyroscope'", error);
			log_error(error_buffer);
		}
		else gyro_count = error;

		pthread_mutex_lock(&output_lock);

		memcpy(&curr_accel, &temp_accel, sizeof(Vector3));
		memcpy(&curr_compass, &temp_compass, sizeof(Vector3));

		int gyro_total = gyro_count + curr_gyro_count;
		if (gyro_total > 0) {
			curr_gyro.x = (temp_gyro.x*gyro_count + curr_gyro.x*curr_gyro_count) / gyro_total;
			curr_gyro.y = (temp_gyro.y*gyro_count + curr_gyro.y*curr_gyro_count) / gyro_total;
			curr_gyro.z = (temp_gyro.z*gyro_count + curr_gyro.z*curr_gyro_count) / gyro_total;
			curr_gyro_count = gyro_total;
		}

		pthread_mutex_unlock(&output_lock);
	}

	return NULL;
}

void set_throttle(int *settings)
{
	pthread_mutex_lock(&motor_lock);
	int i;
	for (i = 0; i < 4; i++) {
		curr_throttle[i] = settings[i];
	}
	pthread_mutex_unlock(&motor_lock);
}

static int update_motors(int *settings)
/* settings points to a 4-length array of integers */
{
	//char buffer[100];
	//int err[4];

	pthread_mutex_lock(&bus_lock);
	/* err[0] = */ send_update( FORWARD_CONTROL, MOTOR_PORT,      (int8_t)settings[0] );
	/* err[1] = */ send_update( FORWARD_CONTROL, MOTOR_STARBOARD, (int8_t)settings[1] );
	/* err[2] = */ send_update( REAR_CONTROL,    MOTOR_PORT,      (int8_t)settings[2] );
	/* err[3] = */ send_update( REAR_CONTROL,    MOTOR_STARBOARD, (int8_t)settings[3] );
	pthread_mutex_unlock(&bus_lock);

	//int i;
	// for (i = 0; i < 4; i++) {
	// 	snprintf(buffer, 100, "motor=%d throttle=%d error=%d", i, settings[i], err[i]);
	// 	log_error(buffer);
	// }
	return 0;
}

void *motor_loop(void *arg)
{
	//update the motors from the most recent motor position

	int motors[4];
	while(!stop) {
		pthread_mutex_lock(&motor_lock);
		int i;
		for (i = 0; i < 4; i++) {
			motors[i] = curr_throttle[i];
		}
		pthread_mutex_unlock(&motor_lock);
		update_motors(motors);
	}

	int i;
	for (i = 0; i < 4; i++) {
		motors[i] = 0;
	}
	update_motors(motors);

	return NULL;
}
