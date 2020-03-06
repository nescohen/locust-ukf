#include "navigation.h"
#include "../hardware/boardutil.h"
#include "../error/error_log.h"
#include "../pid/pid.h"
#include "../math/matrix_util.h"
#include "../math/quaternion_util.h"
#include "../kalman/kalman.h"
#include "../kalman/ukf_mrp.h"
#include "../stop/stop.h"

#include <stdio.h>
#include <string.h>
#include <float.h>
#include <time.h>
#include <math.h>
#include <pthread.h>

#define GRAVITY 			(9.81f)
#define EPSILON 			(1.0f)

#define GYRO_SENSATIVITY 	(4.36332f)
#define ACCL_SENSATIVITY 	(2.f * GRAVITY) // 2gs max converted to ms^-2
#define GYRO_VARIANCE 		(0.193825) // 11.111... degress in radians
#define ACCL_VARIANCE 		(1.1772)
#define COMP_VARIANCE 		(1.1772)

#define ALIGN_TIME 			(2.5) // in seconds
#define SIGNED_16_MAX 		(0x7FFF)
#define PID_SCALE 			(30) // 100 / 2 * M_PI
#define NSEC_TO_SEC 		(1e-9)
#define SEC_TO_NSEC 		((long)1e9)

#define NAV_PRINT

typedef struct controls
{
	int throttle;
	int roll; 
	int pitch;
} Controls;

typedef struct drone_state
{
	int motors[4];
	Pidhist hist_x;
	Pidhist hist_y;
	Ukf_parameters ukf_param;
} Drone_state;

// TODO: make this more modular, referenced from elsewhere
// NOTE: do not make static until this is fixed
double g_north[3];
double g_down[3];

static pthread_t sensor_poll_thread;
static pthread_t motor_update_thread;
static int g_thread_return;

static pthread_mutex_t directives_lock = PTHREAD_MUTEX_INITIALIZER;
static Directives g_directives;
static pthread_mutex_t report_lock = PTHREAD_MUTEX_INITIALIZER;
static Report g_report;

void init_directives(Directives *directives)
{
	directives->throttle = 0;
	directives->next_action = ACTION_NOACTION;
}

void nav_set_directives(Directives *directives)
{
	pthread_mutex_lock(&directives_lock);
	memcpy(&g_directives, directives, sizeof(Directives));
	pthread_mutex_unlock(&directives_lock);
}

void init_report(Report *report)
{
	report->curr_state = STATE_UNINITIALIZED;
	memset(report->ukf_state, 0, sizeof(report->ukf_state));
}

void nav_get_report(Report *writeback)
// warning: blocking
{
	pthread_mutex_lock(&report_lock);
	memcpy(writeback, &g_report, sizeof(Report));
	pthread_mutex_unlock(&report_lock);
}

static int set_report(const Report *report)
// WARNING: blocking
{
	int fail = pthread_mutex_trylock(&report_lock);
	if (fail) {
		return 1;
	}
	else {
		memcpy(&g_report, report, sizeof(Report));
		pthread_mutex_unlock(&report_lock);
		return 0;
	}
}

static int get_directives_nonblock(Directives *writeback)
{
	if (!pthread_mutex_trylock(&directives_lock)) {
		memcpy(writeback, &g_directives, sizeof(Directives));
		pthread_mutex_unlock(&directives_lock);
		return 0;
	}
	else return 1;
}

// static void get_directives_block(Directives *writeback)
// // warning: blocking
// {
// 	pthread_mutex_lock(&directives_lock);
// 	memcpy(writeback, &g_directives, sizeof(Directives));
// 	pthread_mutex_unlock(&directives_lock);
// }

void mrp_to_euler(double *euler_angles, double *mrp)
//expects 3x1 array to write and 3x1 mrp, returns equivelent euler_angle representation from mrp
{
	double angle;
	double axis[3];
	double matrix[9];
	double result[3];
	
	// First convert the mrp into a rotation matrix
	angle = 4*atan(vector_magnitude(mrp));
	normalize_vector(mrp, axis);
	axis_angle_matrix(axis, angle, matrix);

	// Get relevant information from rotation matrix
	result[0] = atan2(matrix[3*1 + 0], matrix[3*0 + 0]);
	result[1] = atan2(-1*matrix[3*2 + 0], sqrt(pow(matrix[3*2 + 1], 2) + pow(matrix[3*2 + 2], 2)));
	result[2] = atan2(matrix[3*2 + 1], matrix[3*2 + 2]);

	memcpy(euler_angles, result, sizeof(result));
}

int detect_nans(double *array, int size)
{
	int i;
	int nan = 0;
	for (i = 0; i < size; i++) {
		if (array[i] != array[i]) nan = 1;
	}
	return nan;
}

void recovery_pid(double x, double y, Controls *controls, int *motors, Pidhist *hist_x, Pidhist *hist_y, double delta_t)
{
	double error_x = x - controls->roll;
	double error_y = y - controls->pitch;

	double correct_x = PID_SCALE*pid(hist_x, error_x, delta_t);
	double correct_y = PID_SCALE*pid(hist_y, error_y, delta_t);

	motors[0] = controls->throttle + (int)(0.5*correct_x) + (int)(0.5*correct_y);
	motors[1] = controls->throttle + (int)(0.5*correct_x) + (int)(-0.5*correct_y);
	motors[2] = controls->throttle + (int)(-0.5*correct_x) + (int)(0.5*correct_y);
	motors[3] = controls->throttle + (int)(-0.5*correct_x) + (int)(-0.5*correct_y);

	int i;
	for (i = 0; i < 4; i++) {
		if (motors[i] < 0) {
			motors[i] = 0;
		}
		if (motors[i] > 200) {
			motors[i] = 200;
		}
	}

#ifdef PID_PRINT
	static int cap = 0;
	if (cap > 1000) {
		printf("[%f, %f] => [%f, %f] => [%d, %d, %d, %d]\n", error_x, error_y, correct_x, correct_y, motors[0], motors[1], motors[2], motors[3]);
		cap = 0;
	}
	else {
		cap += 1;
	}
#endif
}

void sensor_to_array(double array[3], Vector3 sensor, double sensativity)
{
	array[0] = (double)sensor.x / (double)SIGNED_16_MAX * sensativity;
	array[1] = (double)sensor.y / (double)SIGNED_16_MAX * sensativity;
	array[2] = (double)sensor.z / (double)SIGNED_16_MAX * sensativity;
}

void init_drone_state(Drone_state *state)
{
	int i;
	for (i = 0; i < 4; i++) {
		state->motors[i] = 0;
	}
	ukf_param_init(&(state->ukf_param));
	init_hist(&(state->hist_x));
	init_hist(&(state->hist_y));
}

int start_sensors()
{
	if (open_bus(DEVICE_FILE) != 0) {
		log_error("Failed to open i2c device file.");
		return 1;
	}

	int failure = 0;
	failure = failure || gyro_power_on() < 0;
	failure = failure || accl_power_on() < 0;
	failure = failure || comp_power_on() < 0;
	if (failure) {
		log_error("Failed to power on one or more sensors.");
		return 2;
	}

	return 0;
}

int init_nav(Drone_state *state)
// start nav system not including alignment, initialize state
// returns 0 on success, non-zero value indicates failure
{
	// turn on sensors
	int result = start_sensors();
	if (result != 0) {
		return 1;
	}

	// create and start sensor polling thread
	pthread_create(&sensor_poll_thread, NULL, &poll_loop, NULL);
	// create and start motor controlling thread
	pthread_create(&motor_update_thread, NULL, &motor_loop, NULL);

	init_drone_state(state);
	pthread_mutex_lock(&directives_lock);
	init_directives(&g_directives);
	pthread_mutex_unlock(&directives_lock);
	pthread_mutex_lock(&report_lock);
	init_report(&g_report);
	g_report.curr_state = STATE_INITIALIZED;
	pthread_mutex_unlock(&report_lock);

	return 0;
}

int align_nav(Drone_state *state)
// align state with data from sensors
// returns 0 on success, non-zero value indicates failure
{
	int align = 1;
	double align_mean[SIZE_MEASUREMENT];
	double align_var[SIZE_MEASUREMENT];
	double weight_sum = 0;

	struct timespec curr_clock, last_clock;
	if (clock_gettime(CLOCK_REALTIME, &curr_clock) < 0) {
		log_error("Failed to get system time during alignment");
		return 1;
	}
	while (align == 1) {
		// calculate time elapsed during last loop iteration
		last_clock = curr_clock;
		if (clock_gettime(CLOCK_REALTIME, &curr_clock) < 0) {
			log_error("Failed to get system time during alignment");
			return 1;
		}
		double elapsed = (double)(curr_clock.tv_nsec - last_clock.tv_nsec)*NSEC_TO_SEC + (double)(curr_clock.tv_sec - last_clock.tv_sec); 

		// get data from sensors
		Vector3 gyro;
		Vector3 accel;
		Vector3 north;
		double measurement[SIZE_MEASUREMENT];
		get_sensor_data(&gyro, &accel);
		get_compass_data(&north);
		sensor_to_array(measurement, accel, ACCL_SENSATIVITY);
		sensor_to_array(measurement + 3, north, 1);
		sensor_to_array(measurement + 6, gyro, GYRO_SENSATIVITY);

		// calculate mean and variance incrementally
		int i;
		for (i = 0; i < SIZE_MEASUREMENT; i++) {
			double old_mean = align_mean[i];
			align_mean[i] = (align_mean[i]*weight_sum + measurement[i]*elapsed) / (weight_sum + elapsed);
			align_var[i] += elapsed * (measurement[i] - old_mean) * (measurement[i] - align_mean[i]);
		}
		weight_sum += elapsed;
		if(weight_sum >= ALIGN_TIME) align = 0;
	}
	matrix_scale(align_var, align_var, 1 / weight_sum, SIZE_MEASUREMENT, 1);
	memcpy(g_down, align_mean, sizeof(g_down));
	memcpy(g_north, align_mean + 3, sizeof(g_north));

	memset(state->ukf_param.state, 0, 3*sizeof(double));
	memcpy(state->ukf_param.state + 3, align_mean + 6, 3*sizeof(double));

#ifdef NAV_PRINT
	printf("Align State:\n");
	matrix_quick_print(state->ukf_param.covariance, SIZE_STATE, SIZE_STATE);
#endif

	return 0;
}

int update_nav(Drone_state *state, Controls *controls, double delta_t)
// read from sensors, perform single update of kalman filter and pid
// returns 0 on success, non-zero value indicates failure
{
	double measurement[SIZE_MEASUREMENT];
	Vector3 gyro;
	Vector3 accel;
	Vector3 north;

	get_sensor_data(&gyro, &accel);
	get_compass_data(&north);

	double accel_var[3];
	double comp_var[3];
	double gyro_var[3];
	double acceleration[3];
	sensor_to_array(acceleration, accel, ACCL_SENSATIVITY);
	double accel_mag = vector_magnitude(acceleration);
	if (accel_mag + EPSILON > GRAVITY && accel_mag - EPSILON < GRAVITY) {
		normalize_vector(acceleration, measurement + 0);
		matrix_init(accel_var, ACCL_VARIANCE, 3, 1);
	}
	else {
		matrix_init(accel_var, DBL_MAX, 3, 1);
	}
	sensor_to_array(measurement + 3, north, 1);
	normalize_vector(measurement + 3, measurement + 3);
	matrix_init(comp_var, COMP_VARIANCE, 3, 1);
	sensor_to_array(measurement + 6, gyro, GYRO_SENSATIVITY);
	matrix_init(gyro_var, GYRO_VARIANCE, 3, 1);

	int i;
	for (i = 0; i < 3; i++) {
		state->ukf_param.R[i*SIZE_MEASUREMENT + i] = accel_var[i];
	}
	for (i = 3; i < 6; i++) {
		state->ukf_param.R[i*SIZE_MEASUREMENT + i] = comp_var[i - 3];
	}
	for (i = 6; i < 9; i++) {
		state->ukf_param.R[i*SIZE_MEASUREMENT + i] = gyro_var[i - 6];
	}

	ukf_run(&(state->ukf_param), measurement, delta_t);

	// convert mrp to euler angles for pid
	double euler_angles[3];
	mrp_to_euler(euler_angles, state->ukf_param.state);

	if (detect_nans(measurement, SIZE_MEASUREMENT) ||
		detect_nans(euler_angles, 3))
	{
		printf("NAN detected\n");
		return 1;
	}

	recovery_pid(euler_angles[2], euler_angles[1], controls, state->motors, &(state->hist_x), &(state->hist_y), delta_t);

	return 0;
}

void stop_nav()
// shuts down sensors and the sensor and motor threads
{
	stop_hardware_loop();
	pthread_join(sensor_poll_thread, NULL);
	gyro_power_off();
	accl_power_off();
	comp_power_off();
	pthread_join(motor_update_thread, NULL);
}

//TODO: have the alignment step also run a self-test on the gyro and make corrections
// Addendum: the filter seems to be performing quite well with out this, it may not be necessary
//TODO: find the source of the NaNs. In the final program it should be impossible to generate a NaN

static void nav_cycle(Drone_state *state, Directives *directives, double elapsed)
{
	Controls controls;
	get_directives_nonblock(directives);
	controls.throttle = directives->throttle;
	controls.roll = 0;
	controls.pitch = 0;

	update_nav(state, &controls, elapsed);

	set_throttle(state->motors);
}

void *navigation_main(void *arg)
// main navigation loop, running on navigation thread
{
	Drone_state drone_state;

	int nav_state = STATE_UNINITIALIZED;

	Report report;
	init_report(&report);
	set_report(&report);

	Directives directives;
	init_directives(&directives);

	int stop = 0;
	struct timespec curr_clock, last_clock;
	if (clock_gettime(CLOCK_REALTIME, &curr_clock) < 0) stop = 1;
	while(!stop) {
		// calculate time elapsed during last loop iteration
		last_clock = curr_clock;
		if (clock_gettime(CLOCK_REALTIME, &curr_clock) < 0) stop = 1;
		double elapsed = (double)(curr_clock.tv_nsec - last_clock.tv_nsec)*NSEC_TO_SEC + (double)(curr_clock.tv_sec - last_clock.tv_sec); 

		switch(nav_state) {
			case STATE_UNINITIALIZED: {
				// Check for relavent action
				int stale = get_directives_nonblock(&directives);
				if (!stale && directives.next_action == ACTION_INIT) {
					// Navigation initialization
					int error = init_nav(&drone_state);
					if (error) {
						printf("Navigation initialization failed. Exiting.\n");
						g_thread_return = 1;
						pthread_exit(&g_thread_return);
					}
					nav_state = STATE_INITIALIZED;
					report.curr_state = STATE_INITIALIZED;
					set_report(&report);
				}
			} break;
			case STATE_INITIALIZED: {
				// Check for relavent action
				int stale = get_directives_nonblock(&directives);
				if (!stale && directives.next_action == ACTION_ALIGN) {
					// Navigation alignment
					int error = align_nav(&drone_state);
					if (error) {
						printf("Navigation alignment failed. Exiting.\n");
						stop_nav();
						g_thread_return = 1;
						pthread_exit(&g_thread_return);
					}
					nav_state = STATE_ALIGNED;
					report.curr_state = STATE_ALIGNED;
					memcpy(&report.ukf_state, &drone_state.ukf_param.state, sizeof(report.ukf_state));
					set_report(&report);
				}
			} break;
			case STATE_ALIGNED: {
				// Check for relavent action
				int stale = get_directives_nonblock(&directives);
				if (!stale && directives.next_action == ACTION_START) {
					nav_state = STATE_RUNNING;
					report.curr_state = STATE_ALIGNED;
					set_report(&report);
				}
			} break;
			case STATE_RUNNING: {
				nav_cycle(&drone_state, &directives, elapsed);
			} break;
			default: {
				//TODO: implement error handling here: fsm corruption?
			} break;
		}

		if (check_global_stop()) {
			stop = 1;
		}
	}

	stop_nav();
	g_thread_return = 0;
	pthread_exit(&g_thread_return);
}
