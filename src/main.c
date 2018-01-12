/* Nes Cohen */
/* 6/28/2016 */

#include "nav/navigation.h"
#include "hardware/boardutil.h"
#include "hardware/flight-input.h"
#include "error/error_log.h"
#include "pid/pid.h"
#include "kalman/ukf_mrp.h"
#include "math/matrix_util.h"
#include "math/quaternion_util.h"
#include "client/client.h"

#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <float.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

#define NSEC_TO_SEC 1e-9
#define SEC_TO_NSEC ((long)1e9)

volatile sig_atomic_t stop;
void inthand(int signum)
{
	stop = 1;
}

int main(int argc, char **argv)
{
	// signal handlers to prevent motors from staying on
	signal(SIGINT, inthand);
	signal(SIGTSTP, inthand);

	int user_throttle = -1;
	if (argc == 3) {
		char *end = NULL;
		long input = strtol(argv[2], &end, 10);
		if (end != argv[2] && !strcmp(argv[1], "throttle") && input >= 0 && input <= 200) {
			user_throttle = (int) input;
		}
		else {
			printf("Invalid arguments. Exiting.\n");
			return 1;
		}
	}
	else if (argc != 1) {
		printf("Invalid arguments. Exiting.\n");
		return 1;
	}

	// create and start network handling thread
	pthread_t network_client_thread;
	pthread_create(&network_client_thread, NULL, &network_client_start, NULL);

	//TODO: have the alignment step also run a self-test on the gyro and make corrections
	// Addendum: the filter seems to be performing quite well with out this, it may not be necessary
	//TODO: find the source of the NaNs. In the final program it should be impossible to generate a NaN

	Drone_state drone_state;

	// Navigation initialization
	int error = init_nav(&drone_state);
	if (error) {
		printf("Navigation initialization failed. Exiting.\n");
		return 1;
	}

	//Navigation alignment
	error = align_nav(&drone_state);
	if (error) {
		printf("Navigation alignment failed. Exiting.\n");
		stop_nav();
		return 1;
	}

	struct timespec curr_clock, last_clock;
	if (clock_gettime(CLOCK_REALTIME, &curr_clock) < 0) stop = 1;
	while(!stop) {
		// calculate time elapsed during last loop iteration
		last_clock = curr_clock;
		if (clock_gettime(CLOCK_REALTIME, &curr_clock) < 0) stop = 1;
		double elapsed = (double)(curr_clock.tv_nsec - last_clock.tv_nsec)*NSEC_TO_SEC + (double)(curr_clock.tv_sec - last_clock.tv_sec); 

		Controls controls;
		controls.throttle = (user_throttle == -1) ? 100 : user_throttle;
		controls.roll = 0;
		controls.pitch = 0;

		update_nav(&drone_state, &controls, elapsed);
		
		if (user_throttle >= 0) set_throttle(drone_state.motors);

		//printf("Attitude MRP = [%f, %f, %f] ->\t[%f, %f, %f]^t\t (%f, %f, %f)\r", measurement[6], measurement[7], measurement[8], ukf.state[0], ukf.state[1], ukf.state[2], euler_angles[0], euler_angles[1], euler_angles[2]);
	}

	stop_nav();
	network_client_stop();
	pthread_join(network_client_thread, NULL);
	log_complete();

	return 0;
}
