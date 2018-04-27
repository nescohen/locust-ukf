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

#define NSEC_TO_SEC ((double)1e-9)
#define SEC_TO_NSEC ((long)1e9)

void inthand(int signum)
{
	//TODO: set nav controls to indicate it to stop
}

int handle_command(Command *command)
{
	int result = 0;
	switch(command->type) {
		case NETWORK_THROTTLE:
			break;
		case NETWORK_REPORT:
			break;
		case NETWORK_OFF:
			// potentially do other things
			result = 1;
			break;
	}

	return result;
}

void *command_listen_main(void *arg)
{
	Command curr;

	int stop = 0;
	while (!stop) {
		get_command(&curr); // this will block until a command is actually available
		stop = handle_command(&curr);
	}
	pthread_exit(NULL);
}

int main(int argc, char **argv)
{
	signal(SIGINT, inthand);
	signal(SIGTSTP, inthand);

	// create and start network handling thread
	int net_success = network_client_init();
	if (!net_success) {
		return 1;
	}
	pthread_t network_client_thread;
	pthread_create(&network_client_thread, NULL, &network_client_start, NULL);

	pthread_t navigation_thread;
	pthread_create(&navigation_thread, NULL, &navigation_main, NULL);
	void *retval;
	pthread_join(navigation_thread, &retval);
	if (*((int *)retval) != 0) {
		printf("Error occurred during navigation thread execution\n");
	}

	network_client_stop();
	pthread_join(network_client_thread, NULL);
	log_complete();

	return 0;
}
