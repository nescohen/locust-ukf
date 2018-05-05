/* Nes Cohen */
/* 6/28/2016 */

#include "nav/navigation.h"
#include "hardware/boardutil.h"
#include "error/error_log.h"
#include "pid/pid.h"
#include "kalman/ukf_mrp.h"
#include "math/matrix_util.h"
#include "math/quaternion_util.h"
#include "client/client.h"
#include "stop.h"

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

volatile sig_atomic_t g_signal_stop;

void inthand(int signum)
{
	g_signal_stop = 1;
}

int handle_command(Command *command)
{
	int result = 0;
	Directives directives;
	init_directives(&directives);
	switch(command->type) {
		case NETWORK_THROTTLE:
		{
			int new_throttle = command->value;
			if (new_throttle > 200) new_throttle = 200;
			if (new_throttle < 0) new_throttle = 0;
			directives.throttle = new_throttle;
			nav_set_directives(&directives);
		} break;
		case NETWORK_REPORT:
		{
			// TODO: figure out how to report current position
		} break;
		case NETWORK_OFF:
		{
			directives.stop = 1;
			nav_set_directives(&directives);
			result = 1;
		} break;
		case INTERNAL_STOP:
		{
			result = 1;
		} break;
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

	pthread_t command_thread;
	pthread_create(&command_thread, NULL, &command_listen_main, NULL);

	int signal_watch = 1;
	while (signal_watch) {
		// TODO: check for signal here, figure out how to exit
		if (g_signal_stop == 1) {
			signal_watch = 0;
			set_global_stop();
		}
		sleep(1);
	}

	pthread_join(command_thread, NULL);

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
