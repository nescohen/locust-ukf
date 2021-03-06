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
#include "client/listener.h"
#include "stop/stop.h"

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
		if (check_global_stop()) {
			signal_watch = 0;
		}
		else if (g_signal_stop == 1) {
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
