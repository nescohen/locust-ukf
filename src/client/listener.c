/* Nes Cohen */
/* 5/11/2018 */

#include "listener.h"
#include "client.h"
#include "../nav/navigation.h"
#include "../stop/stop.h"

#include <stdlib.h>
#include <pthread.h>

static int handle_command(Command *command)
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
			set_global_stop();
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
