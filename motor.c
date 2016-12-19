#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include "boardutil.h"

#define INTERVAL 5

int global_stop = 0;

void handle_signal(int signal)
{
	global_stop = 1;
}

int main(int argc, char **argv)
{
	signal(SIGINT, handle_signal);
	signal(SIGTSTP, handle_signal);

	int motors[4] = {0, 0, 0, 0};
	int up = 1;

	int motor;
	if (argc > 1) {
		motor = strtol(argv[1], NULL, 0);
		if (motor > 3) motor = 3;
		else if (motor < 0) motor = 0;
	}
	else return 1;

	open_bus(DEVICE_FILE);
	update_motors(motors);

	while (!global_stop) {
		int i;

		printf("Running: %d / 200\n", motors[motor]);
		if (up) {
			/* for (i = 0; i < 4; i++) {
				 motors[i] += INTERVAL;
			} */
			motors[motor] += INTERVAL;
			/* if (motors[0] >= 50) {
				up = 0;
			} */
			if (motors[motor] >= 200) {
				up = 0;
			}
		}
		else {
			/* for (i = 0; i < 4; i++) {
				motors[i] -= INTERVAL;
			} */
			motors[motor] -= INTERVAL;
			/* if (motors[0] <= 0) {
				up = 1;
			} */
			if (motors[motor] <= 0) {
				up = 1;
			}
		}

		update_motors(motors);
		sleep(1);
	}
	
	int i;
	for (i = 0; i < 4; i++)
	{
		motors[i] = 0;
	}

	update_motors(motors);

	return 0;
}
