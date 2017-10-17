#include "../../src/hardware/boardutil.h"
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>

int main(int argc, char **argv)
{
	if (argc < 3) {
		printf("Invalid arguments supplied\n");
		return 1;
	}

	int motor = strtol(argv[1], NULL, 0);
	if (errno != 0 && motor == 0) {
		printf("Input error occurred\n");
	}
	int throttle = strtol(argv[2], NULL, 0);
	if (errno != 0 && throttle == 0) {
		printf("Input error occurred\n");
	}

	if (throttle > 200) throttle = 200;
	if (throttle < 0) throttle = 0;
	if (motor > 3) motor = 3;
	if (motor < 0) motor = 0;

	open_bus(DEVICE_FILE);

	int settings[4];
	int i;
	for (i = 0; i < 4; i++) {
		if (i == motor) {
			settings[i] = throttle;
		}
		else settings[i] = 0;
	}
	update_motors(settings);
	settings[motor] = 0;

	getchar();

	update_motors(settings);
	return 0;
}
