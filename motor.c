#include <unistd.h>
#include "boardutil.h"

volatile int global_stop = 0;

void handle_signal(int signal)
{
	global_stop = 1;
}

int main()
{
	int motors[4] = {0, 0, 0, 0};
	int up = 5;

	open_bus(DEVICE_FILE);
	update_motors(motors);

	while (!global_stop) {
		int i;

		if (up) {
			for (i = 0; i < 4; i++) {
				 motors[i] += 1;
			}
			if (motors[0] >= 50) {
				up = 0;
			}
		}
		else {
			for (i = 0; i < 4; i++) {
				motors[i] -= 1;
			}
			if (motors[0] <= 0) {
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
