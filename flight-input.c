#include "flight-input.h"
#include <stdio.h>
#include <pthread.h>
#include <string.h>

Controls g_controls;
pthread_mutex_t g_control_lock;

void *start_inout()
{
	char buffer[100];
	char command[20];
	char arg[20];

	g_controls.throttle = 0;
	pthread_mutex_init(&g_control_lock, NULL);

	while(1) {
		fgets(buffer, 100, stdin);
		sscanf(buffer, "%20s %20s", &command, &arg);
		printf("Input received: command=\"%s\" arg=\"%s\"\n", command, arg);
		if (strcmp(command, "throttle") == 0) {
			int is_int = 1;
			char *c = arg;
			while (*c != 0) {
				if (*c > 57 || *c < 48) {
					is_int = 0;
					break;
				}
				c++;
			}
			if (is_int) {
				pthread_mutex_lock(&g_control_lock);
				g_controls.throttle = 2*strtol(arg, NULL, 10);
				pthread_mutex_unlock(&g_control_lock);
			}
			else {
				printf("Invalid throttle value\n");
			}
		}
		if (strcmp(command, "trim_p") == 0)
		{
			pthread_mutex_lock(&g_control_lock);
			g_controls.pitch = strtol(arg, NULL, 10);
			pthread_mutex_unlock(&g_control_lock);
		}
		if (strcmp(command, "trim_r") == 0)
		{
			pthread_mutex_lock(&g_control_lock);
			g_controls.roll = strtol(arg, NULL, 10);
			pthread_mutex_unlock(&g_control_lock);
		}
		else {
			printf("Invalid command\n");
		}
		memset(buffer, 0, 100);
		memset(command, 0, 20);
		memset(arg, 0, 20);
	}
}

void get_controls(Controls *controls)
{
	if (pthread_mutex_trylock(&g_control_lock) == 0) {
		memcpy(controls, &g_controls, sizeof(Controls));
		pthread_mutex_unlock(&g_control_lock);
	}
}
