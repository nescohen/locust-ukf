#include "flight-input.h"
#include "error_log.h"
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <string.h>

static Controls g_controls = {0, 0, 0};
static int g_update = 1; /* 1 = needs update, 0 = no update needed */
pthread_mutex_t g_control_lock;

int is_int(char *check)
//Andy - check if supplied parameter is an int
{
	char *c = check;
	if (*c == '-' || *c == '+') c++; /* check for optional sign character */
	while (*c != 0) {
		if (*c > 57 || *c < 48) {
			return 0;
		}
		c++;
	}
	return 1;
}

void process_command(char *command, char *arg)
{
	printf("Input received: command=\"%s\" arg=\"%s\"\n", command, arg);
	if (strcmp(command, "throttle") == 0) {
		if (is_int(arg)) {
			pthread_mutex_lock(&g_control_lock);
			g_update = 1;
			g_controls.throttle = 2*strtol(arg, NULL, 10);
			pthread_mutex_unlock(&g_control_lock);
		}
		else {
			printf("Invalid throttle value\n");
		}
	}
	else if (strcmp(command, "trim_p") == 0) {
		if (is_int(arg)) {
			pthread_mutex_lock(&g_control_lock);
			g_controls.pitch = strtol(arg, NULL, 10);
			g_update = 1;
			pthread_mutex_unlock(&g_control_lock);
		}
		else {
			printf("Invalid trim value\n");
		}
	}
	else if (strcmp(command, "trim_r") == 0)	{
		if(is_int(arg)){
			pthread_mutex_lock(&g_control_lock);
			g_controls.roll = strtol(arg, NULL, 10);
			g_update = 1;
			pthread_mutex_unlock(&g_control_lock);
		}
		else {
			printf("Invalid trim value\n");
		}
	}
	else {
		printf("Invalid command\n");
	}
}


void *start_inout()
{
	char buffer[100];
	char command[20];
	char arg[20];

	pthread_mutex_init(&g_control_lock, NULL);

	while(1) {
		memset(buffer, 0, 100);
		memset(command, 0, 20);
		memset(arg, 0, 20);
		fgets(buffer, 100, stdin);
		sscanf(buffer, "%20s %20s", command, arg);
		process_command(command, arg);
	}
}

void get_controls(Controls *controls)
{
	if (pthread_mutex_trylock(&g_control_lock) == 0) {
		memcpy(controls, &g_controls, sizeof(Controls));
		g_update = 0;
		pthread_mutex_unlock(&g_control_lock);
	}
}

int check_update()
{
	if (pthread_mutex_trylock(&g_control_lock) == 0) {
		int update = g_update;
		pthread_mutex_unlock(&g_control_lock);
		return update;
	}
	else return 0;
}
