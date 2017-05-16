#include <stdio.h>
#include <errno.h>
#include <pthread.h>
#include "error_log.h"

static FILE *error_log = NULL;
static int count = 0;
static pthread_mutex_t error_lock = (pthread_mutex_t) PTHREAD_MUTEX_INITIALIZER;

void log_error(const char *msg)
{
	pthread_mutex_lock(&error_lock);
	if (error_log == NULL) {
		error_log = fopen("error.log", "w");
	}
	if(error_log != NULL) {
		fprintf(error_log, "%d - %s\n", count++, msg);
	}
	else {
		printf("%d\n", errno);
	}
	pthread_mutex_unlock(&error_lock);
}

void log_complete()
{
	pthread_mutex_lock(&error_lock);
	if (error_log != NULL) {
		fclose(error_log);
	}
	pthread_mutex_unlock(&error_lock);
}
