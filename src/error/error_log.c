#include <stdio.h>
#include <errno.h>
#include "error_log.h"

static FILE *error_log = NULL;
static int count = 0;

void log_error(const char *msg)
{
	if (error_log == NULL) {
		error_log = fopen("error.log", "w");
	}
	if(error_log != NULL) {
		fprintf(error_log, "%d - %s\n", count++, msg);
	}
	else {
		printf("%d\n", errno);
	}
}

void log_complete()
{
	if (error_log != NULL) {
		fclose(error_log);
	}
}
