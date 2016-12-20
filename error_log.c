#include "error_log.h"
#include <stdio.h>

static FILE *error_log = NULL;
static count = 0;

void log_error(const char *msg)
{
	if (error_log == NULL) {
		error_log = fopen("error.log", "w");
	}
	fprintf(error_log, "%d - %s\n", count++, msg);
}

void log_complete()
{
	fclose(error_log);
}
