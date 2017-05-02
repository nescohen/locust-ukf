#include <stdio.h>
#include "error_log.h"

static FILE *error_log = NULL;
static int count = 0;

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
