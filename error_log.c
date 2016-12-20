#include "error_log.h"
#include <stdio.h>

static FILE *error_log = NULL;

int log_error(const char *msg)
{
	if (error_log == NULL) {
		error_log = fopen("error.log", "w");
	}
	fprintf(error_log, "%s\n", msg);
}
