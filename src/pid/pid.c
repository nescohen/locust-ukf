// initial test file for demonstrative purposes

#include <math.h>
#include <string.h>
#include "pid.h"

#define KSUBP 0.5
#define KSUBI 0.0
#define KSUBD 0.5
#define SCALE 1.0

void init_hist(Pidhist *history)
{
	history->last_error = 0;
	history->error_sum = 0;
}

double pid(Pidhist *history, double error, double delta_t)
{
	double p = error*KSUBP;
	history->error_sum += error*delta_t;
	double i = history->error_sum*KSUBI;
	double d = ((error - history->last_error) / delta_t)*KSUBD;
	history->last_error = error;
	return -1*(p + i + d)*SCALE;
}
