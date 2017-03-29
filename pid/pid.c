// initial test file for demonstrative purposes

#include <math.h>
#include <string.h>
#include "pid.h"

#define KSUBP 0.4
#define KSUBI 0.3
#define KSUBD 0.3
#define SCALE 1.0

void inti_hist(Pidhist *history)
{
	history->last_error = 0;
	history->error_sum = 0;
}

double pid(Pidhist *history, double error, double delta_t)
{
	double p = error*KSUBP;
	history->error_sum += error*delta_t;
	double i = error_sum*KSUBI;
	double d = (delta_t / history->last_error)*KSUBD;
	history->last_error = error;
	return -1*(p + i + d)*SCALE;
}
