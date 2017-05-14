// initial test file for demonstrative purposes

#include <math.h>
#include <string.h>
#include "pid.h"

#define KSUBP 0.5
#define KSUBI 0.0
#define KSUBD 0.5
#define SCALE 1.0

#define RC 1.0

void init_hist(Pidhist *history)
{
	history->started = 0;
	history->last_error = 0;
	history->error_sum = 0;
}

double pid(Pidhist *history, double error, double delta_t)
{
	// Apply a low pass filter to smooth incoming data
	double alpha = delta_t / (RC + delta_t);
	double low_pass = alpha * error + (1 - alpha)*history->last_error;

	// Do calculations for proportional, integral, derivative
	double p = low_pass*KSUBP;
	history->error_sum += low_pass*delta_t;
	double i = history->error_sum*KSUBI;
	if (!history->started) {
		history->last_error = low_pass;
		history->started = 1;
	}
	double d = ((low_pass - history->last_error) / delta_t)*KSUBD;
	history->last_error = low_pass;

	//return the correction calculated
	return -1*(p + i + d)*SCALE;
}
