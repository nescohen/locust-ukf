#ifndef PID_H
#define PID_H

typedef struct pidhist {
	double last_error;
	double error_sum;
} Pidhist;

void init_hist(Pidhist *history);
double pid(Pidhist *history, double error, double delta_t);

#endif
