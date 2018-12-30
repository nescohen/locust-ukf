#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "../kalman/ukf_mrp.h"

#define ACTION_NOACTION 	((int) 0x0)
#define ACTION_INIT 		((int) 0x1)
#define ACTION_ALIGN 		((int) 0x3)
#define ACTION_START 		((int) 0x3)

#define STATE_UNINITIALIZED ((int) 0x10)
#define STATE_INITIALIZED   ((int) 0x11)
#define STATE_ALIGNED 		((int) 0x12)
#define STATE_RUNNING 		((int) 0x13)

typedef struct directives
{
	int throttle;
	int next_action;
} Directives;

typedef struct report
{
	int curr_state;
	double ukf_state[SIZE_STATE];
} Report;

void init_directives(Directives *directives);
void nav_set_directives(Directives *directives);
// warning: blocking
void nav_get_report(Report *writeback);
// warning: blocking

void *navigation_main(void *arg);
// main navigation loop, running on worker thread

void stop_nav();
// shuts down sensors and the sensor and motor threads

#endif
