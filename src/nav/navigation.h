#ifndef NAVIGATION_H
#define NAVIGATION_H

typedef struct directives
{
	int throttle;
	int stop;
} Directives;

void init_directives(Directives *directives);
void nav_set_directives(Directives *directives);

void *navigation_main(void *arg);
// main navigation loop, running on worker thread

void stop_nav();
// shuts down sensors and the sensor and motor threads

#endif
