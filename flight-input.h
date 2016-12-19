#ifndef FLIGHT_INPUT_H
#define FLIGHT_INPUT_H

typedef struct controls
{
	float throttle;
	float roll; /* unused */
	float pitch; /* unused */
} Controls;

void *start_inout(); /* spawn a thread to call this function to start input output engine */
void get_controls(Controls *controls); /* writes current control settings to struct pointed at by 'controls' */

#endif
