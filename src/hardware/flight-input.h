#ifndef FLIGHT_INPUT_H
#define FLIGHT_INPUT_H

typedef struct controls
{
	int throttle;
	int roll; 
	int pitch;
} Controls;

void *start_inout(); /* spawn a thread to call this function to start input output engine */
void get_controls(Controls *controls); /* writes current control settings to struct pointed at by 'controls' */
int check_update(); /* returns 1 if main thread needs update, 0 if not*/

#endif