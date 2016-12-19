#include "flight-input.h"
#include <stdio.h>
#include <pthread.h>

void *start_inout()
{
	while(1) {
		printf("This is the thread\n");
		sleep(1);
	}
}

void get_controls(Controls *controls)
{

}
