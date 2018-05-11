#include "stop.h"

int global_emergency_stop = 0;
pthread_rwlock_t global_stop_lock = PTHREAD_RWLOCK_INITIALIZER;

void set_global_stop()
{
	pthread_rwlock_wrlock(&global_stop_lock);
	global_emergency_stop = 1;
	pthread_rwlock_unlock(&global_stop_lock);
}

int check_global_stop()
{
	int result;
	pthread_rwlock_rdlock(&global_stop_lock);
	result = global_emergency_stop;
	pthread_rwlock_unlock(&global_stop_lock);
	return result;
}
