#ifndef CLIENT_H
#define CLIENT_H

// control signal related definitions
#define INTERNAL_FLAG ((int)(1 << 20))

#define NETWORK_THROTTLE ((int)1)
#define NETWORK_OFF ((int)2)
#define NETWORK_REPORT ((int)3)
#define INTERNAL_STOP ((int)(1 | INTERNAL_FLAG))

typedef struct command {
	int type;
	int value;
} Command;

void get_command(Command *next);
// blocks until at least one command is available and then dequeues the command and copies to next

int network_client_init();
// returns true if successful and false if failed

void *network_client_start(void *arg);

int establish_connection();

void network_client_stop();

#endif
