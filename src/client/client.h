#ifndef CLIENT_H
#define CLIENT_H

// network related definitions
#define NETWORK_THROTTLE ((int)1)
#define NETWORK_OFF ((int)2)
#define NETWORK_REPORT ((int)3)

typedef struct command {
	int type;
	int value;
} Command;

int poll_command(Command *next);
// non-blocking, true if command is present, false if not

int network_client_init();
// returns true if successful and false if failed

void *network_client_start(void *arg);

int establish_connection();

void network_client_stop();

#endif
