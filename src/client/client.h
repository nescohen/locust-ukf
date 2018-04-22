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

int network_client_init();
// returns true if successful and false if failed

void *network_client_start(void *arg);

int establish_connection();

int network_client_get_throttle();
// returns -1 if stopped

void network_client_stop();

#endif
