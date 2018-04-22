#ifndef CLIENT_H
#define CLIENT_H

// network related definitions
#define NETWORK_THROTTLE 1
#define NETWORK_OFF 2
#define NETWORK_REPORT 3

int network_client_init();
// returns true if successful and false if failed

void *network_client_start(void *arg);

int establish_connection();

int network_client_get_throttle();
// returns -1 if stopped

void network_client_stop();

#endif
