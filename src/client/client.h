#ifndef CLIENT_H
#define CLIENT_H

// network related definitions
#define NETWORK_THROTTLE 1
#define NETWORK_OFF 2

void network_client_init();

void *network_client_start(void *arg);

int establish_connection();

void network_client_stop();

#endif
