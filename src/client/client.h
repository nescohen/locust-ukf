#ifndef CLIENT_H
#define CLIENT_H

// Network command enum
#define NETWORK_INIT		((int)1)
#define NETWORK_ALIGN		((int)2)
#define NETWORK_START		((int)3)
#define NETWORK_THROTTLE 	((int)4)
#define NETWORK_REPORT 		((int)5)
#define NETWORK_OFF 		((int)6)

// control signal related definitions
#define INTERNAL_FLAG 		((int)(1 << 20))
#define INTERNAL_STOP 		((int)(1 | INTERNAL_FLAG))

typedef struct command {
	int type;
	int value;
} Command;

typedef struct response {
	int c_type;
	int value;
} Response;

int send_response(Response *resp);
// blocks until response is accepting into delivery queue

void get_command(Command *next);
// blocks until at least one command is available and then dequeues the command and copies to next

int network_client_init();
// returns true if successful and false if failed

void *network_client_start(void *arg);

int establish_connection();

void network_client_stop();

#endif
