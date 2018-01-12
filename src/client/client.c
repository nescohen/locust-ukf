#include "client.h"
#include "../error/error_log.h"

#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define SOCKET_PORT 6969
#define SERVER_ADDRESS "192.168.1.36"

static int g_sock;
volatile int g_stop;
volatile int g_throttle;
pthread_mutex_t client_lock;

static int decode_int(char *buffer)
// WARNING: assumes int is at least 32 bits
// decodes assuming little endian
{
	int result = 0;
	int i;
	for (i = 0; i < 4; i++) {
		result |= (int)buffer[i] << i*8;
	}

	return result;
}


int establish_connection()
{
	int sock_fd = socket(AF_INET, SOCK_STREAM, 0);
	if (sock_fd <= 0) {
		log_error("Socket creation failure");
		return -1;
	}
	
	struct sockaddr_in serv_addr;
	memset(&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(SOCKET_PORT);
	
	int err = inet_pton(AF_INET, SERVER_ADDRESS, &(serv_addr.sin_addr));
	if (err <= 0) {
		log_error("Address translation failure");
		return -1;
	}

	err = connect(sock_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
	if (err < 0) {
		log_error("Connection to server failed");
		return -1;
	}

	fcntl(sock_fd, F_SETFL, O_NONBLOCK);

	char *confirm = "Connection confirmed";
	write(sock_fd, confirm, strlen(confirm));

	return sock_fd;
}

void network_client_init()
{
	pthread_mutex_init(&client_lock, NULL);
}

void *network_client_start(void *arg)
// this function initiate connection with server and handle the connection
{
	pthread_mutex_lock(&client_lock);
	g_stop = 0;
	pthread_mutex_unlock(&client_lock);
	g_sock = establish_connection();
	while (!g_stop && g_sock > 0) {
		// check for network throttle instructions
		char buffer[8];
		int count = read(g_sock, buffer, 8);
		if (count > 0) {
			int code = decode_int(buffer);
			int value = decode_int(buffer + 4);

			switch(code) {
				case NETWORK_THROTTLE:
					if (value > 200) value = 200;
					if (value < 0) value = 0;
					pthread_mutex_lock(&client_lock);
					g_throttle = value;
					pthread_mutex_unlock(&client_lock);
					break;
				case NETWORK_OFF:
					if (value == 0) {
						pthread_mutex_lock(&client_lock);
						g_stop = 1;
						pthread_mutex_unlock(&client_lock);
					}
					break;
				default:
					log_error("Received unrecognized network command");
			}
		}
	}
	return NULL;
}

int network_client_get_throttle()
// returns -1 if stopped
{
	pthread_mutex_lock(&client_lock);
	if (g_stop) {
		pthread_mutex_unlock(&client_lock);
		return -1;
	}
	else {
		int result = g_throttle;
		pthread_mutex_unlock(&client_lock);
		return result;
	}
}

void network_client_stop()
{
	pthread_mutex_lock(&client_lock);
	g_stop = 1;
	pthread_mutex_unlock(&client_lock);
}
