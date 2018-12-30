#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#define PORT 6969
#define NETWORK_INIT		((int) 1)
#define NETWORK_ALIGN		((int) 2)
#define NETWORK_START		((int) 3)
#define NETWORK_THROTTLE	((int) 4)
#define NETWORK_REPORT		((int) 5)
#define NETWORK_OFF			((int) 6)

#define COMMAND_MIN ((int) 1)
#define COMMAND_MAX ((int) 6)

static char *c_names[6] = { "init"
                          , "align"
						  , "start"
						  , "throttle"
						  , "report"
						  , "off"
						  };

typedef struct response {
	int c_type;
	int value;
} Response;

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


void encode_int(int value, char *buffer)
// asumes int is at least 32 bits
// encodes little endian
{
	int i;
	for (i = 0; i < 4; i++) {
		int mask = 0xFF;
		char to_send = (char)((value >> i*8) & mask);
		buffer[i] = to_send;
	}
}

static void pure_command(int command, char *buffer)
{
	encode_int(command, buffer);
	encode_int(0, buffer + 4);
}

static char *show_command(int command) {
	if (command >= COMMAND_MIN && command <= COMMAND_MAX) {
		return c_names[command - 1];
	}
	else {
		return "unknown!! ";
	}
}

void translate_response(char *resp, Response *wb)
{
	wb->c_type = decode_int(resp);
	wb->value  = decode_int(resp + 4);
}

int main()
{
	//create socket to use
	int sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd <= 0) {
		printf("Socket creation failure\n");
		return 1;
	}

	struct sockaddr_in address;
	socklen_t addr_len = sizeof(address);
	address.sin_family = AF_INET;
	address.sin_port = htons(PORT);
	address.sin_addr.s_addr = INADDR_ANY;

	int err = bind(sockfd, (struct sockaddr *)&address, addr_len);
	if (err != 0) {
		printf("Socket bind failure\n");
		return 1;
	}

	err = listen(sockfd, 1);
	if(err < 0) {
		printf("Port listen failure\n");
		return 1;
	}

	struct sockaddr_in new_address;
	socklen_t new_addr_len = sizeof(new_address);
	int new_sock = accept(sockfd, (struct sockaddr *)&new_address, &new_addr_len);
	if (new_sock <= 0) {
		printf("Connection accept error: %d\n", errno);
		return 1;
	}

	char buffer[100];
	int count = read(new_sock, buffer, 99);
	buffer[count] = 0;

	printf("%s\n", buffer);

	int running = 1;
	while(running) {
		printf("Prompt: ");
		char command[20];
		int argument = -1;
		scanf("%19s", command);

		char buffer[8];
		if (strcmp(command, "init") == 0) {
			pure_command(NETWORK_INIT, buffer);
			write(new_sock, buffer, 8);
		}
		else if (strcmp(command, "align") == 0) {
			pure_command(NETWORK_ALIGN, buffer);
			write(new_sock, buffer, 8);
		}
		else if (strcmp(command, "start") == 0) {
			pure_command(NETWORK_START, buffer);
			write(new_sock, buffer, 8);
		}
		else if (strcmp(command, "throttle") == 0) {
			scanf("%d", &argument);
			if (argument >= 0) {
				encode_int(NETWORK_THROTTLE, buffer);
				encode_int(argument, buffer + 4);
				write(new_sock, buffer, 8);
			}
			else {
				printf("Invalid throttle value\n");
			}
		}
		else if (strcmp(command, "off") == 0) {
			pure_command(NETWORK_OFF, buffer);
			write(new_sock, buffer, 8);
			running = 0;
		}
		else if (strcmp(command, "report") == 0) {
			pure_command(NETWORK_REPORT, buffer);
			write(new_sock, buffer, 8);
		}
		else {
			printf("unrecognized command - '%s'\n", command);
		}

		int count = read(new_sock, buffer, 8);
		if (count == 8) {
			Response response;
			translate_response(buffer, &response);
			if (response.value) {
				printf("%s command completed successfully\n", show_command(response.c_type));
			}
			else {
				printf("%s command failed\n", show_command(response.c_type));
			}
		}

	}

	return 0;
}
