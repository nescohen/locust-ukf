#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#define PORT 6969
#define NETWORK_THROTTLE 1
#define NETWORK_OFF 2
#define NETWORK_REPORT 3

const char off_arr[8] = {NETWORK_OFF, 0, 0, 0, 0, 0, 0, 0};
const char report_arr[8] = {NETWORK_REPORT, 0, 0, 0, 0, 0, 0, 0};

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

int translate_command(char *command) {
	if (strcmp(command, "throttle") == 0) {
		return NETWORK_THROTTLE;
	}
	else if (strcmp(command, "off") == 0) {
		return NETWORK_OFF;
	}
	else if (strcmp(command, "report") == 0) {
		return NETWORK_REPORT;
	}
	else {
		return -1;
	}
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
		
		if (strcmp(command, "throttle") == 0) {
			scanf("%d", &argument);
			if (argument >= 0) {
				char send_arr[8];
				encode_int(NETWORK_THROTTLE, send_arr);
				encode_int(argument, send_arr + 4);
				write(new_sock, send_arr, 8);
			}
			else {
				printf("Invalid throttle value\n");
			}
		}
		else if (strcmp(command, "off") == 0) {
			write(new_sock, off_arr, 8);
			running = 0;
		}
		else if (strcmp(command, "report") == 0) {
			write(new_sock, report_arr, 8);
		}
		else {
			printf("unrecognized command - '%s'\n", command);
		}
	}

	return 0;
}
