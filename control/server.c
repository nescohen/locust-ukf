#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#define PORT 6969
#define NETWORK_THROTTLE 1
#define NETWORK_OFF 2

const char off_arr[8] = {1, 0, 0, 0, 0, 0, 0, 0};

void encode_int(int value, char *buffer)
// asumes int is at least 32 bits
// encodes little endian
{
	int i;
	for (i = 0; i < 4; i++) {
		int mask = 0xFF;
		char to_send = (char)((value >> i) & mask);
		buffer[i] = to_send;
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
	socklen_t new_addr_len;
	int new_sock = accept(sockfd, (struct sockaddr *)&new_address, &new_addr_len);
	while (new_sock <= 0) {
		printf("Connection accept error\n");
		return 1;
	}

	char buffer[100];
	int count = read(new_sock, buffer, 99);
	buffer[count] = 0;

	printf("%s\n", buffer);

	int running = 1;
	while(running) {
		char command[20];
		char argument[20];
		scanf("%20s %20s\n", buffer, argument);
		
		if (strcmp(command, "throttle") == 0) {
			int value = strtol(argument, NULL, 0);
			char send_arr[8];
			encode_int(NETWORK_THROTTLE, send_arr);
			encode_int(value, send_arr);
			write(new_sock, send_arr, 8);
		}
		else if (strcmp(command, "off") == 0) {
			write(new_sock, off_arr, 8);
			running = 0;
		}
	}

	return 0;
}
