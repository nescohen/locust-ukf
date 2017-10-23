#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#define PORT 6969

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

	return 0;
}
