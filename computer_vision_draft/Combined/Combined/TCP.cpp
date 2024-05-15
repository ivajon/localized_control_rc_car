#include "TCP.h"

#ifdef TCP
void TCPclient(char first, char second) {
	int clientSocket = socket(AF_INET, SOCK_STREAM, 0);

	sockaddr_in serverAddress;
	serverAddress.sin_family = AF_INET;
	serverAddress.sin_port = htons(8080);
	serverAddress.sin_addr.s_addr = INADDR_ANY;

	// sending connection request
	connect(clientSocket, (struct sockaddr*)&serverAddress,
		sizeof(serverAddress));

	// sending data
	const char message[2] = { first, second };
	send(clientSocket, message, sizeof(message), 0);

}

