#include "TCP.h"

void TCPclient(char first, char second,int clientSocket) {

	// sending data
	const char message[2] = { first, second };
	send(clientSocket, message, sizeof(message), 0);

}

