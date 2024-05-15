
//#include <winsock2.h>
//#include <sys/types.h>
#pragma once
//For use on linux?
#ifdef TCP
	#include <netinet/in.h>
	#include <sys/socket.h>
	#include <unistd.h>
#endif


void TCPclient(char x, char y);

