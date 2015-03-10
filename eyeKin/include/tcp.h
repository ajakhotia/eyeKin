#ifndef ___TCP__H___
#define ___TCP__H___

#ifndef UNICODE
#define UNICODE
#endif

#define WIN32_LEAN_AND_MEAN

#include <Windows.h>
#include <winsock2.h>
#include <Ws2tcpip.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <thread>
#include <exception>
#include <vector>
#include <mutex>
#include "utilities.h"

#define ADDRESS_FAMILY AF_INET		// Refer https://msdn.microsoft.com/en-us/library/windows/desktop/ms737530(v=vs.85).aspx
#define SOCKET_TYPE SOCK_STREAM		// Refer https://msdn.microsoft.com/en-us/library/windows/desktop/ms737530(v=vs.85).aspx
#define PROTOCOL IPPROTO_TCP		// Refer https://msdn.microsoft.com/en-us/library/windows/desktop/ms737530(v=vs.85).aspx
#define ADDRESS_FLAGS AI_PASSIVE	// Refer https://msdn.microsoft.com/en-us/library/windows/desktop/ms737530(v=vs.85).aspx
#define DUMP_BUFFER_LENGTH 10		// The length of buffer to dump the incoming data after calling disconnect

#pragma comment(lib, "Ws2_32.lib")

namespace personalRobotics
{
	class SocketException : public std::exception
	{
	protected:
		std::string ss;
	public:
		SocketException(std::string s) : ss(s)
		{
			std::cout << ss << std::endl;
		}
		~SocketException()
		{
		}
		virtual const char* what() const throw()
		{
			return ss.c_str();
		}
	};

	class Tcp
	{
	protected:
		struct addrinfo hints;				// Used to give hints to the getaddrinfo function specifying the address family, port number, connection type, etc.
		struct addrinfo *result;			// Used to store a singly linked list which results from the getaddrinfo function. First item on the list is generally most relevant one to use.
		struct addrinfo *ptr;				// A pointer to move through the result linked list
		SOCKET dataSocket;					// A vector of handlers to the actual socket
		std::thread sendThread;				// Sender thread
		std::thread recvThread;				// Recevier thread
		static MutexType<int> socketCount;			// Used to figure out when to unload the dll by calling WSACleanup().
	public:
		Tcp();
		virtual ~Tcp();
		void write(int length, char* bufferPtr);
		bool read(int length, char* bufferPtr);
		void asyncSend(int length, char* bufferPtr, std::mutex &bufferMutex, bool lock = false, bool unlock = true);
		void asyncRead(int length, char* bufferPtr, std::mutex &bufferMutex, bool lock = false, bool unlock = true);
		void cleanup();
		void disconnect();
		void reset();
		MutexBool isConnected;					// Signals if the data socket is connected
		MutexBool sendChannelOpen;
		MutexBool recvChannelOpen;
		MutexBool remoteTerminatedConnection;	// Signals if remote socket terminated the connection
	};

	// Server class
	class TcpServer : public Tcp
	{
	protected:
		SOCKET listener;
		MutexBool listenerStopped;
		std::thread listenerThread;
		void listenRoutine();
	public:
		TcpServer(int portNumber, int addressFamily, int socketType = SOCK_STREAM, int protocol = IPPROTO_TCP, int flags = AI_PASSIVE);
		TcpServer(std::string portNumber, int addressFamily, int socketType = SOCK_STREAM, int protocol = IPPROTO_TCP, int flags = AI_PASSIVE);
		~TcpServer();
		void start();
		void stop();
	};

	// Client class
	/*class TcpClient : public Tcp
	{
	protected:

	public:
		TcpClient();
		~TcpClient();
	};*/
}
#endif