#include "tcp.h"

personalRobotics::Tcp::Tcp()
{
	// Initialize the winsock framework only once when the fisrt socket is created
	if (socketCount.get() == 0)
	{
		WSAData wsaData;
		int returnCode = WSAStartup(MAKEWORD(2, 2), &wsaData);
		if (returnCode != 0)
		{
			throw SocketException("Unable to load dlls for sockets. WSAStartup failed");
		}
	}

	// Clean up the varibles
	std::memset(&hints,0,sizeof(hints));
	result = NULL;
	ptr = NULL;
	dataSocket = INVALID_SOCKET;

	// Setup flags
	isConnected.set(false);
	sendChannelOpen.set(false);
	recvChannelOpen.set(false);
	remoteTerminatedConnection.set(false);

	// Update the number of sockets initialized
	socketCount++;
}
personalRobotics::Tcp::~Tcp()
{
	// Wait for threads to join
	if (sendThread.joinable())
		sendThread.join();
	if (recvThread.joinable())
		recvThread.join();

	// Disconnect any active connections
	disconnect();

	// Close the socket
	closesocket(dataSocket);

	// Decrement the number of live sockets
	socketCount--;

	// Unload the dlls if all sockets are dead
	if (socketCount.get() == 0)
	{
		WSACleanup();
	}
}
void personalRobotics::Tcp::write(int length, char* bufferPtr)
{
	if (sendChannelOpen.get())
	{
		int returnCode = send(dataSocket, bufferPtr, length, 0);
		if (returnCode == SOCKET_ERROR)
		{
			// Shutdown the socket's send stream stream if its not already being closed else simply exit.
			if (sendChannelOpen.get())
			{
				int returnCode = shutdown(dataSocket, SD_SEND);
				if (returnCode == SOCKET_ERROR)
				{
					// Set the flags to crash the entire socket
					sendChannelOpen.set(false);
					recvChannelOpen.set(false);
					reset();
					return;
				}
				else
				{
					sendChannelOpen.set(false);
					remoteTerminatedConnection.set(true);
					cleanup();
					return;
				}
			}
		}
	}
}
bool personalRobotics::Tcp::read(int length, char* bufferPtr)
{
	if (recvChannelOpen.get())
	{
		int recvLen = 0;
		while (recvLen<length)
		{
			int returnCode = recv(dataSocket, &bufferPtr[recvLen], length - recvLen, 0);
			if (returnCode == 0)
			{
				// Shutdown the socket's receive stream if not already being shutdown
				if (recvChannelOpen.get())
				{
					recvChannelOpen.set(false);
					remoteTerminatedConnection.set(true);
					shutdown(dataSocket, SD_RECEIVE);
					cleanup();
					return false;
				}
				return false;
			}
			else if (returnCode == SOCKET_ERROR)
			{
				// Set the flags to crash the entire socket
				recvChannelOpen.set(false);
				sendChannelOpen.set(false);
				remoteTerminatedConnection.set(true);
				
				// Reset the soceket
				reset();
				return false;
			}
			else
			{
				recvLen += returnCode;
			}
		}
		return true;
	}
}
void personalRobotics::Tcp::asyncSend(int length, char* bufferPtr, std::mutex &bufferMutex, bool lock, bool unlock)
{
	if (lock)
		bufferMutex.lock();
	sendThread = std::thread(&personalRobotics::TcpServer::write, this, length, bufferPtr);
	if (unlock)
		bufferMutex.unlock();
}
void personalRobotics::Tcp::asyncRead(int length, char* bufferPtr, std::mutex &bufferMutex, bool lock, bool unlock)
{
	if (lock)
		bufferMutex.lock();
	recvThread = std::thread(&personalRobotics::TcpServer::read, this, length, bufferPtr);
	if (unlock)
		bufferMutex.unlock();
}
void personalRobotics::Tcp::cleanup()
{
	if (sendChannelOpen.get() && !recvChannelOpen.get())
	{
		// Implies receive channel stopped cleanly and awaits to stop write channel
		// Set the flags to close send channel
		sendChannelOpen.set(false);
		
		// Close the send stream
		int returnCode = shutdown(dataSocket, SD_SEND);
		if (returnCode == SOCKET_ERROR)
		{
			std::cout << "Error in closing socket. Error code: " << WSAGetLastError() << std::endl;
			reset();
			return;
		}
	}

	if (!sendChannelOpen.get() && recvChannelOpen.get())
	{
		// Implies send channel closed cleanly and awaits receive channel to close
		// Set flags to close receive channel
		recvChannelOpen.set(false);
		
		// Flush the buffer
		int returnCode = 1;
		char dumpBuffer[DUMP_BUFFER_LENGTH];
		while (returnCode != 0)
		{
			returnCode = recv(dataSocket, dumpBuffer, DUMP_BUFFER_LENGTH, 0);
			if (returnCode == 0)
			{
				shutdown(dataSocket, SD_RECEIVE);
			}
			else if (returnCode == SOCKET_ERROR)
			{
				std::cout << "Error in closing socket. Error code: " << WSAGetLastError() << std::endl;
				reset();
				return;
			}
		}
	}

	if (!sendChannelOpen.get() && !recvChannelOpen.get())
	{
		// Implies both the channels have been shutdown. Reset the socket
		reset();
	}
}
void personalRobotics::Tcp::disconnect()
{
	// Close the send channel if not already closed
	if (sendChannelOpen.get())
	{
		// Shutdown the socket's send stream
		sendChannelOpen.set(false);
		remoteTerminatedConnection.set(false);
		int returnCode = shutdown(dataSocket, SD_SEND);
		if (returnCode == SOCKET_ERROR)
		{
			// Signal closing of all channels before crashing the socket. sendChannel is already signaled as closed.
			recvChannelOpen.set(false);
			std::cout << "Error in closing socket. Error code: " << WSAGetLastError() << std::endl;
			reset();
			return;
		}
	}

	// Flush the read buffer and close the receive channel
	if (recvChannelOpen.get())
	{
		remoteTerminatedConnection.set(false);
		recvChannelOpen.set(false);
		int returnCode = 1;
		char dumpBuffer[DUMP_BUFFER_LENGTH];
		while (returnCode != 0)
		{
			returnCode = recv(dataSocket, dumpBuffer, DUMP_BUFFER_LENGTH, 0);
			if (returnCode == 0)
			{
				shutdown(dataSocket, SD_RECEIVE);
			}
			else if (returnCode == SOCKET_ERROR)
			{
				std::cout << "Error in closing socket. Error code: " << WSAGetLastError() << std::endl;
				reset();
				return;
			}
		}
	}
	reset();
}
void personalRobotics::Tcp::reset()
{
	// Close the socket to free any occupied resources and reset the flags to constructor set values
	sendChannelOpen.set(false);
	recvChannelOpen.set(false);
	isConnected.set(false);
	remoteTerminatedConnection.set(false);
	closesocket(dataSocket);
	dataSocket = INVALID_SOCKET;
}


personalRobotics::TcpServer::TcpServer(int portNumber, int addressFamily, int socketType, int protocol, int flags)
{
	// Cleaning
	listener = INVALID_SOCKET;

	// Setup
	hints.ai_family = addressFamily;
	hints.ai_socktype = socketType;
	hints.ai_protocol = protocol;
	hints.ai_flags = flags;

	// Set flags
	listenerStopped.set(true);

	// Get address information of the host to bind a listener socket to
	int returnCode = getaddrinfo(NULL, std::to_string(portNumber).c_str(), &hints, &result);
	if (returnCode != 0)
	{
		freeaddrinfo(result);
		throw SocketException("Failed to get address information");
	}

	// Configure listener socket
	listener = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
	if (listener == INVALID_SOCKET)
	{
		freeaddrinfo(result);
		closesocket(listener);
		throw SocketException("Failed to initialize a listener socket for incoming connections");
	}

	// Bind the socket to the first relavant address
	for (ptr = result; ptr != NULL; ptr = ptr->ai_next)
	{
		if (ptr->ai_family != addressFamily)
			continue;
		else
		{
			returnCode = bind(listener, ptr->ai_addr, ptr->ai_addrlen);
			freeaddrinfo(result);
			if (returnCode == SOCKET_ERROR)
			{
				closesocket(listener);
				throw SocketException("Unable to bind the listner socket");
			}
			break;
		}
	}
}
personalRobotics::TcpServer::TcpServer(std::string portNumber, int addressFamily, int socketType, int protocol, int flags)
{
	// Cleaning
	listener = INVALID_SOCKET;

	// Setup
	hints.ai_family = addressFamily;
	hints.ai_socktype = socketType;
	hints.ai_protocol = protocol;
	hints.ai_flags = flags;

	// Set flags
	listenerStopped.set(true);

	// Get address information of the host to bind a listener socket to
	int returnCode = getaddrinfo(NULL, portNumber.c_str(), &hints, &result);
	if (returnCode != 0)
	{
		freeaddrinfo(result);
		throw SocketException("Failed to get address information");
	}

	// Configure listener socket
	listener = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
	if (listener == INVALID_SOCKET)
	{
		freeaddrinfo(result);
		closesocket(listener);
		throw SocketException("Failed to initialize a listener socket for incoming connections");
	}

	// Bind the socket to the first relavant address
	for (ptr = result; ptr != NULL; ptr = ptr->ai_next)
	{
		if (ptr->ai_family != addressFamily)
			continue;
		else
		{
			returnCode = bind(listener, ptr->ai_addr, ptr->ai_addrlen);
			freeaddrinfo(result);
			if (returnCode == SOCKET_ERROR)
			{
				closesocket(listener);
				throw SocketException("Unable to bind the listner socket");
			}
			break;
		}
	}
}
personalRobotics::TcpServer::~TcpServer()
{
	// stop the listener thread
	stop();

	shutdown(dataSocket, SD_BOTH);

	// Close the listener socket
	closesocket(listener);
}
void personalRobotics::TcpServer::start()
{
	if (listenerStopped.get())
	{
		listenerStopped.set(false);
		listenerThread = std::thread(&TcpServer::listenRoutine, this);
	}
}
void personalRobotics::TcpServer::stop()
{
	listenerStopped.set(true);
	if (listenerThread.joinable())
		listenerThread.join();
	std::cout << "done deleting listenerthread" << std::endl;
}
void personalRobotics::TcpServer::listenRoutine()
{
	while (!listenerStopped.get())
	{
		int returnCode = listen(listener, SOMAXCONN);
		if (returnCode == SOCKET_ERROR)
		{
			throw SocketException("Unable listen for connections");
		}
		if (!isConnected.get())
		{
			dataSocket = accept(listener, NULL, NULL);
			if (dataSocket == INVALID_SOCKET)
			{
				throw SocketException("Unable to accept incoming connection");
			}
			else
			{
				isConnected.set(true);
				sendChannelOpen.set(true);
				recvChannelOpen.set(true);
			}
		}
	}
}