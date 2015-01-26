#include "tcp.h"

personalRobotics::Tcp::Tcp()
{
	// Initialize the winsock framework only once when the fisrt socket is created
	socketCountMutex.lock();
	if (socketCount == 0)
	{
		socketCountMutex.unlock();
		WSAData wsaData;
		int returnCode = WSAStartup(MAKEWORD(2, 2), &wsaData);
		if (returnCode != 0)
		{
			throw SocketException("Unable to load dlls for sockets. WSAStartup failed");
		}
	}
	else
	{
		socketCountMutex.unlock();
	}

	// Clean up the varibles
	std::memset(&hints,0,sizeof(hints));
	result = NULL;
	ptr = NULL;
	dataSocket = INVALID_SOCKET;

	// Setup flags
	isConnectedMutex.lock();
	isConnected = false;
	isConnectedMutex.unlock();
	rmtTermConnMutex.lock();
	remoteTerminatedConnection = false;
	rmtTermConnMutex.unlock();
	gracefulShutdownMutex.lock();
	gracefulShutdown = true;
	gracefulShutdownMutex.unlock();

	// Update the number of sockets initialized
	socketCountMutex.lock();
	socketCount++;
	socketCountMutex.unlock();
}

personalRobotics::Tcp::~Tcp()
{
	// Wait for threads to join
	if (sendThread.joinable())
		sendThread.join();
	if (recvThread.joinable())
		recvThread.join();

	// Close the socket
	closesocket(dataSocket);

	// Decrement the number of live sockets
	socketCountMutex.lock();
	socketCount--;
	socketCountMutex.unlock();

	// Unload the dlls if all sockets are dead
	socketCountMutex.lock();
	if (socketCount == 0)
	{
		socketCountMutex.unlock();
		WSACleanup();
	}
	else
	{
		socketCountMutex.unlock();
	}
}

int personalRobotics::Tcp::write(int length, char* bufferPtr)
{
	int returnCode = send(dataSocket, bufferPtr, length, 0);
	if (returnCode == SOCKET_ERROR)
	{
		throw SocketException("Error sending data over the socket");
	}

	return returnCode;
}

int personalRobotics::Tcp::read(int length, char* bufferPtr)
{
	int returnCode = recv(dataSocket, bufferPtr, length, 0);
	if (returnCode == 0)
	{
		rmtTermConnMutex.lock();
		remoteTerminatedConnection |= true;
		rmtTermConnMutex.unlock();
		gracefulShutdownMutex.lock();
		gracefulShutdown &= true;
		gracefulShutdownMutex.unlock();
		disconnect();
	}
	else if (returnCode == SOCKET_ERROR)
	{
		rmtTermConnMutex.lock();
		remoteTerminatedConnection |= true;
		rmtTermConnMutex.unlock();
		gracefulShutdownMutex.lock();
		gracefulShutdown &= false;
		gracefulShutdownMutex.unlock();
		throw SocketException("Error in reading data from connection");
	}

	return returnCode;
}

void personalRobotics::Tcp::asyncSend(int length, char* bufferPtr)
{

}

void personalRobotics::Tcp::asyncRead(int length, char* bufferPtr)
{

}

void personalRobotics::Tcp::disconnect()
{
	// Close the send channel
	{
		int returnCode = shutdown(dataSocket, SD_SEND);
		if (returnCode == 0)
		{
			gracefulShutdownMutex.lock();
			gracefulShutdown &= true;
			gracefulShutdownMutex.unlock();
			rmtTermConnMutex.lock();
			remoteTerminatedConnection |= false;
			rmtTermConnMutex.unlock();
		}
		if (returnCode == SOCKET_ERROR)
		{
			gracefulShutdownMutex.lock();
			gracefulShutdown &= false;
			gracefulShutdownMutex.unlock();
			rmtTermConnMutex.lock();
			remoteTerminatedConnection |= false;
			rmtTermConnMutex.unlock();
			throw SocketException("Error trying to shutdown socket");
		}
	}

	// Flush the read buffer and close the receive channel
	{
		rmtTermConnMutex.lock();
		if (!remoteTerminatedConnection)
		{
			rmtTermConnMutex.unlock();
			int returnCode = 1;
			char dumpBuffer[DUMP_BUFFER_LENGTH];
			while (returnCode != 0)
			{
				returnCode = recv(dataSocket, dumpBuffer, DUMP_BUFFER_LENGTH, 0);
				if (returnCode == 0)
				{
					gracefulShutdown &= true;
					remoteTerminatedConnection |= true;
				}
				else if (returnCode == SOCKET_ERROR)
				{
					gracefulShutdown &= false;
					remoteTerminatedConnection |= true;
					throw SocketException("Error in reading data from connection");
				}
			}
		}
		else
		{
			rmtTermConnMutex.unlock();
		}
	}

	// Close the socket to free any occupied resources and reset the flags to constructor set values
	closesocket(dataSocket);
	dataSocket = INVALID_SOCKET;
	isConnectedMutex.lock();
	isConnected = false;
	isConnectedMutex.unlock();
	rmtTermConnMutex.lock();
	remoteTerminatedConnection = false;
	rmtTermConnMutex.unlock();
	gracefulShutdownMutex.lock();
	gracefulShutdown = true;
	gracefulShutdownMutex.unlock();
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
	listenerStoppedMutex.lock();
	listenerStopped = true;
	listenerStoppedMutex.unlock();

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
	listenerStoppedMutex.lock();
	listenerStopped = true;
	listenerStoppedMutex.unlock();

	// Get address information of the host to bind a listener socket to
	int returnCode = getaddrinfo(NULL,portNumber.c_str(), &hints, &result);
	if (returnCode != 0)
	{
		throw SocketException("Failed to get address information");
	}

	// Start listening for incoming connection
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
		}
	}
}

personalRobotics::TcpServer::~TcpServer()
{
	// stop the listener thread
	stop();

	// Close the listener socket
	closesocket(listener);
}

void personalRobotics::TcpServer::start()
{
	listenerStoppedMutex.lock();
	if (listenerStopped)
	{
		listenerStopped = false;
		listenerStoppedMutex.unlock();
		listenerThread = std::thread(&TcpServer::listenRoutine, this);
	}
	else
	{
		listenerStoppedMutex.unlock();
	}
}

void personalRobotics::TcpServer::stop()
{
	listenerStoppedMutex.lock();
	if (!listenerStopped)
	{
		listenerStopped = true;
		listenerStoppedMutex.unlock();
		if (listenerThread.joinable())
			listenerThread.join();
	}
	else
	{
		listenerStoppedMutex.unlock();
	}
}

void personalRobotics::TcpServer::listenRoutine()
{
	while (!listenerStopped)
	{
		int returnCode = listen(listener, SOMAXCONN);
		if (returnCode == SOCKET_ERROR)
		{
			throw SocketException("Unable listen for connections");
		}
		isConnectedMutex.lock();
		if (!isConnected)
		{
			dataSocket = accept(listener, NULL, NULL);
			if (dataSocket = INVALID_SOCKET)
			{
				throw SocketException("Unable to accept incoming connection");
			}
			else
			{
				isConnected = true;
			}
		}
		isConnectedMutex.unlock();
	}
}