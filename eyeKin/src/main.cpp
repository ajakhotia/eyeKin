#include "eyeKin.h"
#include <Windows.h>

// Socket counter for managing loading and unloading of resources
personalRobotics::MutexType<int> personalRobotics::Tcp::socketCount = 0;

// Globals for managing socket streaming and calibration
std::thread sendThread;							// SenderThread
personalRobotics::MutexBool sendData;			// Data control flag
personalRobotics::EyeKin eyeKin;				// Eyekin object to handle all vision calls
void sendRoutine();								// Routine for sending messages to client
bool readRoutine(procamPRL::EntityList &list);	// Routine for reading messages from client
void startStreaming();							// Sends data to currently connected client
void stopStreaming();							// Stops the server and resets the state

void main(int argC, char **argV)
{
	// Do a placeholdercalibration
	sendData.set(false);
	eyeKin.calibrate();

	// Look for incoming commands and change the state of the machine
	startStreaming();
	while (true)
	{
		if (eyeKin.getServer()->isConnected.get())
		{
			procamPRL::EntityList commandList;
			if (readRoutine(commandList))
			{
				switch (commandList.command())
				{
				case procamPRL::EntityList::NONE:
					break;
				case procamPRL::EntityList::START_CALIBRATION:
					std::cout << "starting calibration" << std::endl;
					break;
				case procamPRL::EntityList::CALIBRATION_COMPLETE:
					std::cout << "stopping calibration" << std::endl;
					break;
				case procamPRL::EntityList::START_STREAM:
					std::cout << "starting stream" << std::endl;
					break;
				case procamPRL::EntityList::STOP_STREAM:
					std::cout << "stopping stream" << std::endl;
					break;
				case procamPRL::EntityList::SEND_DISPLAY_INFO_PACKET:
					std::cout << "sending display info packet" << std::endl;
					break;
				case procamPRL::EntityList::DISCONNECT:
					std::cout << "disconnecting ..." << std::endl;
					break;
				}
			}
		}
		else
		{
			Sleep(50);
		}
	}
}


void sendRoutine()
{
	while (sendData.get())
	{
		if (eyeKin.getSegmentor()->newListGenerated.get())
		{
			if (eyeKin.getServer()->isConnected.get())
			{
				// Allocate space for list
				procamPRL::EntityList serializableList;
				if (eyeKin.getSegmentor()->getEntityList()->size() == 0)
					continue;
				eyeKin.generateSerializableList(serializableList);

				// Send data over socket
				std::string outString;
				//bufferMutex.lock();
				serializableList.SerializeToString(&outString);
				int dataLenght = outString.length();
				if (dataLenght > 0)
				{
					int networkOrderDataLength = htonl(dataLenght);
					eyeKin.getServer()->write(4, (char*)&networkOrderDataLength);
					eyeKin.getServer()->write(dataLenght, (char*)outString.c_str());
				}
			}
			else
			{
				eyeKin.getSegmentor()->newListGenerated.set(false);
				sendData.set(false);
			}
		}
		Sleep(25);
	}
}
bool readRoutine(procamPRL::EntityList &list)
{
	// Clear the list
	list.Clear();

	// Read 4 bytes, cast to int and convert to host byte ordering
	char sizeBuffer[4];
	if (eyeKin.getServer()->read(4, sizeBuffer))
	{
		int size = ntohl(*((int*)(sizeBuffer)));
		std::string messageString;
		messageString.reserve(size);
		eyeKin.getServer()->read(size, &messageString[0]);
		list.ParseFromArray((void*)&messageString[0], size);
		return true;
	}
	else
		return false;
}
void startStreaming()
{
	sendData.set(true);
	sendThread = std::thread(&sendRoutine);
}
void stopStreaming()
{
	sendData.set(false);
	if (sendThread.joinable())
		sendThread.join();
}