#include "eyeKin.h"
#include <Windows.h>

// Socket counter for managing loading and unloading of resources
personalRobotics::MutexType<int> personalRobotics::Tcp::socketCount = 0;

// Globals for managing socket streaming and calibration
std::thread sendThread;			// SenderThread
personalRobotics::MutexBool sendData;	// Data control flag
personalRobotics::EyeKin eyeKin;		// Eyekin object to handle all vision calls
void sendThreadRoutine();				// Thread for sending data
void startStreaming();					// Sends data to currently connected client
void stopStreaming();					// Stops the server and resets the state


void main(int argC, char **argV)
{
	// Do a placeholdercalibration
	sendData.set(false);
	eyeKin.calibrate(true);

	// Look for incoming commands and change the state of the machine
	startStreaming();

	
}


void sendThreadRoutine()
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
void startStreaming()
{
	sendData.set(true);
	sendThread = std::thread(&sendThreadRoutine);
}
void stopStreaming()
{
	sendData.set(false);
	if (sendThread.joinable())
		sendThread.join();
}