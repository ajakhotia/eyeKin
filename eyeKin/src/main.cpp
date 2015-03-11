#include "eyeKin.h"
#include <Windows.h>

// Socket counter for managing loading and unloading of resources
personalRobotics::MutexType<int> personalRobotics::Tcp::socketCount = 0;

// Globals for managing socket streaming and calibration
std::thread sendThread;							// SenderThread
personalRobotics::MutexBool sendData;			// Data control flag
personalRobotics::EyeKin eyeKin;				// Eyekin object to handle all vision calls
void sendRoutine();								// Routine for sending messages to client
bool readMessage(procamPRL::EntityList &list);	// Routine for reading messages from client
void writeMessage(procamPRL::EntityList &list);	// Routine for writing message to client
void startStreaming();							// Sends data to currently connected client
void stopStreaming();							// Stops the server and resets the state

// Global handy commands
procamPRL::EntityList SendDisplayInfoPacket;
procamPRL::EntityList Disconnect;
procamPRL::EntityList CalibrationComplete;

void main(int argC, char **argV)
{
	// Do a placeholdercalibration
	sendData.set(false);
	eyeKin.calibrate();

	// Set the values for global commands
	SendDisplayInfoPacket.set_command(procamPRL::EntityList::SEND_DISPLAY_INFO_PACKET);
	Disconnect.set_command(procamPRL::EntityList::DISCONNECT);
	CalibrationComplete.set_command(procamPRL::EntityList::CALIBRATION_COMPLETE);

	// Look for incoming commands and change the state of the machine
	while (true)
	{
		if (eyeKin.getServer()->isConnected.get())
		{
			procamPRL::EntityList commandList;
			if (readMessage(commandList))
			{
				switch (commandList.command())
				{

				case procamPRL::EntityList::NONE:
				{

				}
				break;

				case procamPRL::EntityList::START_CALIBRATION:
				{
					std::cout << "Stopping stream and starting calibration" << std::endl;
					stopStreaming();
					writeMessage(SendDisplayInfoPacket);
					procamPRL::EntityList temp;
					int counter = 0;
					while (temp.command() != procamPRL::EntityList::SEND_DISPLAY_INFO_PACKET && counter++ < 2)
					{
						temp.Clear();
						readMessage(temp);
					}
					if (temp.command() == procamPRL::EntityList::SEND_DISPLAY_INFO_PACKET)
					{
						const procamPRL::Entity& entity = temp.entitylist(0);
						if (entity.image().width() > 0 && entity.image().height() > 0)
							eyeKin.calibrate(false, entity.image().width(), entity.image().height());
						else
						{
							std::cout << "No display size was sent.\nPerforming a dummy calibration" << std::endl;
							eyeKin.calibrate();
						}
					}
					else
					{
						std::cout << "No display information packet was sent.\nPerforming a dummy calibration" << std::endl;
						eyeKin.calibrate();
					}
					writeMessage(CalibrationComplete);
				}
				break;

				case procamPRL::EntityList::CALIBRATION_COMPLETE:
				{
					std::cout << "Invalid command sent.\nClient cannot send CALIBRATION_COMPLETE message" << std::endl;
				}
				break;

				case procamPRL::EntityList::START_STREAM:
				{
					std::cout << "Starting stream" << std::endl;
					startStreaming();
				}
				break;

				case procamPRL::EntityList::STOP_STREAM:
				{
					std::cout << "Stopping stream" << std::endl;
					stopStreaming();
				}
				break;

				case procamPRL::EntityList::SEND_DISPLAY_INFO_PACKET:
				{
					std::cout << "Sending display info packet" << std::endl;
				}
				break;

				case procamPRL::EntityList::DISCONNECT:
				{
					std::cout << "Disconnecting ..." << std::endl;
					writeMessage(Disconnect);
					stopStreaming();
					eyeKin.getServer()->disconnect();
				}
				break;

				}
			}
		}
		else
		{
			Sleep(25);
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
				if (eyeKin.getSegmentor()->getEntityList()->size() != 0)
				{
					// Generate the list
					procamPRL::EntityList serializableList;
					eyeKin.generateSerializableList(serializableList);

					// Send the message over socket
					writeMessage(serializableList);
				}
			}
			else
			{
				// Expire the frame
				eyeKin.getSegmentor()->newListGenerated.set(false);

				// Stop streaming
				//sendData.set(false);

				// Disconnect
				writeMessage(Disconnect);
			}
		}
		Sleep(25);
	}
}
bool readMessage(procamPRL::EntityList &list)
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
void writeMessage(procamPRL::EntityList &list)
{
	std::string outString;
	list.SerializeToString(&outString);
	int dataLenght = outString.length();
	if (dataLenght > 0)
	{
		int networkOrderDataLength = htonl(dataLenght);
		eyeKin.getServer()->write(4, (char*)&networkOrderDataLength);
		eyeKin.getServer()->write(dataLenght, (char*)outString.c_str());
	}
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