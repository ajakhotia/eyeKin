#define _CRT_SECURE_NO_WARNINGS
#include "eyeKin.h"
#include <Windows.h>
#include <iostream>
#include <fstream>

// Socket counter for managing loading and unloading of resources
personalRobotics::MutexType<int> personalRobotics::Tcp::socketCount = 0;

// Globals for managing socket streaming and calibration
std::thread sendThread;																// SenderThread
personalRobotics::MutexBool stream;													// Controls streaming of data over network
personalRobotics::MutexBool runSenderThread;										// Send thread control flag
void sendRoutine(personalRobotics::EyeKin *eyeKin);									// Routine for sending messages to client
bool readMessage(procamPRL::EntityList &list, personalRobotics::EyeKin *eyeKin);	// Routine for reading messages from client
void writeMessage(procamPRL::EntityList &list, personalRobotics::EyeKin *eyeKin);	// Routine for writing message to client
void startSenderThread(personalRobotics::EyeKin *eyeKin);							// Sends data to currently connected client
void stopSenderThread();															// Stops the server and resets the state
BOOL controlEventsHandler(DWORD wtf);
bool controlFlag;

// Global handy commands
procamPRL::EntityList SendDisplayInfoPacket;
procamPRL::EntityList Disconnect;
procamPRL::EntityList CalibrationComplete;

void main(int argC, char **argV)
{
	personalRobotics::EyeKin eyeKin;				// Eyekin object to handle all vision calls
	
	// Generate a log file to record messages and point cout to it.
	time_t t = time(0);
	struct tm *now = localtime(&t);
	std::string name = "c:\\Temp\\eyeKin-" + personalRobotics::full_date_string() + ".log";
	std::cout << "Opening eyeKin log file and redirecting output to it: " << name << std::endl;
	std::ofstream logfile( name );	// create a log file
	logfile.rdbuf()->pubsetbuf(0, 0);					// set log file output to unbuffered mode
	std::cout.rdbuf(logfile.rdbuf());					// redirect cout to logfile
	std::cout << "Opened eyeKin log file.\n";
	std::cout << "cout now redirected to log file.\n";
	std::cerr.rdbuf(logfile.rdbuf());                // redirect cerr to logfile
	std::cout << "cerr now redirected to log file.\n";
  
	// Do a placeholdercalibration
	stream.set(false);
	runSenderThread.set(false);
	eyeKin.calibrate();

	// Start the sender thread. This doesn't mean that the data is actually being sent
	startSenderThread(&eyeKin);

	// Set the values for global commands
	SendDisplayInfoPacket.set_command(procamPRL::EntityList::SEND_DISPLAY_INFO_PACKET);
	Disconnect.set_command(procamPRL::EntityList::DISCONNECT);
	CalibrationComplete.set_command(procamPRL::EntityList::CALIBRATION_COMPLETE);

	controlFlag = true;
	SetConsoleCtrlHandler((PHANDLER_ROUTINE)controlEventsHandler, TRUE);
	int counter = 0;

	// Look for incoming commands and change the state of the machine
	while (controlFlag)
	{
		if (eyeKin.getServer()->isConnected.get())
		{
			procamPRL::EntityList commandList;
			if (readMessage(commandList,&eyeKin))
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
					stream.set(false);
					std::cout << "Requesting the display size information" << std::endl;
					writeMessage(SendDisplayInfoPacket, &eyeKin);
					procamPRL::EntityList temp;
					int counter = 0;
					while (temp.command() != procamPRL::EntityList::SEND_DISPLAY_INFO_PACKET && counter++ < 2)
					{
						temp.Clear();
						std::cout << "Waitiing to read message with display information" << std::endl;
						readMessage(temp, &eyeKin);
					}
					if (temp.command() == procamPRL::EntityList::SEND_DISPLAY_INFO_PACKET)
					{
						std::cout << "Received message with display information" << std::endl;
						const procamPRL::Entity& entity = temp.entitylist(0);
						if (entity.image().width() > 0 && entity.image().height() > 0)
							eyeKin.calibrate(false, entity.image().width(), entity.image().height());
						else
						{
							std::cout << "No display size was sent. Performing a dummy calibration" << std::endl;
							eyeKin.calibrate();
						}
					}
					else
					{
						std::cout << "No display information packet was sent.\nPerforming a dummy calibration" << std::endl;
						eyeKin.calibrate();
					}
					std::cout << "Completed calibartion and notifying the client of the same." << std::endl;
					writeMessage(CalibrationComplete, &eyeKin);
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
					stream.set(true);
				}
				break;

				case procamPRL::EntityList::STOP_STREAM:
				{
					std::cout << "Stopping stream" << std::endl;
					stream.set(false);
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
					stream.set(false);
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
	
	std::cout << "Stopping all threads and exiting" << std::endl;
	exit(0);
}

BOOL controlEventsHandler(DWORD wtf)
{
	switch (wtf)
	{
	case CTRL_C_EVENT:
		std::cout << "ctrl + c received" << std::endl;
		stopSenderThread();
		controlFlag = false;
		break;
	case CTRL_CLOSE_EVENT:
		std::cout << "close received" << std::endl;
		stopSenderThread();
		controlFlag = false;
		std::cout << "about to break" << std::endl;
		break;
	case CTRL_BREAK_EVENT:
		std::cout << "close received" << std::endl;
		stopSenderThread();
		controlFlag = false;
		break;
	case CTRL_LOGOFF_EVENT:
		std::cout << "close received" << std::endl;
		stopSenderThread();
		controlFlag = false;
		break;
	case CTRL_SHUTDOWN_EVENT:
		std::cout << "close received" << std::endl;
		stopSenderThread();
		controlFlag = false;
		break;
	}
	std::cout << "broke" << std::endl;
	return TRUE;
}

void sendRoutine(personalRobotics::EyeKin *eyeKin)
{
	while (runSenderThread.get())
	{
		if (stream.get())
		{
			if (eyeKin->getServer()->isConnected.get())
			{
				if (eyeKin->getSegmentor()->newListGenerated.get())
				{
					if (eyeKin->getSegmentor()->getEntityList()->size() != 0)
					{
						// Generate the list
						procamPRL::EntityList serializableList;
						eyeKin->generateSerializableList(serializableList);

						// Send the message over socket
						writeMessage(serializableList, eyeKin);
					}
				}
			}
			else
			{
				// Expire the frame
				eyeKin->getSegmentor()->newListGenerated.set(false);

				// Stop the stream
				std::cout << "Client disconnected. Stopping the stream" << std::endl;
				stream.set(false);
			}
		}
		Sleep(15);
	}
}
bool readMessage(procamPRL::EntityList &list, personalRobotics::EyeKin *eyeKin)
{
	// Clear the list
	list.Clear();

	// Read 4 bytes, cast to int and convert to host byte ordering
	char sizeBuffer[4];
	if (eyeKin->getServer()->read(4, sizeBuffer))
	{
		int size = ntohl(*((int*)(sizeBuffer)));
		std::string messageString;
		messageString.reserve(size);
		eyeKin->getServer()->read(size, &messageString[0]);
		list.ParseFromArray((void*)&messageString[0], size);
		return true;
	}
	else
		return false;
}
void writeMessage(procamPRL::EntityList &list, personalRobotics::EyeKin *eyeKin)
{
	std::string outString;
	list.SerializeToString(&outString);
	int dataLenght = outString.length();
	if (dataLenght > 0)
	{
		int networkOrderDataLength = htonl(dataLenght);
		eyeKin->getServer()->write(4, (char*)&networkOrderDataLength);
		eyeKin->getServer()->write(dataLenght, (char*)outString.c_str());
	}
}
void startSenderThread(personalRobotics::EyeKin *eyeKin)
{
	runSenderThread.set(true);
	sendThread = std::thread(&sendRoutine,eyeKin);
}
void stopSenderThread()
{
	runSenderThread.set(false);
	if (sendThread.joinable())
		sendThread.join();
	std::cout << "done deleting senderthread" << std::endl;
}
