#include "eyeKin.h"
#include <Windows.h>

personalRobotics::MutexType<int> personalRobotics::Tcp::socketCount = 0;



void main(int argC, char **argV)
{
	int a;							// Dummy variable
	std::thread socketSendThread;	// Reader thread to get commands from the client
	bool sendData = true;					// Data control flag

	// Initialize and calibrate the kinect segmentation system. This also
	// starts a tcp server and listens on port 9000 of the local host.
	personalRobotics::EyeKin eyeKin;
	eyeKin.calibrate();
	// Loop to begin sending data
	while (sendData)
	{
		if (eyeKin.getServer()->isConnected.get() && eyeKin.getSegmentor()->newListGenerated.get())
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
				//eyeKin.getServer()->asyncSend(dataLenght, &outString[0], &bufferMutex,false,true);
				eyeKin.getServer()->write(dataLenght, (char*)outString.c_str());
			}
		}
		Sleep(50);
	}
	std::cin >> a;
}
