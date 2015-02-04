#include "eyeKin.h"
#include <Windows.h>

personalRobotics::MutexType<int> personalRobotics::Tcp::socketCount = 0;

void main(int argC, char **argV)
{
	int a;
	// Initialize and calibrate the kinect segmentation system. This also
	// starts a tcp server and listens on port 9000 of the local host.
	personalRobotics::EyeKin eyeKin;
	eyeKin.calibrate();
	// Loop to begin sending data
	while (true)
	{
		if (eyeKin.getServer()->isConnected.get())
		{
			// Allocate space for list
			procamPRL::EntityList serializableList;
			serializableList.set_frameid(7);
			// Generate serializable list
			//eyeKin.generateSerializableList(serializableList);
			//procamPRL::Entity *ent = serializableList.add_entitylist();
			//personalRobotics::Point2D point;
			//point.set_x(5);
			//point.set_y(6);
			
			// Send data over socket
			std::string outString;
			//std::mutex bufferMutex;
			//bufferMutex.lock();
			serializableList.SerializeToString(&outString);
			int dataLenght = outString.length();
			std::cout << dataLenght << std::endl;
			if (dataLenght > 0)
			{
				try
				{

					int networkOrderDataLength = htonl(dataLenght);
					eyeKin.getServer()->write(4, (char*)&networkOrderDataLength);
					//eyeKin.getServer()->asyncSend(dataLenght, &outString[0], &bufferMutex,false,true);
					eyeKin.getServer()->write(dataLenght, (char*)outString.c_str());
				}
				catch (personalRobotics::SocketException e)
				{

				}
			}
		}
	}
	std::cin >> a;
}
