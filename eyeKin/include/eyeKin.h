#ifndef __EYEKIN_H__
#define __EYEKIN_H__

#include "tcp.h"
#include "entity.pb.h"
#include "objectSegmentation.h"

#define PORT1 9000

namespace personalRobotics
{
	class EyeKin
	{
	protected:
		// Interface
		ObjectSegmentor segmentor;
		TcpServer tcpServer;
		char *readBuffer;
		char *writeBuffer;
		procamPRL::EntityList serializableList;

		// Calibration
		cv::Mat checkerboard;
		int numCheckerPtsX;
		int numCheckerPtsY;
		cv::Mat homography;

		// Counters
		long epoch;

		// Control flags
		bool tablePlaneFound;
		bool homographyFound;
		bool isCalibrating;
		bool newSerializableListGeneratedFlag;

		// Locks
		boost::mutex serializableListMutex;
		boost::mutex readBufferMutex;
		boost::mutex writeBufferMutex;
		boost::mutex newSerializableListGeneratedFlagMutex;
		
		// Configurations
		int screenWidth;
		int screenHeight;
	public:
		// Constructor and destructor
		EyeKin();
		~EyeKin();

		// Calibration methods
		void findTable();
		void findHomography();
		void calibrate();

		// Conversion methods
		void generateSerializableList();

		// Accessors
		TcpServer* getServer();
		procamPRL::EntityList* getSerializableList();

		// Lock accessors
		void lockSerializableList();
		void unlockSerializableList();
		void lockReadBuffer();
		void unlockReadBuffer();
		void lockWriteBuffer();
		void unlockWriteBuffer();
		bool newSerializableListGenerated()
		{
			bool returnFlag;
			newSerializableListGeneratedFlagMutex.lock();
			returnFlag = newSerializableListGeneratedFlag;
			newSerializableListGeneratedFlagMutex.unlock();
			return returnFlag;
		}
		void unsetNewSerializableListGeneratedFlag()
		{
			newSerializableListGeneratedFlagMutex.lock();
			newSerializableListGeneratedFlag = false;
			newSerializableListGeneratedFlagMutex.unlock();
		}
	};
}

#endif