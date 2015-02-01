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
		procamPRL::EntityList serializableList;

		// Calibration
		cv::Mat checkerboard;
		int numCheckerPtsX;
		int numCheckerPtsY;
		cv::Mat homography;

		// Counters
		long epoch;

		// Control flags
		MutexBool tablePlaneFound;
		MutexBool homographyFound;
		MutexBool isCalibrating;

		// Locks
		std::mutex serializableListMutex;
		
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

		// Routines
		void generateSerializableList();

		// Control flags
		MutexBool serializableListGenerated;

		// Accessors
		TcpServer* getServer();
		procamPRL::EntityList* getSerializableList();

		// Thread safety methods
		void lockSerializableList();
		void unlockSerializableList();
	};
}

#endif