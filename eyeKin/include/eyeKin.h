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

		// Calibration
		cv::Mat checkerboard;
		int numCheckerPtsX;
		int numCheckerPtsY;
		cv::Mat homography;
		cv::Point2f projPixelSize;
		cv::Point2f colorPixelSize;

		// Counters
		long epoch;

		// Control flags
		MutexBool tablePlaneFound;
		MutexBool homographyFound;
		MutexBool isCalibrating;
	
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
		void generateSerializableList(procamPRL::EntityList &serializableList);

		// Accessors
		TcpServer* getServer();
		ObjectSegmentor* getSegmentor();
	};
}

#endif