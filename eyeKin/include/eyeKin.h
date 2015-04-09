#ifndef __EYEKIN_H__
#define __EYEKIN_H__

#include "tcp.h"
#include <chrono>
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
		MutexBool segmentorEverStarted;
	
		// Configurations
		int screenWidth;
		int screenHeight;
	public:
		// Constructor and destructor
		EyeKin();
		~EyeKin();
		void reset();

		// Calibration methods
		void findTable();
		void findHomography(bool placeholder);
		void calibrate(bool placeholder=true, int inWidth = DEFAULT_SCREEN_WIDTH, int inHeight = DEFAULT_SCREEN_HEIGHT);

		// Routines
		void generateSerializableList(procamPRL::EntityList &serializableList);

		// Accessors
		TcpServer* getServer();
		ObjectSegmentor* getSegmentor();
	};
}

#endif