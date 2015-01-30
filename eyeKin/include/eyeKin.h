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
		procamPRL::EntityList list;

		// Calibration
		cv::Mat checkerboard;
		int numCheckerPtsX;
		int numCheckerPtsY;
		cv::Mat homography;

		// Control flags
		bool tablePlaneFound;
		bool homographyFound;
		bool isCalibrating;

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
	};
}

#endif