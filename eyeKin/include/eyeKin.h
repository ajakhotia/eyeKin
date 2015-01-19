#ifndef __EYEKIN_H__
#define __EYEKIN_H__

#include "objectSegmentation.h"

namespace personalRobotics
{
	class EyeKin
	{
	protected:
		// Interface
		ObjectSegmentor segmentor;
		
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