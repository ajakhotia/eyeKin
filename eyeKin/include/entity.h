#ifndef __PROCAM_UTILTIES_H__
#define __PROCAM_UTILTIES_H__

#include <iostream>
#include <string.h>
#include <math.h>
#include "ofmain.h"
#include "Kinect.h"
#include "opencv2/opencv.hpp"
#include "opencv2/opencv_modules.hpp"
#include "pcl.h"
#include "settings.h"
#include "timer.h"

namespace personalRobotics
{
	struct Pose2D
	{
		cv::Point2f position;
		float angle;
	};
	class Entity
	{
	public:
		Pose2D pose2Dproj;
		Pose2D pose2Drgb;
		cv::Size2f boundingSize;
		std::vector<cv::Point2f> boundingCornersProj;
		std::vector<cv::Point2f> boundingCornersRgb;
		cv::Mat patch;
	public:
		Entity();
		Entity(cv::Point2f centroid, float angle, float xLenght, float yLenght);
		void generateData(cv::Mat& homography, cv::Mat& rgbImage);
		void drawBoundingBox();
		~Entity();
	};
}
#endif