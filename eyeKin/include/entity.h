#ifndef __PROCAM_UTILTIES_H__
#define __PROCAM_UTILTIES_H__

#include <math.h>
#include "opencv.h"
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
		~Entity();
		void generateData(cv::Mat& homography, cv::Mat& rgbImage);
	};
}
#endif