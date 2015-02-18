#ifndef __PROCAM_UTILTIES_H__
#define __PROCAM_UTILTIES_H__

#include <math.h>
#include "opencv.h"
#include "settings.h"
#include "timer.h"

namespace personalRobotics
{
	struct Pose2Dim
	{
		cv::Point2f position;
		float angle;
	};
	
	class Entity
	{
	public:
		Pose2Dim pose2Dproj;
		Pose2Dim pose2Drgb;
		cv::Size2f boundingSize;
		std::vector<cv::Point2f> boundingCornersProj;
		std::vector<cv::Point2f> boundingCornersRgb;
		std::vector<cv::Point> contour;
		int id;
		cv::Mat patch;
	public:
		Entity();
		Entity(cv::Point2f objectCentroid, float objectAngle, cv::Size2f objectBoundingSize, int inID);
		~Entity();
		void generateData(cv::Mat& homography, cv::Mat& rgbImage);
	};
}
#endif