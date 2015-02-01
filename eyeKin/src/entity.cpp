#include "entity.h"

// Constructor and Destructor
personalRobotics::Entity::Entity()
{

}
personalRobotics::Entity::Entity(cv::Point2f centroid, float angle, float xLength, float yLenght)
{
	pose2Drgb.position = centroid;
	pose2Drgb.angle = 180.f*angle / ((float)(CV_PI));
	boundingSize = cv::Size2f(xLength, yLenght);
}
personalRobotics::Entity::~Entity()
{

}

// Routines
void personalRobotics::Entity::generateData(cv::Mat& homography, cv::Mat& rgbImage)
{
	//Debug
	#ifdef DEBUG_PROFILER
		Timer timer("generateData()");
	#endif
	// Obtain pose in projector space
	cv::Point2f rgbKeyPoint = pose2Drgb.position + cv::Point2f(cos(pose2Drgb.angle),sin(pose2Drgb.angle));
	cv::Point2f projKeyPoint,projVect;
	cv::perspectiveTransform(std::vector<cv::Point2f>(&(pose2Drgb.position), &(pose2Drgb.position) + 1), std::vector<cv::Point2f>(&(pose2Dproj.position), &(pose2Dproj.position) + 1), homography);
	cv::perspectiveTransform(std::vector<cv::Point2f>(&(rgbKeyPoint), &(rgbKeyPoint)+1), std::vector<cv::Point2f>(&(projKeyPoint), &(projKeyPoint)+1), homography);
	projVect = projKeyPoint - pose2Dproj.position;
	pose2Dproj.angle = atan2(projVect.y, projVect.x);

	//Construct the rectangle and extarct patches
	cv::Mat rgbPatch;
	cv::RotatedRect rect(pose2Drgb.position, boundingSize, pose2Drgb.angle);
	cv::Rect boundingRect = rect.boundingRect();
	cv::getRectSubPix(rgbImage, boundingRect.size(), rect.center, rgbPatch);
	cv::Mat rotationMatrix = cv::getRotationMatrix2D(cv::Point2f(boundingRect.width / 2.f, boundingRect.height / 2.f), rect.angle, 1);
	cv::warpAffine(rgbPatch, rgbPatch, rotationMatrix, rgbPatch.size(), cv::INTER_CUBIC);
	cv::getRectSubPix(rgbPatch, boundingSize, cv::Point2f(boundingRect.width / 2.f, boundingRect.height / 2.f), rgbPatch);
		
	//Generate bounding points in RGB and prjector space
	cv::Point2f points[4];
	rect.points(points);
	boundingCornersRgb = std::vector<cv::Point2f>(points, points + sizeof(points) / sizeof(cv::Point2f));
	cv::perspectiveTransform(boundingCornersRgb, boundingCornersProj, homography);

	//Perform contour finding and filling
	cv::Mat gray, cannyOut, mask;
	mask = cv::Mat::zeros(rgbPatch.rows, rgbPatch.cols, CV_8UC1);
	cv::cvtColor(rgbPatch, gray, cv::COLOR_BGR2GRAY);
	cv::blur(gray, gray, cv::Size(3, 3));
	cv::Canny(gray, cannyOut, DEFAULT_CANNY_LOW_THRESHOLD, DEFAULT_CANNY_HIGH_THRESHOLD, DEFAULT_CANNY_KERNEL_SIZE, false);
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(cannyOut, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	int largestContourIdx = 0;
	double largestArea = 0;
	for (int i = 0; i < contours.size(); i++)
	{
		double area = cv::contourArea(contours[i]);
		if (area > largestArea)
		{
			largestArea = area;
			largestContourIdx = i;
		}

	}
	cv::drawContours(mask, contours, largestContourIdx, cv::Scalar(255), CV_FILLED);
	contour = contours[largestContourIdx];
	patch = cv::Mat(rgbPatch.rows, rgbPatch.cols, CV_8UC4);
	cv::Mat tempPatch;
	rgbPatch.copyTo(tempPatch, mask);
	int fromTo[] = {0,0, 1,1, 2,2, 3,3};
	cv::Mat inMatArray[] = {tempPatch, mask};
	cv::mixChannels(inMatArray, 2, &patch, 1, fromTo, 4);
	
	#ifdef DEBUG_PROFILER
		timer.toc();
	#endif
}