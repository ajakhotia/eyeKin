#include "entity.h"

// Constructor and Destructor
personalRobotics::Entity::Entity()
{

}
personalRobotics::Entity::Entity(cv::Point2f objectCentroid, float objectAngle, cv::Size2f objectBoundingSize, int inID)
{
	pose2Drgb.position = objectCentroid;
	pose2Drgb.angle = objectAngle;
	boundingSize = objectBoundingSize;
	id = inID;
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
	cv::Point2f rgbKeyPoint = pose2Drgb.position + cv::Point2f(cos(pose2Drgb.angle), sin(pose2Drgb.angle));
	std::vector<cv::Point2f> rgbPositionVector, projPositionVector;
	rgbPositionVector.push_back(pose2Drgb.position);
	rgbPositionVector.push_back(rgbKeyPoint);
	cv::perspectiveTransform(rgbPositionVector, projPositionVector, homography);
	pose2Dproj.position = projPositionVector[0];
	cv::Point2f projVect = projPositionVector[1] - projPositionVector[0];
	pose2Dproj.angle = atan2f(projVect.y, projVect.x);

	//Construct the rectangle and extarct patches
	cv::RotatedRect rect(pose2Drgb.position, boundingSize, 180*pose2Drgb.angle/CV_PI);
	cv::Rect boundingRect = rect.boundingRect();
	cv::Mat rgbPatch;
	cv::getRectSubPix(rgbImage, boundingRect.size(), rect.center, rgbPatch);
	cv::Mat rotationMatrix = cv::getRotationMatrix2D(cv::Point2f(boundingRect.width / 2.f, boundingRect.height / 2.f), rect.angle, 1);
	cv::warpAffine(rgbPatch, rgbPatch, rotationMatrix, rgbPatch.size(), cv::INTER_CUBIC);
	cv::getRectSubPix(rgbPatch, boundingSize, cv::Point2f(boundingRect.width / 2.f, boundingRect.height / 2.f), rgbPatch);
	cv::flip(rgbPatch, rgbPatch, 1);

	// Make a mask
	cv::Mat maskChannel, fgdModel, bgdModel, maskCopy, blurPatch;
	cv::Rect yoRect(cv::Point(rect.size.width*0.1, rect.size.height*0.1), cv::Size2f(rect.size.width*0.8, rect.size.height*0.8));
	cv::blur(rgbPatch, blurPatch, cv::Size(5, 5));
	cv::grabCut(blurPatch, maskChannel, yoRect, bgdModel, fgdModel, 2, cv::GC_INIT_WITH_RECT);
	for (int i = 0; i < maskChannel.rows*maskChannel.cols; i++)
	{
		if (maskChannel.data[i] == cv::GC_BGD || maskChannel.data[i] == cv::GC_PR_BGD)
			maskChannel.data[i] = 0;
		else
			maskChannel.data[i] = 255;
	}
	maskCopy = maskChannel.clone();
	std::vector<std::vector<cv::Point>> vectorOfContours;
	cv::vector<cv::Vec4i> hierarchy;
	cv::findContours(maskCopy, vectorOfContours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	int largestContourIdx = -1;
	double largestArea = 0;
	for (int i = 0; i < vectorOfContours.size(); i++)
	{
		double area = cv::contourArea(vectorOfContours[i]);
		if (area > largestArea)
		{
			largestArea = area;
			largestContourIdx = i;
		}
	}
	if (largestContourIdx!= -1)
		cv::approxPolyDP(vectorOfContours[largestContourIdx],contour,5,true);
	cv::Mat mask2 = cv::Mat::zeros(maskChannel.size(),CV_8UC1);
	cv::drawContours(mask2, vectorOfContours, largestContourIdx, cv::Scalar(255), CV_FILLED);
	cv::imshow("disp", mask2);
	cv::rectangle(rgbPatch, yoRect, cv::Scalar(255,0,0,0));
	cv::imshow("disp2", rgbPatch);
	cv::waitKey(20);
	patch.create(rgbPatch.rows, rgbPatch.cols, CV_8UC4);
	int fromTo[] = {0,0, 1,1, 2,2, 3,3};
	cv::Mat inMatArray[] = { rgbPatch, mask2 };
	cv::mixChannels(inMatArray, 2, &patch, 1, fromTo, 4);

	#ifdef DEBUG_PROFILER
		timer.toc();
	#endif
}