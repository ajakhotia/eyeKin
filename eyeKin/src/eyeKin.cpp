#include "eyeKin.h"

personalRobotics::EyeKin::EyeKin()
{
	// Configuration
	screenWidth = DEFAULT_SCREEN_WIDTH;
	screenHeight = DEFAULT_SCREEN_HEIGHT;

	// Useful Images
	numCheckerPtsX = 0;
	numCheckerPtsY = 0;
	personalRobotics::createCheckerboard(checkerboard, screenWidth, screenHeight, numCheckerPtsX, numCheckerPtsY);

	//Flags
	tablePlaneFound = false;
	homographyFound = false;
	isCalibrating = true;
}

personalRobotics::EyeKin::~EyeKin()
{

}

void personalRobotics::EyeKin::findTable()
{
	tablePlaneFound = segmentor.findTablePlane();
}

void personalRobotics::EyeKin::findHomography()
{
	std::vector<cv::Point2f> detectedCorners, checkerboardCorners;
	bool foundCorners = findChessboardCorners(*segmentor.getColorImagePtr(), cv::Size(numCheckerPtsX, numCheckerPtsY), detectedCorners, CV_CALIB_CB_ADAPTIVE_THRESH);
	bool foundProjectedCorners = findChessboardCorners(checkerboard, cv::Size(numCheckerPtsX, numCheckerPtsY), checkerboardCorners, CV_CALIB_CB_ADAPTIVE_THRESH);
	if (foundCorners)
	{
		cv::Mat invertedHomography = cv::findHomography(detectedCorners, checkerboardCorners, CV_RANSAC);
		cv::Mat homographyCorrection = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, -1, (DEFAULT_SCREEN_HEIGHT), 0, 0, 1);
		homography = homographyCorrection*invertedHomography;
		homographyFound = true;
	}
}

void personalRobotics::EyeKin::calibrate()
{
	if (!tablePlaneFound && segmentor.isDepthAllocated)
		findTable();
	if (!homographyFound && segmentor.isColorAllocated)
		findHomography();
	if (tablePlaneFound && homographyFound)
	{
		isCalibrating = false;
		segmentor.setHomography(homography);
		segmentor.startSegmentor();
	}
}