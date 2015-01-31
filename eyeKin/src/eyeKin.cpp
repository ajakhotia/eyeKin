#include "eyeKin.h"

personalRobotics::EyeKin::EyeKin() : tcpServer(PORT1, ADDRESS_FAMILY)
{
	// Configuration
	screenWidth = DEFAULT_SCREEN_WIDTH;
	screenHeight = DEFAULT_SCREEN_HEIGHT;

	// Start the server
	tcpServer.start();

	// Useful Images
	numCheckerPtsX = 0;
	numCheckerPtsY = 0;
	personalRobotics::createCheckerboard(checkerboard, screenWidth, screenHeight, numCheckerPtsX, numCheckerPtsY);

	//Flags
	tablePlaneFound = false;
	homographyFound = false;
	isCalibrating = true;
	newSerializableListGeneratedFlagMutex.lock();
	newSerializableListGeneratedFlag = false;
	newSerializableListGeneratedFlagMutex.unlock();

	// Counters
	epoch = 0;
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
	/*std::vector<cv::Point2f> detectedCorners, checkerboardCorners;
	bool foundCorners = findChessboardCorners(*segmentor.getColorImagePtr(), cv::Size(numCheckerPtsX, numCheckerPtsY), detectedCorners, CV_CALIB_CB_ADAPTIVE_THRESH);
	bool foundProjectedCorners = findChessboardCorners(checkerboard, cv::Size(numCheckerPtsX, numCheckerPtsY), checkerboardCorners, CV_CALIB_CB_ADAPTIVE_THRESH);
	if (foundCorners)
	{
		cv::Mat invertedHomography = cv::findHomography(detectedCorners, checkerboardCorners, CV_RANSAC);
		cv::Mat homographyCorrection = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, -1, screenHeight, 0, 0, 1);
		homography = homographyCorrection*invertedHomography;
		homographyFound = true;
	}*/
	homography = (cv::Mat_<double>(3, 3) <<1.7456, 0.0337, -837.4711, -0.0143, -1.7469, 1411.6242, -3.04089, 0.0000, 1) * (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
	homographyFound = true;
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

void personalRobotics::EyeKin::generateSerializableList()
{
	if (segmentor.newListGenerated())
	{
		// Set the non-serializable list to be old
		segmentor.unsetNewListGeneratedFlag();

		// Set the serializable list as new
		newSerializableListGeneratedFlagMutex.lock();
		newSerializableListGeneratedFlag = true;
		newSerializableListGeneratedFlagMutex.unlock();

		// Lock the lists
		segmentor.lockList();
		lockSerializableList();

		// Get access to entityList
		std::vector<personalRobotics::Entity> *listPtr = segmentor.getEntityList();

		// Clear the serializableList
		serializableList.Clear();

		// Copy information from the entity list
		serializableList.set_frameid(++epoch);
		serializableList.set_timestamp(0);
		
		for (std::vector<personalRobotics::Entity>::iterator entityPtr = listPtr->begin(); entityPtr != listPtr->end(); entityPtr++)
		{
			procamPRL::Entity* serializableEntityPtr = serializableList.add_entitylist();
			// Set pose
			personalRobotics::Pose2D *pose = new personalRobotics::Pose2D();
			personalRobotics::Point2D *position = new personalRobotics::Point2D();
			position->set_x(entityPtr->pose2Dproj.position.x);
			position->set_y(entityPtr->pose2Dproj.position.y);
			pose->set_angle(entityPtr->pose2Dproj.angle);
			serializableEntityPtr->set_allocated_pose(pose);

			// Set bounding size
			personalRobotics::Point2D *boundingSize = new personalRobotics::Point2D();
			boundingSize->set_x(entityPtr->boundingSize.width);
			boundingSize->set_y(entityPtr->boundingSize.height);
			serializableEntityPtr->set_allocated_boundingsize(boundingSize);

			// Set pixel size
			personalRobotics::Point2D *pixelSize = new personalRobotics::Point2D();
			pixelSize->set_x(1);
			pixelSize->set_y(1);

			// Set contours
			for (std::vector<cv::Point>::iterator contourPointPtr = entityPtr->contour.begin(); contourPointPtr != entityPtr->contour.end(); contourPointPtr++)
			{
				personalRobotics::Point2D *serializableContourPtr = serializableEntityPtr->add_contours();
				serializableContourPtr->set_x(contourPointPtr->x);
				serializableContourPtr->set_y(contourPointPtr->y);
			}

			// Set Image
			procamPRL::Entity::Image *imagePtr = new procamPRL::Entity::Image();
			imagePtr->set_height(entityPtr->patch.rows);
			imagePtr->set_width(entityPtr->patch.cols);
			imagePtr->set_data((void*)entityPtr->patch.data, entityPtr->patch.channels() * entityPtr->patch.rows * entityPtr->patch.cols);
			serializableEntityPtr->set_allocated_image(imagePtr);
		}

		// Unlock the lists
		unlockSerializableList();
		segmentor.unlockList();
	}
}

personalRobotics::TcpServer* personalRobotics::EyeKin::getServer()
{
	return &tcpServer;
}

procamPRL::EntityList* personalRobotics::EyeKin::getSerializableList()
{
	return &serializableList;
}

void personalRobotics::EyeKin::lockSerializableList()
{
	serializableListMutex.lock();
}

void personalRobotics::EyeKin::unlockSerializableList()
{
	serializableListMutex.unlock();
}

void personalRobotics::EyeKin::lockReadBuffer()
{
	readBufferMutex.lock();
}

void personalRobotics::EyeKin::unlockReadBuffer()
{
	readBufferMutex.unlock();
}

void personalRobotics::EyeKin::lockWriteBuffer()
{
	writeBufferMutex.lock();
}

void personalRobotics::EyeKin::unlockWriteBuffer()
{
	writeBufferMutex.unlock();
}