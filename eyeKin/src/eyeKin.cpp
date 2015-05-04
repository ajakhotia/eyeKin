#include "eyeKin.h"
#include <Windows.h>

// Constructor and Destructor
personalRobotics::EyeKin::EyeKin() : tcpServer(PORT1, ADDRESS_FAMILY)
{
	// Start the server
	tcpServer.start();

	//Flags
	tablePlaneFound.set(false);
	homographyFound.set(false);
	isCalibrating.set(true);
	segmentorEverStarted.set(false);

	// Counters
	epoch = 0;
}
personalRobotics::EyeKin::~EyeKin()
{

}
void personalRobotics::EyeKin::reset()
{
	// Log
	std::cout << "Resetting eyekin by pausing segmentor and resetting all calbration flags" << std::endl;
	// Stop segmentor
	segmentor.pauseSegmentor();

	// Reset the flags
	tablePlaneFound.set(false);
	homographyFound.set(false);
	isCalibrating.set(true);

	// Reset the counter
	epoch = 0;
}


// Calibration methods
void personalRobotics::EyeKin::findTable()
{
	if(segmentor.findTablePlane())
		tablePlaneFound.set(true);
}
void personalRobotics::EyeKin::findHomography(bool placeholder)
{
	if (!placeholder)
	{
		std::vector<cv::Point2f> detectedCorners, checkerboardCorners;
		segmentor.rgbMutex.lock();
		bool foundCorners = findChessboardCorners(*segmentor.getColorImagePtr(), cv::Size(numCheckerPtsX, numCheckerPtsY), detectedCorners, CV_CALIB_CB_ADAPTIVE_THRESH);
		segmentor.rgbMutex.unlock();
		std::cout << "Size of checkerboard being used in findHomography is: (" << checkerboard.cols << ", " << checkerboard.rows << ")" << std::endl;
		bool foundProjectedCorners = findChessboardCorners(checkerboard, cv::Size(numCheckerPtsX, numCheckerPtsY), checkerboardCorners, CV_CALIB_CB_ADAPTIVE_THRESH);

		if (foundCorners && foundProjectedCorners)
		{
			homography = cv::findHomography(detectedCorners, checkerboardCorners, CV_RANSAC);
			homographyFound.set(true);
		}
		else
		{
			std::cout << "Failed to find chessboard corners in the image.\n";
		}
	}
	else
	{
		std::cout << "Performing a placeholder calibration" << std::endl;
		homography = (cv::Mat_<double>(3, 3) << 1.7456, 0.0337, -837.4711, 0.0143, 1.7469, -331.6242, 0.0, 0.0, 1) * (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
		homographyFound.set(true);
	}
}
void personalRobotics::EyeKin::calibrate(bool placeholder, int inWidth, int inHeight)
{
	// Reset
	reset();
	
	// Setup
	screenWidth = inWidth;
	screenHeight = inHeight;
	numCheckerPtsX = 0;
	numCheckerPtsY = 0;
	personalRobotics::createCheckerboard(checkerboard, screenWidth, screenHeight, numCheckerPtsX, numCheckerPtsY);
	
	// Calibration
	int attempts = 0;
	int maxAttempts = 5;
	std::cout << "Starting calibration with\nwidth: " << screenWidth << " height: " << screenHeight << " numXCorners: " << numCheckerPtsX << " numYCorners: " << numCheckerPtsY << std::endl;
	while (isCalibrating.get())
	{	    
	  if (!tablePlaneFound.get() && segmentor.isDepthAllocated.get()) {
	         std::cout << "calling findTable()" << std::endl;
			findTable();
	  }				

		if (!homographyFound.get() && segmentor.isColorAllocated.get())
		{
		  	    std::cout << "calling findHomography()" << std::endl;
			findHomography(placeholder);
			attempts++;
			if (attempts > maxAttempts)
			{
				std::cout << "Calibration failed " << attempts-1 << " times. Performing a placeholder cailbration.\n";
				findHomography(true);
			}
		}
		

		if (tablePlaneFound.get() && homographyFound.get())
		{
			isCalibrating.set(false);
			segmentor.setHomography(homography, screenWidth, screenHeight);

			// Map size of rgb and projector pixels
			std::vector<cv::Point2f> rgbKeyPoints;
			std::vector<cv::Point2f> projKeyPoints;
			rgbKeyPoints.push_back(cv::Point2f(0, 0));
			rgbKeyPoints.push_back(cv::Point2f(10, 0));
			rgbKeyPoints.push_back(cv::Point2f(0, 10));
			cv::perspectiveTransform(rgbKeyPoints, projKeyPoints, homography);
			float delX = 10.f / cv::norm(projKeyPoints[1] - projKeyPoints[0]);
			float delY = 10.f / cv::norm(projKeyPoints[2] - projKeyPoints[0]);
			colorPixelSize = *(segmentor.getRGBpixelSize());
			projPixelSize.x = colorPixelSize.x * delX;
			projPixelSize.y = colorPixelSize.y * delY;
			std::cout << "Calibration complete. Projector pixel size is: (" << projPixelSize.x << "," << projPixelSize.y << ")\n";
			if (segmentorEverStarted.get())
			{
				segmentor.resumeSegmentor();
			}
			else
			{
				segmentorEverStarted.set(true);
				segmentor.startSegmentor();
			}
			
		}
		else
			Sleep(25);
	}
}

// Routines
void personalRobotics::EyeKin::generateSerializableList(procamPRL::EntityList &serializableList)
{
	if (segmentor.newListGenerated.get())
	{
		// Lock the lists
		segmentor.lockList();

		// Set the non-serializable list to be old
		segmentor.newListGenerated.set(false);

		// Get access to entityList
		std::vector<personalRobotics::Entity> *listPtr = segmentor.getEntityList();

		// Set frame ids, time stamp and the pixel sizes
		serializableList.set_frameid(++epoch);
		serializableList.set_timestamp(((double)std::chrono::system_clock::now().time_since_epoch().count())/1000000.f);
		
		// Logging
		std::cout << "A new frame created with frame id: " << epoch << " and timestamp: " << serializableList.timestamp() << " with following entity attributes:" << std::endl;
		int counter = 0;

		for (std::vector<personalRobotics::Entity>::iterator entityPtr = listPtr->begin(); entityPtr != listPtr->end(); entityPtr++,counter++)
		{
			// Add an serializable entity to serializable entity list
			procamPRL::Entity* serializableEntityPtr = serializableList.add_entitylist();
			
			// Set pose
			personalRobotics::Pose2D *pose = new personalRobotics::Pose2D();
			personalRobotics::Point2D *position = new personalRobotics::Point2D();
			position->set_x(entityPtr->pose2Dproj.position.x);
			position->set_y(entityPtr->pose2Dproj.position.y);
			pose->set_angle(entityPtr->pose2Dproj.angle);
			pose->set_allocated_position(position);
			serializableEntityPtr->set_allocated_pose(pose);

			// Set bounding size
			personalRobotics::Point2D *boundingSize = new personalRobotics::Point2D();
			boundingSize->set_x(entityPtr->boundingSize.width);
			boundingSize->set_y(entityPtr->boundingSize.height);
			serializableEntityPtr->set_allocated_boundingsize(boundingSize);

			// Set ID
			serializableEntityPtr->set_id(entityPtr->id);

			// Set contours
			for (std::vector<cv::Point>::iterator contourPointPtr = entityPtr->contour.begin(); contourPointPtr != entityPtr->contour.end(); contourPointPtr++)
			{
				personalRobotics::Point2D *serializableContourPtr = serializableEntityPtr->add_contours();
				serializableContourPtr->set_x(contourPointPtr->x);
				serializableContourPtr->set_y(contourPointPtr->y);
			}

			// Set pixel size
			personalRobotics::Point2D *rgbPixelSizeSerializable = new personalRobotics::Point2D();
			personalRobotics::Point2D *projPixelSizeSerializable = new personalRobotics::Point2D();
			rgbPixelSizeSerializable->set_x(colorPixelSize.x);
			rgbPixelSizeSerializable->set_y(colorPixelSize.y);
			projPixelSizeSerializable->set_x(projPixelSize.x);
			projPixelSizeSerializable->set_y(projPixelSize.y);
			serializableEntityPtr->set_allocated_projpixelsize(projPixelSizeSerializable);

			// Set Image
			procamPRL::Entity::Image *image = new procamPRL::Entity::Image();
			image->set_height(entityPtr->patch.rows);
			image->set_width(entityPtr->patch.cols);
			image->set_allocated_rgbpixelsize(rgbPixelSizeSerializable);
			image->set_data((void*)entityPtr->patch.data, entityPtr->patch.channels() * entityPtr->patch.rows * entityPtr->patch.cols);
			serializableEntityPtr->set_allocated_image(image);

			std::cout << "\t" << counter << ": " << "Pose: (" << pose->position().x() << ", " << pose->position().y()<< ") , " << "Image size: (" << image->width() << ", " << image->height() << ") " << std::endl;
		}

		// Unlock the lists
		segmentor.unlockList();
	}
}

// Accessors
personalRobotics::TcpServer* personalRobotics::EyeKin::getServer()
{
	return &tcpServer;
}
personalRobotics::ObjectSegmentor* personalRobotics::EyeKin::getSegmentor()
{
	return &segmentor;
}
