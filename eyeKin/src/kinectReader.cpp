#include "kinectReader.h"

// Constructors & Destructors
personalRobotics::KinectReader::KinectReader() :	kinectPtr(NULL),
													readerPtr(NULL),
													coordinateMapperPtr(NULL),
													pointCloudPtr(NULL)
{
	// Unset all flags
	stopKinectFlag.set(true);
	newFrameArrived.set(false);
	isColorAllocated.set(false);
	isDepthAllocated.set(false);
	isIRallocated.set(false);

	// Load a dummy image
	dummy = cv::imread("../data/dummy.jpg", CV_LOAD_IMAGE_COLOR);

	// Initialize the sensor
	HRESULT hrGDKS = GetDefaultKinectSensor(&kinectPtr);
	if (SUCCEEDED(hrGDKS))
	{
		HRESULT hrKO = kinectPtr->Open();
		if (SUCCEEDED(hrKO))
		{
			HRESULT hrGCM = kinectPtr->get_CoordinateMapper(&coordinateMapperPtr);
			HRESULT hrOMSFR = kinectPtr->OpenMultiSourceFrameReader(FrameSourceTypes::FrameSourceTypes_Color | FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Infrared, &readerPtr);
			if (FAILED(hrOMSFR))
			{
				std::cout << "Failed to open a frame reader. Exiting application" << std::endl;
				exit(-1);
			}
		}
		else
		{
			std::cout << "Failed to open kinect. Exiting application" << std::endl;
			exit(-1);
		}
	}
}
personalRobotics::KinectReader::~KinectReader()
{
	safeRelease(readerPtr);
	if (kinectPtr)
	{
		kinectPtr->Close();
	}
	safeRelease(kinectPtr);
	
	// Clear the heap
	delete[] pointCloudPtr;
	delete[] depth2colorMappingPtr;
}

// Routines
void personalRobotics::KinectReader::mapInfraredToColor(std::vector<cv::Point> &infraredPoints, std::vector<cv::Point> &colorPoints, ColorSpacePoint *mapping)
{
	if (isDepthAllocated.get())
	{
		colorPoints.clear();
		int colorPointCounter = 0;
		for (std::vector<cv::Point>::iterator irPtr = infraredPoints.begin(); irPtr != infraredPoints.end(); irPtr++)
		{
			if (irPtr->x >= 0 && irPtr->x <= depthWidth && irPtr->y >= 0 && irPtr->y <= depthHeight)
			{
				int indexTL = (int)irPtr->x + depthWidth* ((int)irPtr->y); // Real thing to do is to interpolate bilinearly
				if (mapping[indexTL].X >= 0 && mapping[indexTL].X < rgbWidth && mapping[indexTL].Y >= 0 && mapping[indexTL].Y < rgbHeight)
					colorPoints.push_back(cv::Point(mapping[indexTL].X, mapping[indexTL].Y));
			}
		}
	}
}
void personalRobotics::KinectReader::pollFrames(bool save)
{
	if (!readerPtr)
	{
		return;
	}

	//Declare container for multisource frame 
	IMultiSourceFrame *multiSourceFramePtr = NULL;

	//Declare containers for frames for all 3 image streams
	IColorFrame *colorFramePtr = NULL;
	IDepthFrame *depthFramePtr = NULL;
	IInfraredFrame *infraredFramePtr = NULL;

	//Declare reference container for all 3 image streams
	IColorFrameReference *colorFrameRefPtr = NULL;
	IDepthFrameReference *depthFrameRefPtr = NULL;
	IInfraredFrameReference *infraredFrameRefPtr = NULL;

	//Acquire the latest multisource frame. if successful, acquire the sub-frames
	HRESULT hrALF = readerPtr->AcquireLatestFrame(&multiSourceFramePtr);
	if (SUCCEEDED(hrALF))
	{
		// Obtain the frame references for all 3 image streams
		HRESULT hrGCFR = multiSourceFramePtr->get_ColorFrameReference(&colorFrameRefPtr);
		HRESULT hrGDFR = multiSourceFramePtr->get_DepthFrameReference(&depthFrameRefPtr);
		HRESULT hrGIFR = multiSourceFramePtr->get_InfraredFrameReference(&infraredFrameRefPtr);

		// Check flags
		HRESULT hrACF, hrADF, hrAIF;

		// Check for the success of the frame reference and capture the frame if successful
		if (SUCCEEDED(hrGCFR))
		{
			hrACF = colorFrameRefPtr->AcquireFrame(&colorFramePtr);
			safeRelease(colorFrameRefPtr);
		}
		if (SUCCEEDED(hrGDFR))
		{
			hrADF = depthFrameRefPtr->AcquireFrame(&depthFramePtr);
			safeRelease(depthFrameRefPtr);
		}
		if (SUCCEEDED(hrGIFR))
		{
			hrAIF = infraredFrameRefPtr->AcquireFrame(&infraredFramePtr);
			safeRelease(infraredFrameRefPtr);
		}

		//Access frame captured above
		if (SUCCEEDED(hrACF))
		{
			// Get color images
			if (!isColorAllocated.get())
			{
				IFrameDescription *colorFrameDescPtr = NULL;
				HRESULT hrGCFD = colorFramePtr->get_FrameDescription(&colorFrameDescPtr);
				if (SUCCEEDED(hrGCFD))
				{
					colorFrameDescPtr->get_Width(&rgbWidth);
					colorFrameDescPtr->get_Height(&rgbHeight);
					rgbaMutex.lock();
					rgbMutex.lock();
					rgbImage.create(rgbHeight, rgbWidth, CV_8UC3);
					rgbaImage.create(rgbHeight, rgbWidth, CV_8UC4);
					rgbaMutex.unlock();
					rgbMutex.unlock();
					isColorAllocated.set(true);
					safeRelease(colorFrameDescPtr);
				}
			}
			rgbaMutex.lock();
			rgbMutex.lock();
			colorFramePtr->CopyConvertedFrameDataToArray(rgbWidth*rgbHeight*sizeof(RGBQUAD), rgbaImage.data, ColorImageFormat_Bgra);
			int fromTo[] = { 0, 0, 1, 1, 2, 2 };
			mixChannels(&rgbaImage, 1, &rgbImage, 1, fromTo, 3);
			if (save)
			{
			  cv::imwrite("c:\\Temp\\eyeKin-" + personalRobotics::full_date_string() + "color.png", rgbImage);
			}
			rgbMutex.unlock();
			rgbaMutex.unlock();
			safeRelease(colorFramePtr);
		}

		// Get depth frame and generate point cloud
		if (SUCCEEDED(hrADF))
		{
			if (!isDepthAllocated.get())
			{
				IFrameDescription *depthFrameDescription = NULL;
				HRESULT hrGDFD = depthFramePtr->get_FrameDescription(&depthFrameDescription);
				if (SUCCEEDED(hrGDFD))
				{
					depthFrameDescription->get_Width(&depthWidth);
					depthFrameDescription->get_Height(&depthHeight);
					depthMutex.lock();
					depthImage.create(depthHeight, depthWidth, CV_16UC1);
					depthMutex.unlock();
					pointCloudMutex.lock();
					pointCloudPtr = new CameraSpacePoint[depthWidth*depthHeight];
					pointCloudMutex.unlock();
					depth2colorMappingMutex.lock();
					depth2colorMappingPtr = new ColorSpacePoint[depthWidth*depthHeight];
					depth2colorMappingMutex.unlock();
					isDepthAllocated.set(true);
					safeRelease(depthFrameDescription);
				}
			}
			depthMutex.lock();
			depthFramePtr->CopyFrameDataToArray(depthWidth*depthHeight, (UINT16*)depthImage.data);
			if (save)
			{
			  cv::imwrite("c:\\Temp\\eyeKin-" + personalRobotics::full_date_string() + "depth.png", depthImage);
			}
			pointCloudMutex.lock();
			coordinateMapperPtr->MapDepthFrameToCameraSpace(depthWidth*depthHeight, (UINT16*)depthImage.data, depthWidth*depthHeight, pointCloudPtr);
			pointCloudMutex.unlock();
			depth2colorMappingMutex.lock();
			coordinateMapperPtr->MapDepthFrameToColorSpace(depthWidth*depthHeight, (UINT16*)depthImage.data, depthWidth*depthHeight, depth2colorMappingPtr);
			depth2colorMappingMutex.unlock();
			depthMutex.unlock();
			safeRelease(depthFramePtr);
		}

		// Get IR frame
		if (SUCCEEDED(hrAIF))
		{
			if (!isIRallocated.get())
			{
				IFrameDescription *infraredFrameDescription = NULL;
				HRESULT hrGIFD = infraredFramePtr->get_FrameDescription(&infraredFrameDescription);
				if (SUCCEEDED(hrGIFD))
				{
					infraredFrameDescription->get_Width(&depthWidth);
					infraredFrameDescription->get_Height(&depthHeight);
					irMutex.lock();
					irImage.create(depthHeight, depthWidth, CV_16UC1);
					irMutex.unlock();
					isIRallocated.set(true);
					safeRelease(infraredFrameDescription);
				}
			}
			irMutex.lock();
			infraredFramePtr->CopyFrameDataToArray(depthWidth*depthHeight, (UINT16*)irImage.data);
			irImage = irImage / 32;
			irMutex.unlock();
			safeRelease(infraredFramePtr);
		}

		// Report arrival of new frames
		newFrameArrived.set(true);

		//Release the reference to MultiSourceFrameptr
		safeRelease(multiSourceFramePtr);
	}
}
void personalRobotics::KinectReader::kinectThreadRoutine()
{
	int numFrames = 0;
	while (!stopKinectFlag.get())
	{
		if (numFrames > 500)
		{
			numFrames = 0;
			pollFrames(true);
		}
		else
		{
			pollFrames(false);
		}
		numFrames++;
		Sleep(25);
	};
}
void personalRobotics::KinectReader::startKinect()
{
	std::cout << "Starting kinect reader thread" << std::endl;
	stopKinectFlag.set(false);
	readerThread = std::thread(&personalRobotics::KinectReader::kinectThreadRoutine, this);
	std::cout << "Started kinect reader thread" << std::endl;
}
void personalRobotics::KinectReader::stopKinect()
{
	std::cout << "Stopping kinect reader thread" << std::endl;
	stopKinectFlag.set(true);
	if (readerThread.joinable())
		readerThread.join();
	std::cout << "Stopped kinect reader thread" << std::endl;
}

// Accessors
ICoordinateMapper* personalRobotics::KinectReader::getCoordinateMapper()
{
	return coordinateMapperPtr;
}
cv::Mat* personalRobotics::KinectReader::getColorImagePtr()
{
	if (isColorAllocated.get())
		return &rgbImage;
	else
		return &dummy;
}
cv::Mat* personalRobotics::KinectReader::getDepthImagePtr()
{
	if (isDepthAllocated.get())
		return &depthImage;
	else
		return &dummy;
}
cv::Mat* personalRobotics::KinectReader::getIRimagePtr()
{
	if (isIRallocated.get())
		return &irImage;
	else
		return &dummy;
}
CameraSpacePoint* personalRobotics::KinectReader::getPointCloudPtr()
{
	if (isDepthAllocated.get())
		return pointCloudPtr;
	else
		return NULL;
}
size_t personalRobotics::KinectReader::getPointCloudSize()
{
	return depthWidth*depthHeight;
}
