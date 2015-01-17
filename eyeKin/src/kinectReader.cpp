#include "kinectReader.h"

personalRobotics::KinectReader::KinectReader() :	kinectPtr(NULL),
													readerPtr(NULL),
													coordinateMapperPtr(NULL),
													pointCloudPtr(NULL),
													isColorAllocated(false),
													isDepthAllocated(false),
													isIRallocated(false)
{
	stopKinectFlag = false;
	newFrameArrived = false;
	dummy = cv::imread("../data/dummy.jpg", CV_LOAD_IMAGE_COLOR);

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
				exit(-1);
			}
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
	delete pointCloudPtr;
}

void personalRobotics::KinectReader::pollFrames()
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
		//Obtain the frame references for all 3 image streams
		HRESULT hrGCFR = multiSourceFramePtr->get_ColorFrameReference(&colorFrameRefPtr);
		HRESULT hrGDFR = multiSourceFramePtr->get_DepthFrameReference(&depthFrameRefPtr);
		HRESULT hrGIFR = multiSourceFramePtr->get_InfraredFrameReference(&infraredFrameRefPtr);

		HRESULT hrACF, hrADF, hrAIF;

		//Check for the success of the frame reference and capture the frame if successful
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
			newFrameArrFlagMutex.lock();
			newFrameArrived = true;
			newFrameArrFlagMutex.unlock();
			if (!isColorAllocated)
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
					safeRelease(colorFrameDescPtr);
				}
				isColorAllocated = true;
			}
			rgbaMutex.lock();
			colorFramePtr->CopyConvertedFrameDataToArray(rgbWidth*rgbHeight*sizeof(RGBQUAD), rgbaImage.data, ColorImageFormat_Bgra);
			int fromTo[] = { 0, 0, 1, 1, 2, 2 };
			rgbMutex.lock();
			mixChannels(&rgbaImage, 1, &rgbImage, 1, fromTo, 3);
			rgbaMutex.unlock();
			rgbMutex.unlock();
			safeRelease(colorFramePtr);
		}
		if (SUCCEEDED(hrADF))
		{
			if (!isDepthAllocated)
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
					safeRelease(depthFrameDescription);
				}
				isDepthAllocated = true;
			}
			depthMutex.lock();
			depthFramePtr->CopyFrameDataToArray(depthWidth*depthHeight, (UINT16*)depthImage.data);
			depthMutex.unlock();
			safeRelease(depthFramePtr);
		}
		if (SUCCEEDED(hrAIF))
		{
			if (!isIRallocated)
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
					safeRelease(infraredFrameDescription);
				}
				isIRallocated = true;
			}
			irMutex.lock();
			infraredFramePtr->CopyFrameDataToArray(depthWidth*depthHeight, (UINT16*)irImage.data);
			irMutex.unlock();
			safeRelease(infraredFramePtr);
		}

		//Release the reference to MultiSourceFrameptr
		safeRelease(multiSourceFramePtr);

		//Obtain the point cloud
		if (SUCCEEDED(hrADF))
		{
			pointCloudMutex.lock();
			coordinateMapperPtr->MapDepthFrameToCameraSpace(depthWidth*depthHeight, (UINT16*)depthImage.data, depthWidth*depthHeight, pointCloudPtr);
			pointCloudMutex.unlock();
		}
	}
}

ICoordinateMapper* personalRobotics::KinectReader::getCoordinateMapper()
{
	return coordinateMapperPtr;
}

cv::Mat* personalRobotics::KinectReader::getColorImagePtr()
{
	if (isColorAllocated)
		return &rgbImage;
	else
		return &dummy;
}

cv::Mat* personalRobotics::KinectReader::getDepthImagePtr()
{
	if (isDepthAllocated)
		return &depthImage;
	else
		return &dummy;
}

cv::Mat* personalRobotics::KinectReader::getIRimagePtr()
{
	if (isIRallocated)
		return &irImage;
	else
		return &dummy;
}

CameraSpacePoint* personalRobotics::KinectReader::getPointCloudPtr()
{
	if (isDepthAllocated)
		return pointCloudPtr;
	else
		return NULL;
}

size_t personalRobotics::KinectReader::getPointCloudSize()
{
	return depthWidth*depthHeight;
}

void personalRobotics::KinectReader::kinectThreadRoutine()
{
	while (!stopKinectFlag)
	{
		pollFrames();
	}
}

void personalRobotics::KinectReader::startKinect()
{
	readerThread = boost::thread(&personalRobotics::KinectReader::kinectThreadRoutine, this);
}

void personalRobotics::KinectReader::stopKinect()
{
	stopKinectFlag = true;
	readerThread.join();
}