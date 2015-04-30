#ifndef __KINECT_READER__
#define __KINECT_READER__

#include "Kinect.h"
#include <thread>
#include <mutex>
#include "utilities.h"
#include "opencv.h"

namespace personalRobotics
{
	class KinectReader
	{
	protected:
		// Interfaces
		IKinectSensor *kinectPtr;
		IMultiSourceFrameReader *readerPtr;
		ICoordinateMapper *coordinateMapperPtr;

		// Reader thread
		std::thread readerThread;

		// Parameters
		int rgbWidth;
		int rgbHeight;
		int depthWidth;
		int depthHeight;

		// Containers
		cv::Mat rgbImage;
		cv::Mat rgbaImage;
		cv::Mat depthImage;
		cv::Mat irImage;
		cv::Mat dummy;
		CameraSpacePoint *pointCloudPtr;
		ColorSpacePoint *depth2colorMappingPtr;
	public:
		// Constructor & Destructor
		KinectReader();
		~KinectReader();

		// Routines
		void mapInfraredToColor(std::vector<cv::Point> &infraredPoints, std::vector<cv::Point> &colorPoints, ColorSpacePoint *mapping);
		void pollFrames(bool save);
		void kinectThreadRoutine();
		void startKinect();
		void stopKinect();

		// Thread safe boolean flags
		MutexBool newFrameArrived;
		MutexBool stopKinectFlag;
		MutexBool isColorAllocated;
		MutexBool isDepthAllocated;
		MutexBool isIRallocated;

		// Thread safety locks
		std::mutex rgbMutex;
		std::mutex rgbaMutex;
		std::mutex depthMutex;
		std::mutex irMutex;
		std::mutex pointCloudMutex;
		std::mutex depth2colorMappingMutex;

		// Accessors
		ICoordinateMapper* getCoordinateMapper();
		cv::Mat* getColorImagePtr();
		cv::Mat* getIRimagePtr();
		cv::Mat* getDepthImagePtr();
		CameraSpacePoint* getPointCloudPtr();
		size_t getPointCloudSize();
	};

	template<class Interface> void safeRelease(Interface *&inInterface)
	{
		inInterface->Release();
		inInterface = NULL;
	}
}
#endif