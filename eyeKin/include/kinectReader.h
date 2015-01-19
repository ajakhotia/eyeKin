#ifndef __KINECT_READER__
#define __KINECT_READER__

#include "Kinect.h"
#include "boost/thread.hpp"
#include "opencv.h"

namespace personalRobotics
{
	class KinectReader
	{
	protected:
		IKinectSensor *kinectPtr;
		IMultiSourceFrameReader *readerPtr;
		ICoordinateMapper *coordinateMapperPtr;
		boost::thread readerThread;
		boost::mutex rgbMutex;
		boost::mutex rgbaMutex;
		boost::mutex depthMutex;
		boost::mutex irMutex;
		boost::mutex pointCloudMutex;
		boost::mutex newFrameArrFlagMutex;
		bool newFrameArrived;
		bool stopKinectFlag;
		int rgbWidth;
		int rgbHeight;
		int depthWidth;
		int depthHeight;
		cv::Mat rgbImage;
		cv::Mat rgbaImage;
		cv::Mat depthImage;
		cv::Mat irImage;
		CameraSpacePoint *pointCloudPtr;
		cv::Mat dummy;
	public:
		KinectReader();
		~KinectReader();
		void pollFrames();
		ICoordinateMapper* getCoordinateMapper();
		cv::Mat* getColorImagePtr();
		cv::Mat* getIRimagePtr();
		cv::Mat* getDepthImagePtr();
		CameraSpacePoint* getPointCloudPtr();
		size_t getPointCloudSize();
		void kinectThreadRoutine();
		void startKinect();
		void stopKinect();
		bool isColorAllocated;
		bool isDepthAllocated;
		bool isIRallocated;
	};
	template<class Interface> void safeRelease(Interface *&inInterface)
	{
		inInterface->Release();
		inInterface = NULL;
	}
}
#endif