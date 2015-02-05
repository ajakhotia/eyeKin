#ifndef __OBJECT_SEGMENTATION_H__
#define __OBJECT_SEGMENTATION_H__

#include "settings.h"
#include "kinectReader.h"
#include "entity.h"
#include "pcl.h"
#include "timer.h"

namespace personalRobotics
{
	class ObjectSegmentor : public KinectReader
	{
	protected:
		// Config params
		int maxRansacIters;
		float ransacMargin;
		float distCutoff;
		float radialThreshold;
		float clusterTolerance;
		int minClusterSize;
		int maxClusterSize;
		float minThreshold;
		float maxThreshold;

		// Containers
		pcl::PointCloud<pcl::PointXYZ>::Ptr pclPtr;
		pcl::ModelCoefficients::Ptr planePtr;
		std::vector<personalRobotics::Entity> entityList;
		cv::Mat homography;
		cv::Point2f pixelSize;

		// Container locks
		std::mutex pclPtrLock;
		std::mutex entityListLock;

		// Control flags
		MutexBool stopSegmentorFlag;
		MutexBool homographySetFlag;

		// Runner threads
		std::thread segementorThread;

		// Routines
		void planeSegment();
	public:
		// Constructor and Destructor
		ObjectSegmentor();
		~ObjectSegmentor();

		// Thread safety measures
		void lockList();
		void unlockList();
		void lockPcl();
		void unlockPcl();
		MutexBool newListGenerated;

		// Accessors
		std::vector<personalRobotics::Entity>* getEntityList();
		cv::Point2f* getRGBpixelSize();

		// Setters
		void setHomography(cv::Mat &inhomography);

		// Routines
		bool findTablePlane();
		void startSegmentor();
		void segmentorThreadRoutine();
		void stopSegmentor();
	};

	void createCheckerboard(cv::Mat& checkerboard, int width, int height, int& numBlocksX, int& numBlocksY);
}
#endif