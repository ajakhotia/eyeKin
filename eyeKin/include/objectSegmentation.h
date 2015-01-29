#ifndef __OBJECT_SEGMENTATION_H__
#define __OBJECT_SEGMENTATION_H__

#include "kinectReader.h"
#include "entity.h"
#include "utilities.h"
#include "settings.h"
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

		// Container locks
		boost::mutex pclPtrLock;
		boost::mutex entityListLock;
		boost::mutex newListGeneratedFlagLock;

		// Control flags
		bool stopSegmentorFlag;
		bool homographySetFlag;
		bool newListGeneratedFlag;

		// Runner threads
		boost::thread segementorThread;

		// Methods
		void planeSegment();
	public:
		ObjectSegmentor();
		~ObjectSegmentor();
		bool findTablePlane();

		// Accessors
		void lockList();
		void unlockList();
		void lockPcl();
		void unlockPcl();
		void lockNewListGenFlag();
		void unlockNewListGenFlag();
		std::vector<personalRobotics::Entity>* getEntityList();

		// Setters
		void setHomography(cv::Mat &inhomography);

		// Routines
		void startSegmentor();
		void segmentorThreadRoutine();
		void stopSegmentor();
	};
	void createCheckerboard(cv::Mat& checkerboard, int width, int height, int& numBlocksX, int& numBlocksY);
}
#endif