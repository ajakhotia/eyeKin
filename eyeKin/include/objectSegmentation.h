#ifndef __OBJECT_SEGMENTATION_H__
#define __OBJECT_SEGMENTATION_H__

#include "settings.h"
#include "kinectReader.h"
#include "entity.h"
#include "pcl.h"
#include "timer.h"

namespace personalRobotics
{
	struct IDLookUp
	{
		int id;
		cv::Point2f centroid;
		float angle;
		cv::Size2f boundingSize;
	};

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
		float objectDifferenceThreshold;

		// Containers
		pcl::PointCloud<pcl::PointXYZ>::Ptr pclPtr;
		pcl::ModelCoefficients::Ptr planePtr;
		std::vector<personalRobotics::Entity> entityList;
		cv::Mat homography;
		cv::Point2f pixelSize;
		std::vector<IDLookUp> previousIDList;
		std::vector<IDLookUp> currentIDList;

		// Container locks
		std::mutex pclPtrLock;
		std::mutex entityListLock;

		// Control flags
		MutexBool stopSegmentorFlag;
		MutexBool homographySetFlag;

		// Runner threads
		std::thread segementorThread;

		//Counters
		int objectCount;

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
		std::vector<IDLookUp>* getIDList();

		// Setters
		void setHomography(cv::Mat &inhomography);

		// Routines
		bool findTablePlane();
		void startSegmentor();
		void segmentorThreadRoutine();
		void stopSegmentor();
		void calculateOverallChangeInFrames(std::vector<IDLookUp> pIDList, std::vector<IDLookUp> cIDList);
		float calculateEntityDifferences(cv::Point2f IDcentroid, cv::Point2f objectCentroid, float IDangle, float objectAngle, cv::Size2f IDBoundingSize, cv::Size2f objectBoundingSize);
	};

	void createCheckerboard(cv::Mat& checkerboard, int width, int height, int& numBlocksX, int& numBlocksY);
}
#endif