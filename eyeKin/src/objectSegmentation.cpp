#include "objectSegmentation.h"

personalRobotics::ObjectSegmentor::ObjectSegmentor()
{
	// Set all config params
	minThreshold = DEFAULT_MIN_DEPTH_LIMIT;
	maxThreshold = DEFAULT_MAX_DEPTH_LIMIT;
	maxRansacIters = DEFAULT_MAX_RANSAC_ITERATIONS;
	ransacMargin = DEFAULT_DEPTH_MARGIN;
	distCutoff = DEFAULT_DISTANCE_CUTOFF;
	radialThreshold = DEFAULT_RADIAL_CUTOFF;
	clusterTolerance = DEFAULT_CLUSTER_TOLERANCE;
	minClusterSize = DEFAULT_MINIMUM_CLUSTER_SIZE;
	maxClusterSize = DEFAULT_MAXIMUM_CLUSTER_SIZE;

	// Set initial state of flags
	stopSegmentorFlag = false;

	// Initialize pcl containers
	pclPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	planePtr = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);

	// Start the kinect thread
	startKinect();
}

personalRobotics::ObjectSegmentor::~ObjectSegmentor()
{
	// Stop the kinect
	stopKinect();

	// Stop segmentation thread
	stopSegmentor();
}

bool personalRobotics::ObjectSegmentor::findTablePlane()
{
	if (!pointCloudPtr)
		return false;
	size_t numPoints = depthHeight*depthWidth;
	pclPtr->clear();
	pclPtr->resize(numPoints);
	size_t dstPoint = 0;
	pointCloudMutex.lock();
	for (size_t point = 0; point < numPoints; point++)
	{
		if (pointCloudPtr[point].Z > minThreshold && pointCloudPtr[point].Z < maxThreshold)
		{
			pclPtr.get()->points[dstPoint].x = pointCloudPtr[point].X;
			pclPtr.get()->points[dstPoint].y = pointCloudPtr[point].Y;
			pclPtr.get()->points[dstPoint].z = pointCloudPtr[point].Z;
			dstPoint++;
		}
	}
	pointCloudMutex.unlock();
	pclPtr->resize(dstPoint);

	// Segment out the plane using least squares and RANSAC
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(maxRansacIters);
	seg.setDistanceThreshold(ransacMargin);
	seg.setInputCloud(pclPtr);
	seg.segment(*inliers, *planePtr);

	// Change the sign of all number so that 'd' in ax+by+cz+d is always positive
	float signOfD = (planePtr->values[3]) / abs(planePtr->values[3]);
	planePtr->values[0] = planePtr->values[0] * signOfD;
	planePtr->values[1] = planePtr->values[1] * signOfD;
	planePtr->values[2] = planePtr->values[2] * signOfD;
	planePtr->values[3] = planePtr->values[3] * signOfD;

	// Return
	return true;
}

void personalRobotics::ObjectSegmentor::planeSegment()
{
	//Debug
	#ifdef DEBUG_PROFILER
		Timer timer("planeSegment()");
	#endif
	newFrameArrFlagMutex.lock();
	if (newFrameArrived)
	{
		// Unset the flag
		newFrameArrived = false;
		newFrameArrFlagMutex.unlock();

		// Empty the containers
		size_t numPoints = depthHeight*depthWidth;
		lockPcl();
		pclPtr->clear();
		pclPtr->resize(numPoints);


		// Obtain the plane semented pointcloud
		size_t dstPoint = 0;
		pointCloudMutex.lock();
		rgbMutex.lock();
		cv::Mat rgbImageCopy = rgbImage.clone();
		rgbMutex.unlock();
		for (size_t point = 0; point < numPoints; point++)
		{
			if (pointCloudPtr[point].Z > minThreshold && pointCloudPtr[point].Z < maxThreshold)
			{

				float dist = (planePtr->values[0] * pointCloudPtr[point].X + planePtr->values[1] * pointCloudPtr[point].Y + planePtr->values[2] * pointCloudPtr[point].Z + planePtr->values[3]);
				if (dist > distCutoff)
				{
					float radius = sqrt(pointCloudPtr[point].X*pointCloudPtr[point].X / pointCloudPtr[point].Z / pointCloudPtr[point].Z + pointCloudPtr[point].Y*pointCloudPtr[point].Y / pointCloudPtr[point].Z / pointCloudPtr[point].Z);
					if (radius < radialThreshold)
					{
						pclPtr->points[dstPoint].x = pointCloudPtr[point].X;
						pclPtr->points[dstPoint].y = pointCloudPtr[point].Y;
						pclPtr->points[dstPoint].z = pointCloudPtr[point].Z;
						dstPoint++;
					}
				}
			}
		}
		pointCloudMutex.unlock();
		pclPtr->resize(dstPoint);

		// Suppress noise
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud(pclPtr);
		sor.setMeanK(20);
		sor.setStddevMulThresh(0.5);
		sor.filter(*pclPtr);
		unlockPcl();

		// Down sample using voxel grid
		pcl::PointCloud<pcl::PointXYZ>::Ptr dsCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
		voxelGrid.setInputCloud(pclPtr);
		voxelGrid.setLeafSize(0.012f, 0.012f, 0.012f);
		voxelGrid.filter(*dsCloud);

		// Project the points onto table plane
		pcl::PointCloud<pcl::PointXYZ>::Ptr projDsCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::ProjectInliers<pcl::PointXYZ> orthProjection;
		orthProjection.setModelType(pcl::SACMODEL_PLANE);
		orthProjection.setInputCloud(dsCloud);
		orthProjection.setModelCoefficients(planePtr);
		orthProjection.filter(*projDsCloud);

		// Clustering
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(projDsCloud);
		std::vector<pcl::PointIndices> clusterIndices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance(clusterTolerance);
		ec.setMinClusterSize(minClusterSize);
		ec.setMaxClusterSize(maxClusterSize);
		ec.setSearchMethod(tree);
		ec.setInputCloud(projDsCloud);
		ec.extract(clusterIndices);

		// Extract cloud for each object
		lockList();
		entityList.clear();
		lockNewListGenFlag();
		newListGeneratedFlag = true;
		unlockNewListGenFlag();
		for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it)
		{
			CameraSpacePoint *cameraSpacePoints = new CameraSpacePoint[it->indices.size()];
			int pointNum = 0;
			for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
			{
				cameraSpacePoints[pointNum] = { projDsCloud->points[*pit].x, projDsCloud->points[*pit].y, projDsCloud->points[*pit].z };		// Orthographic projection on table followed by projective transform onto RGB
				pointNum++;
			}

			// Map the points to RGB space
			cv::Mat points(pointNum, 2, CV_32F);
			coordinateMapperPtr->MapCameraPointsToColorSpace(pointNum, cameraSpacePoints, pointNum, (ColorSpacePoint*)points.data);
			delete[] cameraSpacePoints;

			//Find mean and covariance
			cv::Mat cvCentroid, cvCovar;
			cv::calcCovarMatrix(points, cvCovar, cvCentroid, CV_COVAR_NORMAL | CV_COVAR_ROWS, CV_32F);

			//Find xAxis and yAxis
			cv::Mat eigenValues, eigenVectors;
			cv::eigen(cvCovar / (pointNum - 1), true, eigenValues, eigenVectors);
			if (eigenValues.at<float>(0, 0) > 1 && eigenValues.at<float>(0, 1) > 1)
				entityList.push_back(personalRobotics::Entity(cv::Point2f(cvCentroid.at<float>(0, 0), cvCentroid.at<float>(0, 1)), atan2(eigenVectors.at<float>(0, 1), eigenVectors.at<float>(0, 0)), sqrtf(eigenValues.at<float>(0, 0))*6.5f, sqrtf(eigenValues.at<float>(0, 1))*6.5f));
		}
		for (std::vector<personalRobotics::Entity>::iterator entityPtr = entityList.begin(); entityPtr != entityList.end(); entityPtr++)
		{
			entityPtr->generateData(homography, rgbImageCopy);
		}
		unlockList();
	}
	else
	{
		newFrameArrFlagMutex.unlock();
	}
	//Debug
	#ifdef DEBUG_PROFILER
		timer.toc();
	#endif
}

void personalRobotics::ObjectSegmentor::lockList()
{
	entityListLock.lock();
}

void personalRobotics::ObjectSegmentor::unlockList()
{
	entityListLock.unlock();
}

void personalRobotics::ObjectSegmentor::lockPcl()
{
	pclPtrLock.lock();
}

void personalRobotics::ObjectSegmentor::unlockPcl()
{
	pclPtrLock.unlock();
}

void personalRobotics::ObjectSegmentor::lockNewListGenFlag()
{
	newListGeneratedFlagLock.lock();
}
void personalRobotics::ObjectSegmentor::unlockNewListGenFlag()
{
	newListGeneratedFlagLock.unlock();
}

std::vector<personalRobotics::Entity>* personalRobotics::ObjectSegmentor::getEntityList()
{
	return &entityList;
}

void personalRobotics::ObjectSegmentor::setHomography(cv::Mat &inHomography)
{
	homography = inHomography.clone();
	homographySetFlag = true;
}

void personalRobotics::ObjectSegmentor::startSegmentor()
{
	segementorThread = boost::thread(&personalRobotics::ObjectSegmentor::segmentorThreadRoutine,this);
}

void personalRobotics::ObjectSegmentor::segmentorThreadRoutine()
{
	while (!stopSegmentorFlag)
	{
		planeSegment();
	}
}

void personalRobotics::ObjectSegmentor::stopSegmentor()
{
	stopSegmentorFlag = true;
	segementorThread.join();
}

void personalRobotics::createCheckerboard(cv::Mat& checkerboard, int width, int height, int& numBlocksX, int& numBlocksY)
{
	checkerboard.create(height, width, CV_8UC1);
	int blockSize = (personalRobotics::gcd(height, width));
	numBlocksX = width / blockSize;
	numBlocksY = height / blockSize;
	int color = 0;
	for (int row = 0; row<(numBlocksY); row++)
	{
		if (color == 0)
			color = 255;
		else
			color = 0;
		for (int col = 0; col<(numBlocksX); col++)
		{
			if (color == 0)
				color = 255;
			else
				color = 0;
			cv::Scalar cvcolor(color);
			cv::Point p1(col*blockSize, row*blockSize);
			cv::Point p2((col + 1)*blockSize, (row + 1)*blockSize);
			if (row == 0 || col == 0 || row == numBlocksY - 1 || col == numBlocksX - 1)
				rectangle(checkerboard, p1, p2, cv::Scalar(255), CV_FILLED);
			else
				rectangle(checkerboard, p1, p2, cvcolor, CV_FILLED);
		}
	}
	numBlocksX -= 3;
	numBlocksY -= 3;
}