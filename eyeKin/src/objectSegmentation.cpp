#include "objectSegmentation.h"

// Constructor and Destructor
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
	stopSegmentorFlag.set(true);

	// Initialize pcl containers
	lockPcl();
	pclPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	unlockPcl();
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

// Thread safety measures
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

// Accessors
std::vector<personalRobotics::Entity>* personalRobotics::ObjectSegmentor::getEntityList()
{
	return &entityList;
}

// Setters
void personalRobotics::ObjectSegmentor::setHomography(cv::Mat &inHomography)
{
	homography = inHomography.clone();
	homographySetFlag.set(true);
}

// Routines
void personalRobotics::ObjectSegmentor::planeSegment()
{
	if (newFrameArrived.get())
	{
		// Unset the flag
		newFrameArrived.set(false);

		// Empty the containers
		size_t numPoints = depthHeight*depthWidth;
		lockPcl();
		pclPtr->clear();
		pclPtr->resize(numPoints);

		// Obtain the plane semented pointcloud
		size_t dstPoint = 0;
		pointCloudMutex.lock();
		cv::Mat IRmask(rgbHeight, rgbWidth, CV_8UC1,cv::Scalar(255));
		irMutex.lock();
		cv::Mat irImageCopy = irImage.clone();
		irMutex.unlock();
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
		sor.setMeanK(25);
		sor.setStddevMulThresh(0.5);
		sor.filter(*pclPtr);
		
		// Down sample using voxel grid
		pcl::PointCloud<pcl::PointXYZ>::Ptr dsCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
		voxelGrid.setInputCloud(pclPtr);
		voxelGrid.setLeafSize(0.012f, 0.012f, 0.012f);
		voxelGrid.filter(*dsCloud);
		unlockPcl();

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
		newListGenerated.set(true);
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

			//Map the points to depth and IR Space
			cv::Mat dpoints(pointNum, 2, CV_32F);				//Added by Kevin
			coordinateMapperPtr->MapCameraPointsToDepthSpace(pointNum, cameraSpacePoints, pointNum, (DepthSpacePoint*)dpoints.data);		//Added by Kevin
			delete[] cameraSpacePoints;
			cv::Rect boundRect = cv::boundingRect(dpoints);

			//finds the contours using the ir image
			boundRect += cv::Size(4, 4);
			int depthPointNum = boundRect.area();
			cv::Mat croppedImage = irImageCopy(boundRect);
			std::vector<std::vector<cv::Point>> ncontours;
			std::vector<std::vector<cv::Point>> npoints;
			cv::Mat cannyOut;
			cv::Canny(croppedImage, cannyOut, DEFAULT_IRCANNY_LOW_THRESHOLD, DEFAULT_IRCANNY_HIGH_THRESHOLD, DEFAULT_IRCANNY_KERNEL_SIZE, false);
			cv::findContours(cannyOut, npoints, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
			int numpoints = npoints.size();
			coordinateMapperPtr->MapDepthPointsToColorSpace(numpoints, (DepthSpacePoint*)npoints.data, 0, 0, numpoints, (ColorSpacePoint*)ncontours.data);
			cv::drawContours(IRmask, ncontours, 0, cv::Scalar(0), CV_FILLED);

			//Find mean and covariance
			cv::Mat cvCentroid, cvCovar;
			cv::calcCovarMatrix(points, cvCovar, cvCentroid, CV_COVAR_NORMAL | CV_COVAR_ROWS, CV_32F);

			//Find xAxis and yAxis
			cv::Mat eigenValues, eigenVectors;
			cv::eigen(cvCovar / (pointNum - 1), true, eigenValues, eigenVectors);
			if (eigenValues.at<float>(0, 0) > 1 && eigenValues.at<float>(0, 1) > 1)
				entityList.push_back(personalRobotics::Entity(cv::Point2f(cvCentroid.at<float>(0, 0), cvCentroid.at<float>(0, 1)), atan2(eigenVectors.at<float>(0, 1), eigenVectors.at<float>(0, 0)), sqrtf(eigenValues.at<float>(0, 0))*6.5f, sqrtf(eigenValues.at<float>(0, 1))*6.5f));
		}

		// Generate patch and geometric data for each of the entity
		for (std::vector<personalRobotics::Entity>::iterator entityPtr = entityList.begin(); entityPtr != entityList.end(); entityPtr++)
		{
			entityPtr->generateData(homography, rgbImageCopy, IRmask);
		}
		unlockList();
	}
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

	// Find pixel size


	// Return
	return true;
}
void personalRobotics::ObjectSegmentor::startSegmentor()
{
	stopSegmentorFlag.set(false);
	segementorThread = std::thread(&personalRobotics::ObjectSegmentor::segmentorThreadRoutine, this);
}
void personalRobotics::ObjectSegmentor::segmentorThreadRoutine()
{
	while (!stopSegmentorFlag.get())
	{
		planeSegment();
	}
}
void personalRobotics::ObjectSegmentor::stopSegmentor()
{
	stopSegmentorFlag.set(true);
	if (segementorThread.joinable())
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