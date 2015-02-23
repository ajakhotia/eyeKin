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
	radialThreshold = (DEFAULT_RADIAL_CUTOFF)*(DEFAULT_RADIAL_CUTOFF);
	clusterTolerance = DEFAULT_CLUSTER_TOLERANCE;
	minClusterSize = DEFAULT_MINIMUM_CLUSTER_SIZE;
	maxClusterSize = DEFAULT_MAXIMUM_CLUSTER_SIZE;
	objectDifferenceThreshold = OBJECT_DIFFERENCE_THRESHOLD;

	// Set initial state of flags
	stopSegmentorFlag.set(true);

	// Initialize pcl containers
	lockPcl();
	pclPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	unlockPcl();
	planePtr = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);

	objectCount = 0;

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
cv::Point2f* personalRobotics::ObjectSegmentor::getRGBpixelSize()
{
	return &rgbPixelSize;
}
std::vector<personalRobotics::IDLookUp>* personalRobotics::ObjectSegmentor::getIDList()
{
	return &previousIDList;
}
bool personalRobotics::ObjectSegmentor::getStatic()
{
	return frameStatic;
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
		size_t dstPoint = 0;

		// Copy the RGB image
		rgbMutex.lock();
		pointCloudMutex.lock();
		cv::Mat rgbImageCopy = rgbImage.clone();
		rgbMutex.unlock();

		// Convert points to pointcloud and plane segment as well
		for (size_t point = 0; point < numPoints; point++)
		{
			if (pointCloudPtr[point].Z > minThreshold && pointCloudPtr[point].Z < maxThreshold)
			{
				if ((planePtr->values[0] * pointCloudPtr[point].X + planePtr->values[1] * pointCloudPtr[point].Y + planePtr->values[2] * pointCloudPtr[point].Z + planePtr->values[3]) > distCutoff)
				{
					if ((pointCloudPtr[point].X * pointCloudPtr[point].X / pointCloudPtr[point].Z / pointCloudPtr[point].Z + pointCloudPtr[point].Y * pointCloudPtr[point].Y / pointCloudPtr[point].Z / pointCloudPtr[point].Z) < radialThreshold)
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

		if (dstPoint > 20000)
		{
			std::cout << "rejecting frame, too many points: " << dstPoint << std::endl;
			unlockPcl();
			return;
		}

		// Suppress noise
		pcl::PointCloud<pcl::PointXYZ>::Ptr sorOutput(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud(pclPtr);
		sor.setMeanK(10);
		sor.setStddevMulThresh(0.5);
		sor.filter(*sorOutput);
		unlockPcl();

		// Down sample using voxel grid
		pcl::PointCloud<pcl::PointXYZ>::Ptr projDsCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
		voxelGrid.setInputCloud(sorOutput);
		voxelGrid.setLeafSize(0.012f, 0.012f, 0.012f);
		voxelGrid.filter(*projDsCloud);

		// Project the points onto table plane
		/*pcl::PointCloud<pcl::PointXYZ>::Ptr projDsCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::ProjectInliers<pcl::PointXYZ> orthProjection;
		orthProjection.setModelType(pcl::SACMODEL_PLANE);
		orthProjection.setInputCloud(dsCloud);
		orthProjection.setModelCoefficients(planePtr);
		orthProjection.filter(*projDsCloud);*/

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
		for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it)
		{
			// Convert PCL to kinect camera point representation
			CameraSpacePoint *cameraSpacePoints = new CameraSpacePoint[it->indices.size()];
			int pointNum = 0;
			bool validCluster = true;
			for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
			{
				if (!onBoundingEdges(projDsCloud->points[*pit]))
					cameraSpacePoints[pointNum++] = { projDsCloud->points[*pit].x, projDsCloud->points[*pit].y, projDsCloud->points[*pit].z };
				else
				{
					std::cout << "marking cluster for deletion" << std::endl;
					validCluster = false;
					break;
				}
			}

			if (!validCluster)
			{
				std::cout << "deleting cluster" << std::endl;
				delete[] cameraSpacePoints;
				continue;
			}

			std::cout << pointNum << std::endl;
			// Map the points to RGB space and infrared space
			cv::Mat colorSpacePoints(pointNum, 2, CV_32F);
			coordinateMapperPtr->MapCameraPointsToColorSpace(pointNum, cameraSpacePoints, pointNum, (ColorSpacePoint*)colorSpacePoints.data);

			// Release the memory
			delete[] cameraSpacePoints;

			//Find mean and covariance
			cv::Mat cvCentroid, cvCovar;
			cv::calcCovarMatrix(colorSpacePoints, cvCovar, cvCentroid, CV_COVAR_NORMAL | CV_COVAR_ROWS, CV_32F);

			//Find xAxis and yAxis
			cv::Mat eigenValues, eigenVectors;
			cv::eigen(cvCovar*(1.f / (pointNum - 1)), true, eigenValues, eigenVectors);
			if (eigenValues.at<float>(0, 0) > 1 && eigenValues.at<float>(1, 0) > 1)
			{
				bool match = false;
				float minScore = objectDifferenceThreshold;
				cv::Point2f objectCentroid = cv::Point2f(cvCentroid.at<float>(0, 0), cvCentroid.at<float>(0, 1));
				float objectAngle = atan2(eigenVectors.at<float>(0, 1), eigenVectors.at<float>(0, 0));
				cv::Size2f objectBoundingSize = cv::Size2f(sqrtf(eigenValues.at<float>(0, 0))*6.5f, sqrtf(eigenValues.at<float>(1, 0))*6.5f);
				IDLookUp iLU;

				for (std::vector<IDLookUp>::iterator idPtr = previousIDList.begin(); idPtr != previousIDList.end(); idPtr++)
				{
					float currentScore = calculateEntityDifferences(idPtr->centroid, objectCentroid, idPtr->angle, objectAngle, idPtr->boundingSize, objectBoundingSize);
					if (currentScore < objectDifferenceThreshold && currentScore < minScore)
					{
						iLU.id = idPtr->id;
						iLU.numFramesSame = idPtr->numFramesSame + 1;
						minScore = currentScore;
						std::cout << minScore << std::endl;
						match = true;
					}
				}
				iLU.centroid = objectCentroid;
				iLU.angle = objectAngle;
				iLU.boundingSize = objectBoundingSize;
				if (!match){
					objectCount++;
					iLU.id = objectCount;
					iLU.numFramesSame = 1;
				}
				currentIDList.push_back(iLU);
				entityList.push_back(personalRobotics::Entity(objectCentroid, objectAngle, objectBoundingSize, iLU.id));
			}
		}

		// Generate patch and geometric data for each of the entity
		for (std::vector<personalRobotics::Entity>::iterator entityPtr = entityList.begin(); entityPtr != entityList.end(); entityPtr++)
		{
			entityPtr->generateData(homography, rgbImageCopy);
		}

		frameStatic = calculateOverallChangeInFrames(currentIDList);
		std::cout << "Static?  = " << frameStatic << std::endl;
		previousIDList = currentIDList;

		// Check for static condition here and set new list to true for static frames only
		newListGenerated.set(true);
		unlockList();
		currentIDList.clear();
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
	CameraSpacePoint keyPoints[3];
	ColorSpacePoint projectedKeyPoints[3];
	keyPoints[0] = { 0, 0, (-1 * (planePtr->values[3] + planePtr->values[0] * 0 + planePtr->values[1] * 0) / planePtr->values[2]) };	//(0  , 0   ,z1)
	keyPoints[1] = { 0.1, 0, (-1 * (planePtr->values[3] + planePtr->values[0] * 0.1 + planePtr->values[1] * 0) / planePtr->values[2]) };	//(0.1, 0   ,z2)
	keyPoints[2] = { 0, 0.1, (-1 * (planePtr->values[3] + planePtr->values[0] * 0 + planePtr->values[1] * 0.1) / planePtr->values[2]) };	//(0  , 0.1 ,z3)
	coordinateMapperPtr->MapCameraPointsToColorSpace(3, keyPoints, 3, projectedKeyPoints);
	double delX = sqrt((projectedKeyPoints[1].X - projectedKeyPoints[0].X)*(projectedKeyPoints[1].X - projectedKeyPoints[0].X) + (projectedKeyPoints[1].Y - projectedKeyPoints[0].Y)*(projectedKeyPoints[1].Y - projectedKeyPoints[0].Y));
	double delY = sqrt((projectedKeyPoints[2].X - projectedKeyPoints[0].X)*(projectedKeyPoints[2].X - projectedKeyPoints[0].X) + (projectedKeyPoints[2].Y - projectedKeyPoints[0].Y)*(projectedKeyPoints[2].Y - projectedKeyPoints[0].Y));
	rgbPixelSize.x = 100 / delX;
	rgbPixelSize.y = 100 / delY;

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
	cv::namedWindow("disp");
	cv::namedWindow("disp2");
	cv::startWindowThread();
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

// 
float personalRobotics::ObjectSegmentor::calculateEntityDifferences(cv::Point2f IDcentroid, cv::Point2f objectCentroid, float IDangle, float objectAngle, cv::Size2f IDBoundingSize, cv::Size2f objectBoundingSize)
{
	if (abs(IDBoundingSize.width*IDBoundingSize.height - objectBoundingSize.width*objectBoundingSize.height) >= 50)
		return 10000;
	else
	{
		return sqrt(pow((IDcentroid.x - objectCentroid.x), 2) + pow((IDcentroid.y - objectCentroid.y), 2) + pow((IDangle - objectAngle), 2) + 10 * (pow((IDBoundingSize.width - objectBoundingSize.width), 2) + pow((IDBoundingSize.height - objectBoundingSize.height), 2)));
	}
}
bool personalRobotics::ObjectSegmentor::calculateOverallChangeInFrames(std::vector<personalRobotics::IDLookUp> cIDList)
{
	for (std::vector<IDLookUp>::iterator cIDPtr = cIDList.begin(); cIDPtr != cIDList.end(); cIDPtr++)
	{
		if (cIDPtr->numFramesSame < 3)
		{ 
			return false;
		}
	}
	return true;
}
bool personalRobotics::ObjectSegmentor::onBoundingEdges(pcl::PointXYZ point)
{
	if (abs(point.z - minThreshold) < 0.04)	// Its a hack and will not work for general situation. Better way is to actually create a min plane
	{
		return true;
	}
	else
	{
		// check if its the edge of the depth map
	}
	return false;
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