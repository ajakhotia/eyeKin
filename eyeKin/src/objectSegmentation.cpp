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
	objectMovementThreshold = OBJECT_MOVEMENT_THRESHOLD;

	// Set initial state of flags
	stopSegmentorFlag.set(true);

	// Initialize pcl containers
	lockPcl();
	pclPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	unlockPcl();
	planePtr = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);

	objectCount = 0;
	frameStatic = false;
	prevFrameStatic = false;

	// Start the kinect thread
	startKinect();
}
personalRobotics::ObjectSegmentor::~ObjectSegmentor()
{
	// Stop segmentation thread
	stopSegmentor();
	
	// Stop the kinect
	stopKinect();	
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
void personalRobotics::ObjectSegmentor::setHomography(cv::Mat inHomography, int width, int height)
{
	homography = inHomography.clone();
	homographySetFlag.set(true);

	CameraSpacePoint keyPoints[3];
	ColorSpacePoint projectedKeyPoints[3];
	keyPoints[0] = { 0, 0, (-1 * (planePtr->values[3] + planePtr->values[0] * 0 + planePtr->values[1] * 0) / planePtr->values[2]) };	//(0  , 0   ,z1)
	keyPoints[1] = { 0.1, 0, (-1 * (planePtr->values[3] + planePtr->values[0] * 0.1 + planePtr->values[1] * 0) / planePtr->values[2]) };	//(0.1, 0   ,z2)
	keyPoints[2] = { 0, 0.1, (-1 * (planePtr->values[3] + planePtr->values[0] * 0 + planePtr->values[1] * 0.1) / planePtr->values[2]) };	//(0  , 0.1 ,z3)
	std::cout << "The keyPoints are: " << std::endl;
	for (int i = 0; i < 3; i++)
	{
		std::cout << "(" << keyPoints[i].X << ", " << keyPoints[i].Y << ", " << keyPoints[i].Z << ")" << std::endl;
	}
	coordinateMapperPtr->MapCameraPointsToColorSpace(3, keyPoints, 3, projectedKeyPoints);

	std::cout << "The projected keyPoints are: " << std::endl;
	for (int i = 0; i < 3; i++)
	{
		std::cout << "(" << projectedKeyPoints[i].X << ", " << projectedKeyPoints[i].Y << ")" << std::endl;
	}

	cv::Mat rgbToKeyPoints(2, 2, CV_32FC1);
	rgbToKeyPoints.at<float>(0, 0) = projectedKeyPoints[1].X - projectedKeyPoints[0].X;
	rgbToKeyPoints.at<float>(1, 0) = projectedKeyPoints[1].Y - projectedKeyPoints[0].Y;
	rgbToKeyPoints.at<float>(0, 1) = projectedKeyPoints[2].X - projectedKeyPoints[0].X;
	rgbToKeyPoints.at<float>(1, 1) = projectedKeyPoints[2].Y - projectedKeyPoints[0].Y;

	std::cout << "RGB frame to keypoints conversion matrix is:" << std::endl;
	std::cout << rgbToKeyPoints << std::endl;

	std::vector<cv::Point2f> projCornersInProj, projCornersInRGB;

	std::cout << "setting homography with width: " << width << " height: " << height << " H = " << inHomography << std::endl;
	projCornersInProj.push_back(cv::Point2f(0, 0));
	projCornersInProj.push_back(cv::Point2f(width, 0));
	projCornersInProj.push_back(cv::Point2f(width, height));
	projCornersInProj.push_back(cv::Point2f(0, height));

	cv::perspectiveTransform(projCornersInProj, projCornersInRGB, homography.inv());

	std::cout << "Projector corners in projector frame are at:" << std::endl;
	for (std::vector<cv::Point2f>::iterator iter = projCornersInProj.begin(); iter != projCornersInProj.end(); iter++)
	{
		std::cout << *iter << std::endl;
	}
	std::cout << "Projector corners in RGB image are at:" << std::endl;
	for (std::vector<cv::Point2f>::iterator iter = projCornersInRGB.begin(); iter != projCornersInRGB.end(); iter++)
	{
		std::cout << *iter << std::endl;
	}
	
	cv::Mat projCornersInRGBMat(2, 4, CV_32FC1);
	projCornersInRGBMat.at<float>(0, 0) = projCornersInRGB[0].x - projectedKeyPoints[0].X;
	projCornersInRGBMat.at<float>(1, 0) = projCornersInRGB[0].y - projectedKeyPoints[0].Y;
	projCornersInRGBMat.at<float>(0, 1) = projCornersInRGB[1].x - projectedKeyPoints[0].X;
	projCornersInRGBMat.at<float>(1, 1) = projCornersInRGB[1].y - projectedKeyPoints[0].Y;
	projCornersInRGBMat.at<float>(0, 2) = projCornersInRGB[2].x - projectedKeyPoints[0].X;
	projCornersInRGBMat.at<float>(1, 2) = projCornersInRGB[2].y - projectedKeyPoints[0].Y;
	projCornersInRGBMat.at<float>(0, 3) = projCornersInRGB[3].x - projectedKeyPoints[0].X;
	projCornersInRGBMat.at<float>(1, 3) = projCornersInRGB[3].y - projectedKeyPoints[0].Y;

	std::cout << "Projector's corners represented as vectors from center of projected origin is:" << std::endl;
	std::cout << projCornersInRGBMat << std::endl;

	cv::Mat weights (2, 4, CV_32FC1);
	weights = rgbToKeyPoints.inv() * projCornersInRGBMat;
	cv::Mat keyPointsVectors (3, 2, CV_32FC1);

	std::cout << "The weights matrix is:" << std::endl;
	std::cout << weights << std::endl;

	/*keyPointsVectors.at<float>(0, 0) = keyPoints[1].X - keyPoints[0].X;
	keyPointsVectors.at<float>(1, 0) = keyPoints[1].Y - keyPoints[0].Y;
	keyPointsVectors.at<float>(2, 0) = keyPoints[1].Z - keyPoints[0].Z;
	keyPointsVectors.at<float>(0, 1) = keyPoints[2].X - keyPoints[0].X;
	keyPointsVectors.at<float>(1, 1) = keyPoints[2].Y - keyPoints[0].Y;
	keyPointsVectors.at<float>(2, 1) = keyPoints[2].Z - keyPoints[0].Z;
	cv::Mat weightMultVectors(3, 4, CV_32FC1);

	cv::Mat origin(3, 4, CV_32FC1);
	origin.at<float>(0, 0) = keyPoints[0].X;
	origin.at<float>(0, 1) = keyPoints[0].X;
	origin.at<float>(0, 2) = keyPoints[0].X;
	origin.at<float>(0, 3) = keyPoints[0].X;
	origin.at<float>(1, 0) = keyPoints[0].Y;
	origin.at<float>(1, 1) = keyPoints[0].Y;
	origin.at<float>(1, 2) = keyPoints[0].Y;
	origin.at<float>(1, 3) = keyPoints[0].Y;
	origin.at<float>(2, 0) = keyPoints[0].Z;
	origin.at<float>(2, 1) = keyPoints[0].Z;
	origin.at<float>(2, 2) = keyPoints[0].Z;
	origin.at<float>(2, 3) = keyPoints[0].Z;
	
	weightMultVectors = keyPointsVectors * weights;
	cv::Mat projCornersInCam(3, 4, CV_32FC1);
	cv::add(origin, weightMultVectors, projCornersInCam);*/

	std::vector <cv::Point3f> cornerPoints;
	for (int i = 0; i < 4; i++)
	{
		float X, Y, Z;
		X = weights.at<float>(0, i)*keyPoints[1].X + weights.at<float>(1, i)*keyPoints[2].X + (1.f - weights.at<float>(0, i) - weights.at<float>(1, i))*keyPoints[0].X;
		Y = weights.at<float>(0, i)*keyPoints[1].Y + weights.at<float>(1, i)*keyPoints[2].Y + (1.f - weights.at<float>(0, i) - weights.at<float>(1, i))*keyPoints[0].Y;
		Z = weights.at<float>(0, i)*keyPoints[1].Z + weights.at<float>(1, i)*keyPoints[2].Z + (1.f - weights.at<float>(0, i) - weights.at<float>(1, i))*keyPoints[0].Z;
		cornerPoints.push_back(cv::Point3f(X, Y, Z));
	}
	/*
	cornerPoints.push_back(cv::Point3f(projCornersInCam.at<float>(0, 0), projCornersInCam.at<float>(1, 0), projCornersInCam.at<float>(2, 0)));
	cornerPoints.push_back(cv::Point3f(projCornersInCam.at<float>(0, 1), projCornersInCam.at<float>(1, 1), projCornersInCam.at<float>(2, 1)));
	cornerPoints.push_back(cv::Point3f(projCornersInCam.at<float>(0, 2), projCornersInCam.at<float>(1, 2), projCornersInCam.at<float>(2, 2)));
	cornerPoints.push_back(cv::Point3f(projCornersInCam.at<float>(0, 3), projCornersInCam.at<float>(1, 3), projCornersInCam.at<float>(2, 3)));
	*/
	std::cout << "Projected screen edge point pairs are: " << std::endl;
	planeNormals.clear();
	for (int i = 0; i < 4; i++)
	{
		std::cout << "(" << cornerPoints[i] << "," << cornerPoints[(i + 1) % 4] << ")" << std::endl;
		float X, Y, Z;
		X = cornerPoints[i].y * cornerPoints[(i + 1) % 4].z - cornerPoints[i].z * cornerPoints[(i + 1) % 4].y;
		Y = cornerPoints[i].z * cornerPoints[(i + 1) % 4].x - cornerPoints[i].x * cornerPoints[(i + 1) % 4].z;
		Z = cornerPoints[i].x * cornerPoints[(i + 1) % 4].y - cornerPoints[i].y * cornerPoints[(i + 1) % 4].x;
		planeNormals.push_back(cv::Point3f(X, Y, Z));
	}

	cv::Point3f testPoint(keyPoints[0].X, keyPoints[0].Y, keyPoints[0].Z);

	std::cout << "The sign corrected frustrum normals are:" << std::endl;
	for (int i = 0; i < 4; i++)
	{
		float a = planeNormals[i].x*keyPoints[0].X + planeNormals[i].y*keyPoints[0].Y + planeNormals[i].z*keyPoints[0].Z;
		float signOfA = a / abs(a);
		planeNormals[i].x = planeNormals[i].x*signOfA;
		planeNormals[i].y = planeNormals[i].y*signOfA;
		planeNormals[i].z = planeNormals[i].z*signOfA;
		std::cout << planeNormals[i] << std::endl;
	}

	// Log
	std::cout << "Finished settting the homography and computing the frustum" << std::endl;
}

// Routines
void personalRobotics::ObjectSegmentor::planeSegment()
{
	if (!pauseThreadFlag.get())
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
			{
			  static int count = 50;
			  if (--count < 0) { std::cout << "Segmentor still running" << std::endl; count = 50; }
			}
			// Copy the RGB image
			pointCloudMutex.lock();
			rgbMutex.lock();
			cv::Mat rgbImageCopy;
			rgbImage.copyTo(rgbImageCopy);
			rgbMutex.unlock();

			// Add plane based rejection
			// Convert points to pointcloud and plane segment as well
			for (size_t point = 0; point < numPoints; point++)
			{
				if (pointCloudPtr[point].Z > minThreshold && pointCloudPtr[point].Z < maxThreshold)
				{
					if ((planePtr->values[0] * pointCloudPtr[point].X + planePtr->values[1] * pointCloudPtr[point].Y + planePtr->values[2] * pointCloudPtr[point].Z + planePtr->values[3]) > distCutoff)
					{
						if ((planeNormals[0].x * pointCloudPtr[point].X + planeNormals[0].y * pointCloudPtr[point].Y + planeNormals[0].z * pointCloudPtr[point].Z) > 0)
						{
							if ((planeNormals[1].x * pointCloudPtr[point].X + planeNormals[1].y * pointCloudPtr[point].Y + planeNormals[1].z * pointCloudPtr[point].Z) > 0)
							{
								if ((planeNormals[2].x * pointCloudPtr[point].X + planeNormals[2].y * pointCloudPtr[point].Y + planeNormals[2].z * pointCloudPtr[point].Z) > 0)
								{
									if ((planeNormals[3].x * pointCloudPtr[point].X + planeNormals[3].y * pointCloudPtr[point].Y + planeNormals[3].z * pointCloudPtr[point].Z) > 0)
									{
										pclPtr->points[dstPoint].x = pointCloudPtr[point].X;
										pclPtr->points[dstPoint].y = pointCloudPtr[point].Y;
										pclPtr->points[dstPoint].z = pointCloudPtr[point].Z;
										dstPoint++;
									}
								}
							}
						}
					}
				}
			}

			pointCloudMutex.unlock();
			pclPtr->resize(dstPoint);

			if (dstPoint > 60000)
			{
				//std::cout << "rejecting frame, too many points: " << dstPoint << std::endl;
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

			// Vector to track valid clusters
			int validClusterCounter = 0;
			int allClusterCounter = 0;
			for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it)
			{
				// Convert PCL to kinect camera point representation
				allClusterCounter++;
				CameraSpacePoint *cameraSpacePoints = new CameraSpacePoint[it->indices.size()];
				int pointNum = 0;
				bool validCluster = true;
				for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
				{
					if (!onBoundingEdges(projDsCloud->points[*pit]))
						cameraSpacePoints[pointNum++] = { projDsCloud->points[*pit].x, projDsCloud->points[*pit].y, projDsCloud->points[*pit].z };
					else
					{
						validCluster = false;
						break;
					}
				}
				if (!validCluster)
				{
					delete[] cameraSpacePoints;
					continue;
				}

				validClusterCounter++;
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
					cv::Size2f objectBoundingSize = cv::Size2f(sqrtf(eigenValues.at<float>(0, 0))*7.5f, sqrtf(eigenValues.at<float>(1, 0))*7.5f);
					IDLookUp iLU;

					for (std::vector<IDLookUp>::iterator idPtr = previousIDList.begin(); idPtr != previousIDList.end(); idPtr++)
					{
						float currentScore = calculateEntityDifferences(idPtr->centroid, objectCentroid, idPtr->angle, objectAngle, idPtr->boundingSize, objectBoundingSize);
						if (currentScore < objectDifferenceThreshold && currentScore < minScore)
						{
							iLU.id = idPtr->id;
							if (currentScore < objectMovementThreshold)
							{
								iLU.numFramesSame = idPtr->numFramesSame + 1;
							}
							else
							{
								iLU.numFramesSame = 1;
							}
							minScore = currentScore;
							match = true;
						}
					}
					iLU.centroid = objectCentroid;
					iLU.angle = objectAngle;
					iLU.boundingSize = objectBoundingSize;
					if (!match)
					{
						objectCount++;
						iLU.id = objectCount;
						iLU.numFramesSame = 1;
					}
					currentIDList.push_back(iLU);
					entityList.push_back(personalRobotics::Entity(objectCentroid, objectAngle, objectBoundingSize, iLU.id));
				}
			}
			{
			  // FIXME: this is a simple decimation counter to reduce the amount of log output; this would be
			  // better implemented by a logging routine which can instead emit summaries only when the state changes,
			  // and then emit a time-stamped message with an event count.
			  static int count = 50;
			  if (--count < 0) {
			    count = 50;
			    std::cout << "Generated " << allClusterCounter << " cluster" << std::endl;
			    std::cout << "Generated " << validClusterCounter << " valid cluster" << std::endl;
			  }
			}
			// See if frames are static
			prevFrameStatic = frameStatic;
			frameStatic = calculateOverallChangeInFrames(currentIDList);
			previousIDList = currentIDList;

			// Generate patch and geometric data for each of the entity
			if (frameStatic && !prevFrameStatic)
			{
				for (std::vector<personalRobotics::Entity>::iterator entityPtr = entityList.begin(); entityPtr != entityList.end(); entityPtr++)
				{
					entityPtr->generateData(homography, rgbImageCopy);
				}
				newListGenerated.set(true);
				std::cout << "New static frame generated." << std::endl;
			}
			unlockList();
			currentIDList.clear();
		}
		else
			Sleep(15);
	}
	else
		Sleep(20);
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

	std::cout << "findTablePlane() computed plane values: "
		  << planePtr->values[0] << " "
		  << planePtr->values[1] << " "
		  << planePtr->values[2] << " "
		  << planePtr->values[3] << std::endl;
	
	// Find pixel size
	CameraSpacePoint keyPoints[3];
	ColorSpacePoint projectedKeyPoints[3];
	keyPoints[0] = { 0, 0, (-1 * (planePtr->values[3] + planePtr->values[0] * 0 + planePtr->values[1] * 0) / planePtr->values[2]) };		//(0  , 0   ,z1)
	keyPoints[1] = { 0.1, 0, (-1 * (planePtr->values[3] + planePtr->values[0] * 0.1 + planePtr->values[1] * 0) / planePtr->values[2]) };	//(0.1, 0   ,z2)
	keyPoints[2] = { 0, 0.1, (-1 * (planePtr->values[3] + planePtr->values[0] * 0 + planePtr->values[1] * 0.1) / planePtr->values[2]) };	//(0  , 0.1 ,z3)
	coordinateMapperPtr->MapCameraPointsToColorSpace(3, keyPoints, 3, projectedKeyPoints);
	double delX = sqrt((projectedKeyPoints[1].X - projectedKeyPoints[0].X)*(projectedKeyPoints[1].X - projectedKeyPoints[0].X) + (projectedKeyPoints[1].Y - projectedKeyPoints[0].Y)*(projectedKeyPoints[1].Y - projectedKeyPoints[0].Y));
	double delY = sqrt((projectedKeyPoints[2].X - projectedKeyPoints[0].X)*(projectedKeyPoints[2].X - projectedKeyPoints[0].X) + (projectedKeyPoints[2].Y - projectedKeyPoints[0].Y)*(projectedKeyPoints[2].Y - projectedKeyPoints[0].Y));

	// Value of 0.1 used above is in meters, to get pixel size in mm, divide 100 by delX and delY
	rgbPixelSize.x = 100 / delX;
	rgbPixelSize.y = 100 / delY;

	std::cout << "findTablePlane() computed rgbPixelSize: (" << rgbPixelSize.x << ", " << rgbPixelSize.y <<")"<< std::endl;
	// Return
	return true;
}
void personalRobotics::ObjectSegmentor::startSegmentor()
{
	std::cout << "Starting object segmentor thread" << std::endl;
	pauseThreadFlag.set(false);
	stopSegmentorFlag.set(false);
	segementorThread = std::thread(&personalRobotics::ObjectSegmentor::segmentorThreadRoutine, this);
	std::cout << "Started object segmentor thread" << std::endl;
}
void personalRobotics::ObjectSegmentor::segmentorThreadRoutine()
{
	while (!stopSegmentorFlag.get())
	{
		planeSegment();
		Sleep(25);
	}
}
void personalRobotics::ObjectSegmentor::stopSegmentor()
{
	std::cout << "Stopping object segmentor thread" << std::endl;
	stopSegmentorFlag.set(true);
	pauseThreadFlag.set(true);
	if (segementorThread.joinable())
		segementorThread.join();
	std::cout << "Stopped object segmentor thread" << std::endl;
}
void personalRobotics::ObjectSegmentor::pauseSegmentor()
{
	pauseThreadFlag.set(true);
	std::cout << "Paused object segmentor thread" << std::endl;
}
void personalRobotics::ObjectSegmentor::resumeSegmentor()
{
	pauseThreadFlag.set(false);
	std::cout << "Resumed object segmentor thread" << std::endl;
}

// Helper functions
float personalRobotics::ObjectSegmentor::calculateEntityDifferences(cv::Point2f IDcentroid, cv::Point2f objectCentroid, float IDangle, float objectAngle, cv::Size2f IDBoundingSize, cv::Size2f objectBoundingSize)
{
	//if (abs(IDBoundingSize.width*IDBoundingSize.height - objectBoundingSize.width*objectBoundingSize.height) >= 50)
	//	return 10000;
	//else
	//{
		return sqrt( (pow((IDcentroid.x - objectCentroid.x), 2) + pow((IDcentroid.y - objectCentroid.y), 2)) + 0*(pow((IDangle - objectAngle), 1)) + 0*(pow((IDBoundingSize.width - objectBoundingSize.width), 2) + pow((IDBoundingSize.height - objectBoundingSize.height), 2)));
	//}
}
bool personalRobotics::ObjectSegmentor::calculateOverallChangeInFrames(std::vector<personalRobotics::IDLookUp> cIDList)
{
	for (std::vector<IDLookUp>::iterator cIDPtr = cIDList.begin(); cIDPtr != cIDList.end(); cIDPtr++)
	{
		if (cIDPtr->numFramesSame < 5)
		{ 
			return false;
		}
	}
	return true;
}
bool personalRobotics::ObjectSegmentor::onBoundingEdges(pcl::PointXYZ point)
{
	if ((planeNormals[0].x * point.x + planeNormals[0].y * point.y + planeNormals[0].z * point.z) < 0.05)
		return true;
	if ((planeNormals[1].x * point.x + planeNormals[1].y * point.y + planeNormals[1].z * point.z) < 0.05)
		return true;
	if ((planeNormals[2].x * point.x + planeNormals[2].y * point.y + planeNormals[2].z * point.z) < 0.05)
		return true;
	if ((planeNormals[3].x * point.x + planeNormals[3].y * point.y + planeNormals[3].z * point.z) < 0.05)
		return true;
	
	return false;
}
void personalRobotics::createCheckerboard(cv::Mat& checkerboard, int width, int height, int& numBlocksX, int& numBlocksY)
{
	checkerboard.create(height, width, CV_8UC1);
	int blockSize = (personalRobotics::gcd(height, width));
	numBlocksX = width / blockSize;
	numBlocksY = height / blockSize;
	while (numBlocksX < 6 || numBlocksY < 6)
	{
		if (blockSize % 2 != 0)
			std::cout << "Deadlock! check or change the resolution of display" << std::endl;
		else
		{
			blockSize /= 2;
			numBlocksX = width / blockSize;
			numBlocksY = height / blockSize;
		}
	}
	std::cout << "Creating checkerboard with width: " << width << " height: " << height << " block size: " << blockSize << std::endl;
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
